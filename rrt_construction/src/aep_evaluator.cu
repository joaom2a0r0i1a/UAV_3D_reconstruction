#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_constants.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

// ==========================================
// 1. CONFIGURATION STRUCT
// ==========================================
// This holds all the dynamic parameters.
// We define it here so both Host and Device can see it.
struct KernelParams {
    float voxel_size;
    float gain_range;     // r_max
    float fov_y_rad;      // Horizontal FOV
    float fov_p_rad;      // Vertical FOV
    float camera_pitch;   // Pitch offset
    
    float dtheta;         // Angular step (yaw)
    float dphi;           // Angular step (pitch)
    
    float phi_start;
    float phi_end;
};

// Map Definitions
#define V_FREE 0
#define V_OCCUPIED 1
#define V_UNKNOWN 2

// Angular discretization (degrees)
#define DTHETA_DEG 2
#define DPHI_DEG   2

// Angular bins
#define THETA_BINS (360 / DTHETA_DEG)   // 36
#define PHI_BINS   (180 / DPHI_DEG)     // 18

#define TOTAL_RAYS (THETA_BINS * PHI_BINS)
#define MAX_THREADS_PER_BLOCK 512

// ==========================================
// 2. THE KERNEL (The Worker)
// ==========================================
__global__ void evaluate_aep_kernel_single(
    const uint8_t* __restrict__ map,
    const int3 map_dim,
    const float3 map_origin,
    const float3 candidate_pos,
    float* __restrict__ result_gain,
    float* __restrict__ result_yaw,
    KernelParams params) {
    // Shared memory to accumulate gains per yaw sector
    __shared__ float s_yaw_gains[THETA_BINS];

    int tid = threadIdx.x;

    // 1. Initialize Shared Memory
    if (tid < THETA_BINS) {
        s_yaw_gains[tid] = 0.0f;
    }

    __syncthreads();

    // 2. Calculate Ray Counts
    int rows_in_fov = (int)ceilf(params.fov_p_rad / params.dphi);
    if (rows_in_fov < 1) rows_in_fov = 1;
    
    int rays_total = THETA_BINS * rows_in_fov;

    // 3. Parallel Raycast Loop (Block-Stride Loop)
    // Multiple threads work together to cover all rays for this ONE candidate
    for (int idx = tid; idx < rays_total; idx += blockDim.x) {
        
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS; 
        
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float dir_x = cosf(theta) * sin_phi;
        float dir_y = sinf(theta) * sin_phi;
        float dir_z = cosf(phi);

        // --- DDA SETUP ---
        float gx = (candidate_pos.x - map_origin.x) / params.voxel_size;
        float gy = (candidate_pos.y - map_origin.y) / params.voxel_size;
        float gz = (candidate_pos.z - map_origin.z) / params.voxel_size;

        int ix = floor(gx);
        int iy = floor(gy);
        int iz = floor(gz);

        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        float tMaxX, tMaxY, tMaxZ;
        if (stepX > 0) tMaxX = (ix + 1.0f - gx) * tDeltaX; else tMaxX = (gx - ix) * tDeltaX;
        if (stepY > 0) tMaxY = (iy + 1.0f - gy) * tDeltaY; else tMaxY = (gy - iy) * tDeltaY;
        if (stepZ > 0) tMaxZ = (iz + 1.0f - gz) * tDeltaZ; else tMaxZ = (gz - iz) * tDeltaZ;

        float ray_gain = 0.0f;
        float t = 0.0f;
        float max_t = params.gain_range / params.voxel_size;

        // --- DDA LOOP ---
        while (t < max_t) {
            if (ix >= 0 && ix < map_dim.x &&
                iy >= 0 && iy < map_dim.y &&
                iz >= 0 && iz < map_dim.z) 
            {
                int flat_idx = iz * (map_dim.x * map_dim.y) + iy * map_dim.x + ix;
                uint8_t val = map[flat_idx];

                if (val == V_OCCUPIED) {
                    break;
                } 
                else if (val == V_UNKNOWN) {
                    float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                    float dt = t_exit - t; 
                    float dr = dt * params.voxel_size;
                    float r = t * params.voxel_size;
                    float term1 = 2.0f * r * r * dr;
                    float term2 = (dr * dr * dr) / 6.0f;
                    ray_gain += (term1 + term2) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }

            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                ix += stepX; 
                t = tMaxX; 
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                iy += stepY; 
                t = tMaxY; 
                tMaxY += tDeltaY;
            } else {
                iz += stepZ; 
                t = tMaxZ; 
                tMaxZ += tDeltaZ;
            }
        }

        // Accumulate gain into shared memory (Atomic because multiple rays map to same yaw bin)
        if (ray_gain > 0.0f) {
            atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
        }
    }

    __syncthreads();

    // 4. Sliding Window Optimization (Single Thread)
    if (tid == 0) {
        float max_gain = 0.0f;
        int best_start_idx = 0;
        
        int sectors_in_fov = (int)(params.fov_y_rad / params.dtheta);
        if (sectors_in_fov < 1) sectors_in_fov = 1;

        for (int i = 0; i < THETA_BINS; ++i) {
            float current_window_gain = 0.0f;
            for (int k = 0; k < sectors_in_fov; ++k) {
                int idx = (i + k) % THETA_BINS;
                current_window_gain += s_yaw_gains[idx];
            }
            if (current_window_gain > max_gain) {
                max_gain = current_window_gain;
                best_start_idx = i;
            }
        }

        // Write Final Result
        *result_gain = max_gain;
        
        float start_angle = -CUDART_PI_F + (best_start_idx * params.dtheta);
        float center_angle = start_angle + (params.fov_y_rad * 0.5f);
        if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
        
        *result_yaw = center_angle;
    }
}

__global__ void evaluate_aep_kernel(
    const uint8_t* __restrict__ map,
    const int3 map_dim,
    const float3 map_origin,
    const float3* __restrict__ positions,
    float* __restrict__ results_gain,
    float* __restrict__ results_yaw,
    KernelParams params) {

    __shared__ float s_yaw_gains[THETA_BINS];

    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;

    if (ray_id < THETA_BINS) {
        s_yaw_gains[ray_id] = 0.0f;
    }

    __syncthreads();

    int rows_in_fov = (int)ceilf(params.fov_p_rad / params.dphi);
    if (rows_in_fov < 1) {
        rows_in_fov = 1;
    }

    // The total number of rays we actually calculate per candidate
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int ray_id = threadIdx.x; ray_id < rays_per_candidate; ray_id += blockDim.x) {
    //for (int ray_id = threadIdx.x; ray_id < TOTAL_RAYS; ray_id += blockDim.x) {
        int theta_idx = ray_id % THETA_BINS;
        int phi_idx   = ray_id / THETA_BINS;
        //if (phi_idx >= PHI_BINS) continue;

        // Compute angles
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float3 cam_pos = positions[candidate];

        float sin_phi = sinf(phi);
        float dir_x = cosf(theta) * sin_phi;
        float dir_y = sinf(theta) * sin_phi;
        float dir_z = cosf(phi);

        // Origin in Voxel Coordinates
        float gx = (cam_pos.x - map_origin.x) / params.voxel_size;
        float gy = (cam_pos.y - map_origin.y) / params.voxel_size;
        float gz = (cam_pos.z - map_origin.z) / params.voxel_size;

        // Integer start indices
        int ix = floor(gx);
        int iy = floor(gy);
        int iz = floor(gz);

        // Step Direction
        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        // tDelta: Distance along ray to cross one voxel in each dimension
        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        // tMax: Distance to the *next* boundary
        float tMaxX, tMaxY, tMaxZ;

        if (stepX > 0) {
            tMaxX = (ix + 1.0f - gx) * tDeltaX;
        } else {
            tMaxX = (gx - ix) * tDeltaX;
        }

        if (stepY > 0) {
            tMaxY = (iy + 1.0f - gy) * tDeltaY;
        } else {
            tMaxY = (gy - iy) * tDeltaY;
        }

        if (stepZ > 0) {
            tMaxZ = (iz + 1.0f - gz) * tDeltaZ;
        } else {
            tMaxZ = (gz - iz) * tDeltaZ;
        }

        float ray_gain = 0.0f;
        float t = 0.0f;
        float max_t = params.gain_range / params.voxel_size;

        while (t < max_t) {
            if (ix >= 0 && ix < map_dim.x &&
                iy >= 0 && iy < map_dim.y &&
                iz >= 0 && iz < map_dim.z) {
                
                int flat_idx = iz * (map_dim.x * map_dim.y) + iy * map_dim.x + ix;
                uint8_t val = map[flat_idx];

                if (val == V_OCCUPIED) {
                    break;
                } 
                else if (val == V_UNKNOWN) {
                    float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                    float dt = t_exit - t; 
                    float dr = dt * params.voxel_size;

                    float r = t * params.voxel_size;
                    float term1 = 2.0f * r * r * dr;
                    float term2 = (dr * dr * dr) / 6.0f;
                    ray_gain += (term1 + term2) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }

            // Step to Next Voxel
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                ix += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                iy += stepY;
                t = tMaxY;
                tMaxY += tDeltaY;
            } else {
                iz += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }

        if (ray_gain > 0.0f) {
            atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
        }
    }

    __syncthreads();

    // -----------------------------------------------------------
    //    Sliding Window Optimization (Single Thread per Block)
    // -----------------------------------------------------------
    if (ray_id == 0) {
        float max_gain = 0.0f;
        int best_start_idx = 0;

        // How many 10-degree strips fit in my Horizontal FOV?
        int sectors_in_fov = (int)(params.fov_y_rad / params.dtheta);
        if (sectors_in_fov < 1) {
            sectors_in_fov = 1;
        }

        // Slide the window 0 to 36
        for (int i = 0; i < THETA_BINS; ++i) {
            float current_window_gain = 0.0f;
            
            // Sum up the strips inside the window
            for (int k = 0; k < sectors_in_fov; ++k) {
                int idx = (i + k) % THETA_BINS; // Wrap around (35 -> 0)
                current_window_gain += s_yaw_gains[idx];
            }

            // Keep track of the winner
            if (current_window_gain > max_gain) {
                max_gain = current_window_gain;
                best_start_idx = i;
            }
        }

        // 7. Write Final Result to Global Memory
        results_gain[candidate] = max_gain;
        
        // Calculate the center angle of the best window
        float start_angle = -CUDART_PI_F + (best_start_idx * params.dtheta);
        float center_angle = start_angle + (params.fov_y_rad * 0.5f);
        
        // Normalize angle to -PI to PI
        if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
        
        results_yaw[candidate] = center_angle;
    }
}

__global__ void evaluate_aep_kernel_depth(
    const uint8_t* __restrict__ map,
    const int3 map_dim,
    const float3 map_origin,
    const float3* __restrict__ positions,
    float* __restrict__ results_gain,
    float* __restrict__ results_yaw,
    float* __restrict__ depth_buffer_all,
    float* __restrict__ depth_buffer,
    KernelParams params) {

    __shared__ float s_yaw_gains[THETA_BINS];

    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;

    if (ray_id < THETA_BINS) {
        s_yaw_gains[ray_id] = 0.0f;
    }

    __syncthreads();

    int rows_in_fov = (int)ceilf(params.fov_p_rad / params.dphi);
    if (rows_in_fov < 1) {
        rows_in_fov = 1;
    }

    // The total number of rays we actually calculate per candidate
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int ray_id = threadIdx.x; ray_id < rays_per_candidate; ray_id += blockDim.x) {
        int theta_idx = ray_id % THETA_BINS;
        int phi_idx   = ray_id / THETA_BINS;

        // Compute angles
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float3 cam_pos = positions[candidate];

        float sin_phi = sinf(phi);
        float dir_x = cosf(theta) * sin_phi;
        float dir_y = sinf(theta) * sin_phi;
        float dir_z = cosf(phi);

        // Origin in Voxel Coordinates
        float gx = (cam_pos.x - map_origin.x) / params.voxel_size;
        float gy = (cam_pos.y - map_origin.y) / params.voxel_size;
        float gz = (cam_pos.z - map_origin.z) / params.voxel_size;

        // Integer start indices
        int ix = floor(gx);
        int iy = floor(gy);
        int iz = floor(gz);

        // Step Direction
        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        // tDelta: Distance along ray to cross one voxel in each dimension
        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        // tMax: Distance to the *next* boundary
        float tMaxX, tMaxY, tMaxZ;

        if (stepX > 0) {
            tMaxX = (ix + 1.0f - gx) * tDeltaX;
        } else {
            tMaxX = (gx - ix) * tDeltaX;
        }

        if (stepY > 0) {
            tMaxY = (iy + 1.0f - gy) * tDeltaY;
        } else {
            tMaxY = (gy - iy) * tDeltaY;
        }

        if (stepZ > 0) {
            tMaxZ = (iz + 1.0f - gz) * tDeltaZ;
        } else {
            tMaxZ = (gz - iz) * tDeltaZ;
        }

        float ray_gain = 0.0f;
        float t = 0.0f;
        float max_t = params.gain_range / params.voxel_size;
        float final_depth = params.gain_range;

        while (t < max_t) {
            if (ix >= 0 && ix < map_dim.x &&
                iy >= 0 && iy < map_dim.y &&
                iz >= 0 && iz < map_dim.z) {
                
                int flat_idx = iz * (map_dim.x * map_dim.y) + iy * map_dim.x + ix;
                uint8_t val = map[flat_idx];

                if (val == V_OCCUPIED) {
                    final_depth = t * params.voxel_size;
                    break;
                } 
                else if (val == V_UNKNOWN) {
                    float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                    float dt = t_exit - t; 
                    float dr = dt * params.voxel_size;

                    float r = t * params.voxel_size;
                    float term1 = 2.0f * r * r * dr;
                    float term2 = (dr * dr * dr) / 6.0f;
                    ray_gain += (term1 + term2) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }

            // Step to Next Voxel
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                ix += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                iy += stepY;
                t = tMaxY;
                tMaxY += tDeltaY;
            } else {
                iz += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }

        int global_ray_idx = candidate * rays_per_candidate + ray_id;
        depth_buffer_all[global_ray_idx] = final_depth;

        if (ray_gain > 0.0f) {
            atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
        }
    }

    __syncthreads();

    // -----------------------------------------------------------
    //    Sliding Window Optimization (Single Thread per Block)
    // -----------------------------------------------------------
    if (ray_id == 0) {
        float max_gain = 0.0f;
        int best_start_idx = 0;

        // How many 10-degree strips fit in my Horizontal FOV?
        int sectors_in_fov = (int)(params.fov_y_rad / params.dtheta);
        if (sectors_in_fov < 1) {
            sectors_in_fov = 1;
        }

        // Slide the window 0 to 36
        for (int i = 0; i < THETA_BINS; ++i) {
            float current_window_gain = 0.0f;
            
            // Sum up the strips inside the window
            for (int k = 0; k < sectors_in_fov; ++k) {
                int idx = (i + k) % THETA_BINS; // Wrap around (35 -> 0)
                current_window_gain += s_yaw_gains[idx];
            }

            // Keep track of the winner
            if (current_window_gain > max_gain) {
                max_gain = current_window_gain;
                best_start_idx = i;
            }
        }

        // 7. Write Final Result to Global Memory
        results_gain[candidate] = max_gain;
        
        // Calculate the center angle of the best window
        float start_angle = -CUDART_PI_F + (best_start_idx * params.dtheta);
        float center_angle = start_angle + (params.fov_y_rad * 0.5f);
        
        // Normalize angle to -PI to PI
        if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
        
        results_yaw[candidate] = center_angle;

        // Copy relevant depth buffer portion

        int out_stride = sectors_in_fov * rows_in_fov;
        int my_out = candidate * out_stride;
        int my_in  = candidate * rays_per_candidate;

        for (int phi_idx = 0; phi_idx < rows_in_fov; phi_idx++) {
            int row_start = phi_idx * THETA_BINS;
            for (int theta_idx = 0; theta_idx < sectors_in_fov; theta_idx++) {
                int global_theta_idx = my_in + ((best_start_idx + theta_idx) % THETA_BINS);

                int global_ray_idx = global_theta_idx + row_start;
                int local_ray_idx = my_out + theta_idx + (phi_idx * sectors_in_fov);

                depth_buffer[local_ray_idx] = depth_buffer_all[global_ray_idx];
            }
        }
    }
}

// --- Helper: Get Linear Interpolation of 1/Z ---
__device__ inline float get_ray_depth_at_pixel(
    float start_inv_z, float end_inv_z, 
    float total_dist_pixels, float current_dist_pixels) {
    if (total_dist_pixels < 1.0f) return 1.0f / start_inv_z;
    
    float alpha = current_dist_pixels / total_dist_pixels;
    float current_inv_z = (1.0f - alpha) * start_inv_z + alpha * end_inv_z;
    return 1.0f / current_inv_z;
}

// Helper: Liang-Barsky Line Clipping
// returns true if segment intersects, updates t0 and t1 (normalized 0..1 relative to segment)
__device__ bool clip_line_2d(float u0, float v0, float u1, float v1, 
                             float w_min, float w_max, float h_min, float h_max, 
                             float* s0, float* s1) {
    float dx = u1 - u0;
    float dy = v1 - v0;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {u0 - w_min, w_max - u0, v0 - h_min, h_max - v0};

    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0f) { // Parallel to border
            if (q[i] < 0.0f) return false; // Outside
        } else {
            float r = q[i] / p[i];
            if (p[i] < 0.0f) { // Entering
                if (r > *s1) return false;
                if (r > *s0) *s0 = r;
            } else { // Exiting
                if (r < *s0) return false;
                if (r < *s1) *s1 = r;
            }
        }
    }
    return true;
}

__device__ float3 compute_skip_distance(
    int p_width, int p_height,
    float fx, float fy, float cx, float cy, 
    float3 parent_pos, float3 R0, float3 R1, float3 R2,
    const float* __restrict__ parent_depth_buffer,
    float3 ray_start, float3 ray_dir, float max_dist) {

    const float z_near = 0.1f;
    const float z_far  = max_dist;

    // --- 1. Transform Ray to Camera Frame ---
    float3 diff = make_float3(ray_start.x - parent_pos.x, 
                              ray_start.y - parent_pos.y, 
                              ray_start.z - parent_pos.z);
    
    float3 O; 
    O.x = R0.x*diff.x + R0.y*diff.y + R0.z*diff.z;
    O.y = R1.x*diff.x + R1.y*diff.y + R1.z*diff.z;
    O.z = R2.x*diff.x + R2.y*diff.y + R2.z*diff.z;

    float3 D;
    D.x = R0.x*ray_dir.x + R0.y*ray_dir.y + R0.z*ray_dir.z;
    D.y = R1.x*ray_dir.x + R1.y*ray_dir.y + R1.z*ray_dir.z;
    D.z = R2.x*ray_dir.x + R2.y*ray_dir.y + R2.z*ray_dir.z;

    // --- 2. Z-Clipping (Slab Method) ---
    float t0 = 0.0f;
    float t1 = max_dist;

    if (fabsf(D.z) < 1e-6f) {
        if (O.z < z_near || O.z > z_far) return make_float3(-1.0f, -1.0f, 0.0f);
    } else {
        float inv_Dz = 1.0f / D.z;
        float t_near = (z_near - O.z) * inv_Dz;
        float t_far  = (z_far  - O.z) * inv_Dz;

        float t_min = fminf(t_near, t_far);
        float t_max = fmaxf(t_near, t_far);

        t0 = fmaxf(t0, t_min);
        t1 = fminf(t1, t_max);
    }

    if (t0 >= t1) return make_float3(-1.0f, -1.0f, 0.0f);

    float3 P_start = make_float3(O.x + t0*D.x, O.y + t0*D.y, O.z + t0*D.z);
    float3 P_end   = make_float3(O.x + t1*D.x, O.y + t1*D.y, O.z + t1*D.z);

    // --- 3. Project to Pixels ---
    float inv_z0 = 1.0f / P_start.z;
    float inv_z1 = 1.0f / P_end.z;

    float u0 = fx * P_start.x * inv_z0 + cx;
    float v0 = fy * P_start.y * inv_z0 + cy;
    float u1 = fx * P_end.x   * inv_z1 + cx;
    float v1 = fy * P_end.y   * inv_z1 + cy;

    // --- 4. 2D Screen Clipping (Liang-Barsky) ---
    float s_min = 0.0f;
    float s_max = 1.0f;
    
    // Check against full width/height
    if (!clip_line_2d(u0, v0, u1, v1, 
                      0.0f, (float)p_width, 
                      0.0f, (float)p_height, 
                      &s_min, &s_max)) {
        return make_float3(-1.0f, -1.0f, 0.0f);
    }

    // --- 5. Compute Exact Frustum Interval [t_visible_start, t_visible_end] ---
    // A. Interpolate 1/z (Linear in screen space) at the clip boundaries
    float w_start = inv_z0 + s_min * (inv_z1 - inv_z0);
    float w_end   = inv_z0 + s_max * (inv_z1 - inv_z0);

    // B. Recover t (Meters)
    float t_visible_start; 
    float t_visible_end;

    if (fabsf(D.z) > 1e-6f) {
        float z_s = 1.0f / w_start;
        float z_e = 1.0f / w_end;
        t_visible_start = (z_s - O.z) / D.z;
        t_visible_end   = (z_e - O.z) / D.z;
    } else {
        t_visible_start = t0 + s_min * (t1 - t0);
        t_visible_end   = t0 + s_max * (t1 - t0);
    }
    
    /*printf("DEBUG_ROT | W_Cam:%.2f,%.2f,%.2f | W_RStart:%.2f,%.2f,%.2f | W_RDir:%.2f,%.2f,%.2f | CamO:%.2f,%.2f,%.2f | CamD:%.2f,%.2f,%.2f | ZSlab:%.2f,%.2f | z:%.2f,%.2f | uv:(%.1f,%.1f)->(%.1f,%.1f) | uv_clip:(%.1f,%.1f)->(%.1f,%.1f) | s:%.3f,%.3f | FINAL_t:%.3f,%.3f (Mode:%d)\n",
           parent_pos.x, parent_pos.y, parent_pos.z,
           ray_start.x, ray_start.y, ray_start.z,
           ray_dir.x, ray_dir.y, ray_dir.z,
           O.x, O.y, O.z, 
           D.x, D.y, D.z,
           t0, t1, 
           safe_z0, safe_z1,
           u0, v0, u1, v1,
           u_c0, v_c0, u_c1, v_c1, 
           s_min, s_max,
           t_visible_start, t_visible_end,
           (int)use_perspective);*/

    //return make_float3(t_visible_start, t_visible_end, 0.0f);

    // --- 6. Apply 2D Clipping to our Variables ---
    float u_start = u0 + s_min * (u1 - u0);
    float v_start = v0 + s_min * (v1 - v0);
    float u_end   = u0 + s_max * (u1 - u0);
    float v_end   = v0 + s_max * (v1 - v0);

    // --- 7. Woo's DDA on Clipped Segment ---
    int x = floor(u_start);
    int y = floor(v_start);
    int x_end = floor(u_end);
    int y_end = floor(v_end);

    // Clamp Start
    x = max(0, min(x, p_width - 1));
    y = max(0, min(y, p_height - 1));

    // Clamp End
    x_end = max(0, min(x_end, p_width - 1));
    y_end = max(0, min(y_end, p_height - 1));

    // Step Direction
    int stepX = (u_end > u_start) ? 1 : ((u_end < u_start) ? -1 : 0);
    int stepY = (v_end > v_start) ? 1 : ((v_end < v_start) ? -1 : 0);

    // tDelta: Distance along ray to cross one voxel in each dimension
    float dx = u_end - u_start;
    float dy = v_end - v_start;
    float tDeltaX = (dx != 0.0f) ? fabsf(1.0f / dx) : 1e30f;
    float tDeltaY = (dy != 0.0f) ? fabsf(1.0f / dy) : 1e30f;

    // tMax: Distance to the *next* boundary
    float tMaxX, tMaxY;

    if (stepX > 0) {
        tMaxX = (floor(u_start) + 1.0f - u_start) * tDeltaX;
    } else {
        tMaxX = (u_start - floor(u_start)) * tDeltaX;
    }

    if (stepY > 0) {
        tMaxY = (floor(v_start) + 1.0f - v_start) * tDeltaY;
    } else {
        tMaxY = (v_start - floor(v_start)) * tDeltaY;
    }

    // Total distance in pixels for 1/z interpolation
    float current_t = 0.0f;
    float w_curr = w_start;
    bool hit_obstacle_in_depth = false;
    bool hit_any_limit = false;

    float status = 1.0f;

    // --- 8. Step through every pixel along the ray ---
    while (current_t <= 1.0f) {
        if (x >= 0 && x < p_width && y >= 0 && y < p_height) {
            w_curr = w_start + current_t * (w_end - w_start);
            float my_z = 1.0f / w_curr;
            float parent_z = parent_depth_buffer[y * p_width + x];
            if (parent_z <= (my_z + 0.05f)) {
                hit_any_limit = true;
                if (parent_z < (max_dist - 0.1f) && parent_z > 0.1f) { 
                    status = -1.0f;
                    hit_obstacle_in_depth = true;
                }
                break;
            }
        } else {
            hit_any_limit = true;
            break;
        }

        if (x == x_end && y == y_end) break;

        if (tMaxX < tMaxY) {
            x += stepX;
            current_t = tMaxX;
            tMaxX += tDeltaX;
        } else {
            y += stepY;
            current_t = tMaxY;
            tMaxY += tDeltaY;
        }
    }

    if (!hit_any_limit) {
        w_curr = w_end;
        current_t = 1.0f;
    }

    // --- 9. Result Recovery ---
    float t_hit;

    if (fabsf(D.z) > 1e-6f) {
        float z_hit = 1.0f / w_curr;
        t_hit = (z_hit - O.z) / D.z;
    } else {
        t_hit = t_visible_start + current_t * (t_visible_end - t_visible_start);
    }

    return make_float3(t_visible_start, t_hit, status);
}

__global__ void evaluate_marginal_gain_kernel(
    const uint8_t* __restrict__ map,
    const int3 map_dim,
    const float3 map_origin,
    const float3* __restrict__ positions,
    float3 parent_pos,
    float parent_yaw,
    const float* __restrict__ parent_depth_buffer,
    float3 R0, float3 R1, float3 R2,
    int p_width, int p_height,
    float fx, float fy, float cx, float cy,
    float* __restrict__ results_gain,
    float* __restrict__ results_yaw,
    float* __restrict__ depth_buffer_all,
    float* __restrict__ depth_buffer,
    KernelParams params) {
    
    __shared__ float s_yaw_gains[THETA_BINS];

    //bool debug = (threadIdx.x == 0 && blockIdx.x == 0);

    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;

    if (ray_id < THETA_BINS) {
        s_yaw_gains[ray_id] = 0.0f;
    }

    __syncthreads();

    int rows_in_fov = (int)(params.fov_p_rad / params.dphi);
    if (rows_in_fov < 1) {
        rows_in_fov = 1;
    }
    int sectors_in_fov = (int)(params.fov_y_rad / params.dtheta);
    if (sectors_in_fov < 1) {
        sectors_in_fov = 1;
    }

    // The total number of rays we actually calculate per candidate
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    // ---------------------------------------------------------
    // 1. RAYCAST LOOP
    // ---------------------------------------------------------
    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {

        //if (debug && idx == 0) printf("[GPU] Processing Ray 0...\n");

        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS; 
        
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float3 cam_pos = positions[candidate];

        float sin_phi = sinf(phi);
        float dir_x = cosf(theta) * sin_phi;
        float dir_y = sinf(theta) * sin_phi;
        float dir_z = cosf(phi);

        //if (debug && idx == 0) printf("[GPU] Ray Dir: %.2f, %.2f, %.2f\n", dir_x, dir_y, dir_z);

        float3 ray_dir = make_float3(dir_x, dir_y, dir_z);

        // ---------------------------------------------------------
        // 2. MARGINAL GAIN CHECK (Skip Logic)
        // ---------------------------------------------------------
        // 2.1. Origin relative to Parent
        float3 skip_interval = compute_skip_distance(sectors_in_fov, rows_in_fov,
                                                fx, fy, cx, cy, 
                                                parent_pos, R0, R1, R2,
                                                parent_depth_buffer,
                                                cam_pos, ray_dir, params.gain_range);

        // Convert Skip Interval from Meters to Voxels
        float skip_len_m = skip_interval.y - skip_interval.x;
        float skip_start_vox = (skip_interval.x != -1.0f) ? (skip_interval.x / params.voxel_size) : -1.0f;
        float skip_end_vox   = (skip_interval.x != -1.0f) ? (skip_interval.y / params.voxel_size) : -1.0f;
        float status = skip_interval.z;

        float t = 0.0f;
        float max_t = params.gain_range / params.voxel_size;
        float final_depth = params.gain_range;
        bool has_jumped = false;

        /*if (skip_interval.x != -1.0f) {
            if (status == -1.0f) {
                float wall_dist_vox = skip_end_vox;

                if (wall_dist_vox < max_t) {
                    max_t = wall_dist_vox;
                    final_depth = skip_interval.y; // Record exact depth
                }
            }

            if (skip_start_vox <= 1.0f) {
                t = skip_end_vox;
                has_jumped = true;
            }
        }*/

        //if (debug && idx == 0) printf("[GPU] Ray %d: Skip Interval (m): [%.2f, %.2f], (vox): [%.2f, %.2f], (status): %.2f \n", 
        //        idx, skip_interval.x, skip_interval.y, skip_start_vox, skip_end_vox, status);

        // ---------------------------------------------------------
        // 3. RAYCASTING 3D WOO DDA
        // ---------------------------------------------------------

        // Origin in Voxel Coordinates
        float gx = (cam_pos.x - map_origin.x) / params.voxel_size + (dir_x * t);
        float gy = (cam_pos.y - map_origin.y) / params.voxel_size + (dir_y * t);
        float gz = (cam_pos.z - map_origin.z) / params.voxel_size + (dir_z * t);

        // Integer start indices
        int ix = floor(gx);
        int iy = floor(gy);
        int iz = floor(gz);

        // Step Direction
        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        // tDelta: Distance along ray to cross one voxel in each dimension
        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        // tMax: Distance to the *next* boundary
        float tMaxX, tMaxY, tMaxZ;

        if (stepX > 0) {
            tMaxX = (ix + 1.0f - gx) * tDeltaX;
        } else {
            tMaxX = (gx - ix) * tDeltaX;
        }

        if (stepY > 0) {
            tMaxY = (iy + 1.0f - gy) * tDeltaY;
        } else {
            tMaxY = (gy - iy) * tDeltaY;
        }

        if (stepZ > 0) {
            tMaxZ = (iz + 1.0f - gz) * tDeltaZ;
        } else {
            tMaxZ = (gz - iz) * tDeltaZ;
        }

        float ray_gain = 0.0f;

        /*if (skip_interval.x != -1.0f && skip_interval.x < params.voxel_size) {
            t = skip_interval.y / params.voxel_size;
            tMaxX += t;
            tMaxY += t;
            tMaxZ += t;
        }*/

        //if (debug && idx == 0) printf("[GPU] DDA Start: t=%.2f max_t=%.2f voxel=%.3f\n", t, max_t, params.voxel_size);

        while (t < max_t) {
            if (!has_jumped && skip_start_vox >= 0.0f && t >= skip_start_vox && t < skip_end_vox) {

                // Before jumping, verify we aren't currently in a wall.
                if (ix >= 0 && ix < map_dim.x && iy >= 0 && iy < map_dim.y && iz >= 0 && iz < map_dim.z) {
                    int check_idx = iz * (map_dim.x * map_dim.y) + iy * map_dim.x + ix;
                    if (map[check_idx] == V_OCCUPIED) {
                        final_depth = t * params.voxel_size;
                        break; // Stop immediately, do not jump.
                    }
                }

                has_jumped = true;
                float next_t = skip_end_vox;

                // --- RE-INITIALIZE DDA AT NEW 't' ---
                float next_gx = (cam_pos.x - map_origin.x) / params.voxel_size + (dir_x * next_t);
                float next_gy = (cam_pos.y - map_origin.y) / params.voxel_size + (dir_y * next_t);
                float next_gz = (cam_pos.z - map_origin.z) / params.voxel_size + (dir_z * next_t);

                int next_ix = floor(next_gx); 
                int next_iy = floor(next_gy); 
                int next_iz = floor(next_gz);

                // Only perform the jump logic if we actually leave the current voxel.
                if (next_ix != ix || next_iy != iy || next_iz != iz) {
                    t = next_t;
                    gx = next_gx; gy = next_gy; gz = next_gz;
                    ix = next_ix; iy = next_iy; iz = next_iz;

                    if (stepX > 0) tMaxX = (ix + 1.0f - gx) * tDeltaX; 
                    else           tMaxX = (gx - ix) * tDeltaX;
                    tMaxX += t;

                    if (stepY > 0) tMaxY = (iy + 1.0f - gy) * tDeltaY; 
                    else           tMaxY = (gy - iy) * tDeltaY;
                    tMaxY += t;

                    if (stepZ > 0) tMaxZ = (iz + 1.0f - gz) * tDeltaZ; 
                    else           tMaxZ = (gz - iz) * tDeltaZ;
                    tMaxZ += t;

                    // If the jump took us straight into an obstacle, stop.
                    if (status == -1.0f) {
                        final_depth = t * params.voxel_size;
                        break;
                    }

                    continue;
                }
            }
            
            if (ix >= 0 && ix < map_dim.x &&
                iy >= 0 && iy < map_dim.y &&
                iz >= 0 && iz < map_dim.z) {
                
                int flat_idx = iz * (map_dim.x * map_dim.y) + iy * map_dim.x + ix;
                uint8_t val = map[flat_idx];

                if (val == V_OCCUPIED) {
                    final_depth = t * params.voxel_size;
                    break;
                } 
                else if (val == V_UNKNOWN) {
                    float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                    float dt = t_exit - t; 
                    float dr = dt * params.voxel_size;

                    float r = t * params.voxel_size;
                    float term1 = 2.0f * r * r * dr;
                    float term2 = (dr * dr * dr) / 6.0f;
                    ray_gain += (term1 + term2) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }

            // Step to Next Voxel
            if (tMaxX < tMaxY && tMaxX < tMaxZ) {
                ix += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            } else if (tMaxY < tMaxZ) {
                iy += stepY;
                t = tMaxY;
                tMaxY += tDeltaY;
            } else {
                iz += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }

        int global_ray_idx = candidate * rays_per_candidate + idx;
        depth_buffer_all[global_ray_idx] = final_depth;

        if (ray_gain > 0.0f) {
            atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
        }
    }

    //if (debug) printf("[GPU] Thread 0 Finished!\n");
    __syncthreads();

    // -----------------------------------------------------------
    //    Sliding Window Optimization (Single Thread per Block)
    // -----------------------------------------------------------
    if (ray_id == 0) {
        float max_gain = 0.0f;
        int best_start_idx = 0;

        // How many 10-degree strips fit in my Horizontal FOV?
        int sectors_in_fov = (int)(params.fov_y_rad / params.dtheta);
        if (sectors_in_fov < 1) {
            sectors_in_fov = 1;
        }

        // Slide the window 0 to 36
        for (int i = 0; i < THETA_BINS; ++i) {
            float current_window_gain = 0.0f;
            
            // Sum up the strips inside the window
            for (int k = 0; k < sectors_in_fov; ++k) {
                int idx = (i + k) % THETA_BINS; // Wrap around (35 -> 0)
                current_window_gain += s_yaw_gains[idx];
            }

            // Keep track of the winner
            if (current_window_gain > max_gain) {
                max_gain = current_window_gain;
                best_start_idx = i;
            }
        }

        // 7. Write Final Result to Global Memory
        results_gain[candidate] = max_gain;
        
        // Calculate the center angle of the best window
        float start_angle = -CUDART_PI_F + (best_start_idx * params.dtheta);
        float center_angle = start_angle + (params.fov_y_rad * 0.5f);
        
        // Normalize angle to -PI to PI
        if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
        
        results_yaw[candidate] = center_angle;

        // Copy relevant depth buffer portion
        int out_stride = sectors_in_fov * rows_in_fov;
        int my_out = candidate * out_stride;
        int my_in  = candidate * rays_per_candidate;

        for (int phi_idx = 0; phi_idx < rows_in_fov; phi_idx++) {
            int row_start = phi_idx * THETA_BINS;
            for (int theta_idx = 0; theta_idx < sectors_in_fov; theta_idx++) {
                int global_theta_idx = my_in + ((best_start_idx + theta_idx) % THETA_BINS);

                int global_ray_idx = global_theta_idx + row_start;
                int local_ray_idx = my_out + theta_idx + (phi_idx * sectors_in_fov);

                depth_buffer[local_ray_idx] = depth_buffer_all[global_ray_idx];
            }
        }
    }
}

// ==========================================
// 3. HOST WRAPPER
// ==========================================
extern "C" void launch_aep_kernel_single(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float pos_x, float pos_y, float pos_z,
    float* h_result_gain, float* h_result_yaw,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {
    // 1. Pack Params
    KernelParams params;
    params.voxel_size = voxel_size;
    params.gain_range = gain_range;
    params.fov_y_rad = fov_y;
    params.fov_p_rad = fov_p;
    params.camera_pitch = pitch;
    
    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    // 2. Allocate Result Memory
    float* d_res_gain;
    float* d_res_yaw;
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));

    // 3. Launch (1 Block, Many Threads)
    // We launch 1 block because we are processing 1 candidate.
    // The threads inside the block process the rays in parallel.
    dim3 blocks(1);
    
    // Calculate total rays to optimize thread count
    int rows = (int)ceilf(params.fov_p_rad / params.dphi);
    if(rows < 1) rows = 1;
    int total_rays = THETA_BINS * rows;
    
    dim3 threads(min(total_rays, MAX_THREADS_PER_BLOCK));

    int3 map_dim = make_int3(dx, dy, dz);
    float3 map_origin = make_float3(ox, oy, oz);
    float3 candidate_pos = make_float3(pos_x, pos_y, pos_z);

    evaluate_aep_kernel_single<<<blocks, threads>>>(
        d_map, map_dim, map_origin, candidate_pos, d_res_gain, d_res_yaw, params
    );

    cudaDeviceSynchronize();

    // 4. Download Results
    cudaMemcpy(h_result_gain, d_res_gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_result_yaw, d_res_yaw, sizeof(float), cudaMemcpyDeviceToHost);

    // 5. Cleanup
    cudaFree(d_res_gain);
    cudaFree(d_res_yaw);
}

extern "C" void launch_aep_kernel(
    const uint8_t* h_map, 
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain, float* h_results_yaw,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {
    // 1. Pack Params
    KernelParams params;
    params.voxel_size = voxel_size;
    params.gain_range = gain_range;
    params.fov_y_rad = fov_y;
    params.fov_p_rad = fov_p;
    params.camera_pitch = pitch;

    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    // 2. Allocate Device Memory (Same as before)
    uint8_t* d_map;
    float3* d_positions;
    float* d_results_gain;
    float* d_results_yaw;

    size_t map_size = dx * dy * dz * sizeof(uint8_t);
    size_t cand_size = num_candidates * sizeof(float3);
    size_t res_size = num_candidates * sizeof(float);
    
    cudaMalloc(&d_map, map_size);
    cudaMalloc(&d_positions, cand_size);
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    // 3. Prepare Data
    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }

    cudaMemcpy(d_map, h_map, map_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    // 4. Launch
    dim3 blocks(num_candidates);
    dim3 threads(min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK));
    
    int3 map_dim = make_int3(dx, dy, dz);
    float3 map_origin = make_float3(ox, oy, oz);

    // Pass 'params' by value to the kernel
    evaluate_aep_kernel<<<blocks, threads>>>(
        d_map, map_dim, map_origin, d_positions, d_results_gain, d_results_yaw, params
    );

    cudaDeviceSynchronize();
    
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Kernel Error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(h_results_gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_results_yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    cudaFree(d_map);
    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    delete[] h_positions;
}

extern "C" void launch_aep_kernel_batch(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain, float* h_results_yaw,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {
    // 1. Pack Params
    KernelParams params;
    params.voxel_size = voxel_size;
    params.gain_range = gain_range;
    params.fov_y_rad = fov_y;
    params.fov_p_rad = fov_p;
    params.camera_pitch = pitch;

    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    // 2. Allocate Candidate Memory ONLY
    float3* d_positions;
    float* d_results_gain;
    float* d_results_yaw;

    size_t cand_size = num_candidates * sizeof(float3);
    size_t res_size = num_candidates * sizeof(float);
    
    cudaMalloc(&d_positions, cand_size);
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    // 3. Prepare Candidate Data
    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }

    // 4. Copy Candidates
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    // 5. Launch
    dim3 blocks(num_candidates);
    dim3 threads(min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK));
    
    int3 map_dim = make_int3(dx, dy, dz);
    float3 map_origin = make_float3(ox, oy, oz);

    evaluate_aep_kernel<<<blocks, threads>>>(
        d_map, map_dim, map_origin, d_positions, d_results_gain, d_results_yaw, params
    );

    cudaDeviceSynchronize();

    // 6. Download Results
    cudaMemcpy(h_results_gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_results_yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    // 7. Cleanup
    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    delete[] h_positions;
}

extern "C" void launch_aep_kernel_batch_depth(
    uint8_t* d_map,
    int dx, int dy, int dz, 
    float ox, float oy, float oz,
    float* h_pos_x, float* h_pos_y, float* h_pos_z,
    float* h_results_gain, float* h_results_yaw, float* h_results_depths,
    int num_candidates,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {
    
    // 1. Pack Params
    KernelParams params;
    params.voxel_size = voxel_size;
    params.gain_range = gain_range;
    params.fov_y_rad = fov_y;
    params.fov_p_rad = fov_p;
    params.camera_pitch = pitch;

    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    int window_width = floor(params.fov_y_rad / params.dtheta);
    int window_height = floor(params.fov_p_rad / params.dphi);

    int rays_per_candidate = THETA_BINS * window_height;

    int buffer_size_all = num_candidates * rays_per_candidate * sizeof(float); 
    int buffer_size = num_candidates * window_width * window_height * sizeof(float);

    // 2. Allocate Candidate Memory ONLY
    float3* d_positions;
    float* d_results_gain;
    float* d_results_yaw;

    float* d_depth_buffer_all;
    float* d_depth_buffer;

    size_t cand_size = num_candidates * sizeof(float3);
    size_t res_size = num_candidates * sizeof(float);
    
    cudaMalloc(&d_positions, cand_size);
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);

    // 3. Prepare Candidate Data
    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }

    // 4. Copy Candidates
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    // 5. Launch
    dim3 blocks(num_candidates);
    dim3 threads(min(rays_per_candidate, MAX_THREADS_PER_BLOCK));
    
    int3 map_dim = make_int3(dx, dy, dz);
    float3 map_origin = make_float3(ox, oy, oz);

    evaluate_aep_kernel_depth<<<blocks, threads>>>(
        d_map, map_dim, map_origin, d_positions, d_results_gain, d_results_yaw, d_depth_buffer_all, d_depth_buffer, params
    );

    cudaDeviceSynchronize();

    // 6. Download Results
    cudaMemcpy(h_results_gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_results_yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    if (h_results_depths != nullptr) {
        cudaMemcpy(h_results_depths, d_depth_buffer, buffer_size, cudaMemcpyDeviceToHost);
    }

    // 7. Cleanup
    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    cudaFree(d_depth_buffer_all);
    cudaFree(d_depth_buffer);
    
    delete[] h_positions;
}

extern "C" void launch_marginal_gain_kernel(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    float h_parent_x, float h_parent_y, float h_parent_z,
    float h_parent_yaw, float* h_parent_R, float* h_parent_depth,
    float* h_result_gain, float* h_result_yaw, float* h_result_depths,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {
    
    // 1. Pack Params
    KernelParams params;
    params.voxel_size = voxel_size;
    params.gain_range = gain_range;
    params.fov_y_rad = fov_y;
    params.fov_p_rad = fov_p;
    params.camera_pitch = pitch;

    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    int window_width = floor(params.fov_y_rad / params.dtheta);
    if (window_width < 1) window_width = 1;
    int window_height = floor(params.fov_p_rad / params.dphi);
    if (window_height < 1) window_height = 1;

    int rays_per_candidate = THETA_BINS * window_height;

    int buffer_size_all = rays_per_candidate * sizeof(float); 
    int buffer_size = window_width * window_height * sizeof(float);
    int parent_buffer_size = window_width * window_height * sizeof(float);

    float fx = (window_width / 2.0f) / tanf(params.fov_y_rad * 0.5f);
    float fy = (window_height / 2.0f) / tanf(params.fov_p_rad * 0.5f);
    float cx = window_width / 2.0f;
    float cy = window_height / 2.0f;

    // 2. Allocate Device Memory
    float3* d_cand_pos;
    float* d_res_gain;
    float* d_res_yaw;

    float* d_depth_buffer_all;
    float* d_depth_buffer;
    float* d_parent_depth_buffer;

    cudaMalloc(&d_cand_pos, sizeof(float3));
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));

    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);
    cudaMalloc(&d_parent_depth_buffer, parent_buffer_size);

    float3 h_pos = make_float3(h_cand_x, h_cand_y, h_cand_z);
    cudaMemcpy(d_cand_pos, &h_pos, sizeof(float3), cudaMemcpyHostToDevice);

    if (h_parent_depth != nullptr) {
        cudaMemcpy(d_parent_depth_buffer, h_parent_depth, parent_buffer_size, cudaMemcpyHostToDevice);
    } else {
        cudaMemset(d_parent_depth_buffer, 0, parent_buffer_size); 
    }

    // 3. Launch Kernel
    dim3 blocks(1);
    dim3 threads(min(rays_per_candidate, MAX_THREADS_PER_BLOCK));

    int3 map_dim = make_int3(dx, dy, dz);
    float3 map_origin = make_float3(ox, oy, oz);
    float3 parent_pos = make_float3(h_parent_x, h_parent_y, h_parent_z);
    float parent_yaw = h_parent_yaw;

    float3 R0 = make_float3(h_parent_R[0], h_parent_R[1], h_parent_R[2]);
    float3 R1 = make_float3(h_parent_R[3], h_parent_R[4], h_parent_R[5]);
    float3 R2 = make_float3(h_parent_R[6], h_parent_R[7], h_parent_R[8]);

    evaluate_marginal_gain_kernel<<<blocks, threads>>>(
        d_map, map_dim, map_origin, d_cand_pos, parent_pos, parent_yaw, d_parent_depth_buffer,
        R0, R1, R2, window_width, window_height, fx, fy, cx, cy,
        d_res_gain, d_res_yaw, d_depth_buffer_all, d_depth_buffer, params
    );

    cudaDeviceSynchronize();

    // 4. Download
    cudaMemcpy(h_result_gain, d_res_gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_result_yaw, d_res_yaw, sizeof(float), cudaMemcpyDeviceToHost);

    if (h_result_depths != nullptr) {
        cudaMemcpy(h_result_depths, d_depth_buffer, buffer_size, cudaMemcpyDeviceToHost);
    }

    // 5. Cleanup
    cudaFree(d_cand_pos);
    cudaFree(d_res_gain);
    cudaFree(d_res_yaw);
    cudaFree(d_depth_buffer_all);
    cudaFree(d_depth_buffer);
    cudaFree(d_parent_depth_buffer);
}

extern "C" void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size) {
    cudaMalloc((void**)dev_ptr, size);
}

extern "C" void wrapper_cuda_free(void* dev_ptr) {
    if (dev_ptr) {
        cudaFree(dev_ptr);
    }
}

extern "C" void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size) {
    cudaMemcpy(dev_ptr, host_ptr, size, cudaMemcpyHostToDevice);
}