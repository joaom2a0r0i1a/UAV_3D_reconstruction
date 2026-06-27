#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_constants.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <rrt_construction/aep_device_math.cuh>

// ============================================================================
//  aep_evaluator.cu
//
//  CUDA front end for the AEP information-gain and marginal-gain raycasters.
//  This translation unit holds ONLY:
//    * __global__ kernels  -- thin orchestration; the math lives in
//                             aep_device_math.cuh.
//    * extern "C" launchers -- host glue that packs parameters, manages device
//                             memory and launches the kernels.
//
//  Kernel argument lists are bundled into small structs (MapContext, ParentFrame,
//  AncestorSet, GainResults) to stay well under the JPL 6-argument limit. The
//  extern "C" launcher signatures are the ABI consumed by gain_evaluator.cpp and
//  are intentionally left unchanged.
// ============================================================================

// ----------------------------------------------------------------------------
//  Host helpers (shared by every launcher).
// ----------------------------------------------------------------------------

// Pack the dynamic launch parameters, deriving the angular steps and phi band.
static KernelParams make_kernel_params(float voxel_size, float gain_range,
                                       float fov_y, float fov_p, float pitch) {
    KernelParams params;
    params.voxel_size   = voxel_size;
    params.gain_range   = gain_range;
    params.fov_y_rad    = fov_y;
    params.fov_p_rad    = fov_p;
    params.camera_pitch = pitch;

    params.dtheta = DTHETA_DEG * CUDART_PI_F / 180.0f;
    params.dphi   = DPHI_DEG   * CUDART_PI_F / 180.0f;

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);
    return params;
}

// Derive the parent depth-image geometry (pixel size + pinhole intrinsics) that
// every marginal kernel shares.
static aep::ParentCameraConfig derive_camera_config(float gain_range, float voxel_size,
                                                    const KernelParams& params) {
    aep::ParentCameraConfig cam;
    cam.p_width  = ceil((2.0f * gain_range * tanf(params.fov_y_rad * 0.5f)) / voxel_size);
    cam.p_height = ceil((2.0f * gain_range * tanf(params.fov_p_rad * 0.5f)) / voxel_size);
    cam.fx = (cam.p_width  / 2.0f) / tanf(params.fov_y_rad * 0.5f);
    cam.fy = (cam.p_height / 2.0f) / tanf(params.fov_p_rad * 0.5f);
    cam.cx = cam.p_width  / 2.0f;
    cam.cy = cam.p_height / 2.0f;
    return cam;
}

// ============================================================================
//  Kernels: AEP information gain (no parent occlusion).
// ============================================================================

// One candidate, one block. Threads share the ray workload and reduce to the
// best yaw window.
__global__ void evaluate_aep_kernel_single(MapContext m, float3 candidate_pos,
                                           float* __restrict__ result_gain,
                                           float* __restrict__ result_yaw,
                                           KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int tid = threadIdx.x;
    if (tid < THETA_BINS) s_yaw_gains[tid] = 0.0f;
    __syncthreads();

    int rows_in_fov = max(1, (int)ceilf(params.fov_p_rad / params.dphi));
    int rays_total  = THETA_BINS * rows_in_fov;

    for (int idx = tid; idx < rays_total; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 dir = aep::spherical_ray_dir(theta, phi);

        float depth;
        float ray_gain = march_gain_basic(m, candidate_pos, dir, sin_phi, params, &depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (tid == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        *result_gain = max_gain;
        *result_yaw  = aep::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
    }
}

// One candidate per block, batched over many candidates.
__global__ void evaluate_aep_kernel(MapContext m, const float3* __restrict__ positions,
                                    float* __restrict__ results_gain,
                                    float* __restrict__ results_yaw,
                                    KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < THETA_BINS) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov = max(1, (int)ceilf(params.fov_p_rad / params.dphi));
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 dir = aep::spherical_ray_dir(theta, phi);

        float depth;
        float ray_gain = march_gain_basic(m, positions[candidate], dir, sin_phi, params, &depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (ray_id == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        results_gain[candidate] = max_gain;
        results_yaw[candidate]  = aep::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
    }
}

// As above, but also records the per-ray first-hit depth and copies the windowed
// subset that falls inside the chosen yaw FOV.
__global__ void evaluate_aep_kernel_depth(MapContext m, const float3* __restrict__ positions,
                                          float* __restrict__ results_gain,
                                          float* __restrict__ results_yaw,
                                          float* __restrict__ depth_buffer_all,
                                          float* __restrict__ depth_buffer,
                                          KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < THETA_BINS) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov = max(1, (int)ceilf(params.fov_p_rad / params.dphi));
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 dir = aep::spherical_ray_dir(theta, phi);

        float final_depth;
        float ray_gain = march_gain_basic(m, positions[candidate], dir, sin_phi, params, &final_depth);
        depth_buffer_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (ray_id == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best_start_idx = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        results_gain[candidate] = max_gain;
        results_yaw[candidate]  = aep::yaw_window_center_angle(best_start_idx, params.dtheta, params.fov_y_rad);

        // Copy the depths that lie inside the selected yaw window.
        int my_out = candidate * (sectors_in_fov * rows_in_fov);
        int my_in  = candidate * rays_per_candidate;
        for (int phi_idx = 0; phi_idx < rows_in_fov; phi_idx++) {
            int row_start = phi_idx * THETA_BINS;
            for (int theta_idx = 0; theta_idx < sectors_in_fov; theta_idx++) {
                int global_ray_idx = my_in + ((best_start_idx + theta_idx) % THETA_BINS) + row_start;
                int local_ray_idx  = my_out + theta_idx + (phi_idx * sectors_in_fov);
                depth_buffer[local_ray_idx] = depth_buffer_all[global_ray_idx];
            }
        }
    }
}

// ============================================================================
//  Kernels: marginal information gain (subtract space ancestors already saw).
// ============================================================================

// Legacy single-parent, single-interval skip (v1).
__global__ void evaluate_marginal_gain_kernel(MapContext m, const float3* __restrict__ positions,
                                              ParentFrame parent, GainResults out,
                                              KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < THETA_BINS) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov    = max(1, (int)(params.fov_p_rad / params.dphi));
    int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 ray_dir = aep::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Project the ray into the parent frustum, convert the observed-free span
        // to voxel units, then march while jumping that span.
        float3 interval = compute_skip_distance(parent.cam, parent.pos, parent.R, parent.depth,
                                                cam_pos, ray_dir, params.gain_range);
        float skip_start_vox = (interval.x != -1.0f) ? (interval.x / params.voxel_size) : -1.0f;
        float skip_end_vox   = (interval.x != -1.0f) ? (interval.y / params.voxel_size) : -1.0f;

        float final_depth;
        float ray_gain = march_marginal_gain_single(m, cam_pos, ray_dir, sin_phi,
                                                     skip_start_vox, skip_end_vox, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = aep::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = parent.cam.p_width * parent.cam.p_height;
    generate_depth_buffer(m, parent.cam, positions[candidate], s_best_yaw, params,
                          out.depth + candidate * buffer_rays);
}

// Legacy single-parent, multi-segment skip without jumping (v2).
__global__ void evaluate_marginal_gain_kernel_v2(MapContext m, const float3* __restrict__ positions,
                                                 ParentFrame parent, GainResults out,
                                                 KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < THETA_BINS) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov    = max(1, (int)(params.fov_p_rad / params.dphi));
    int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 ray_dir = aep::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        compute_multi_segment_skip_distance(parent.cam, parent.pos, parent.R, parent.depth,
                                            cam_pos, ray_dir, params,
                                            skip_m, &skip_count, MAX_SEGS, &status);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i) {
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size,
                                      skip_m[i].y / params.voxel_size);
        }

        float final_depth;
        float ray_gain = march_marginal_gain_suppress(m, cam_pos, ray_dir, sin_phi,
                                                      skip_vox, skip_count, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = aep::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = parent.cam.p_width * parent.cam.p_height;
    generate_depth_buffer(m, parent.cam, positions[candidate], s_best_yaw, params,
                          out.depth + candidate * buffer_rays);
}

// Canonical multi-ancestor marginal gain with jumping + range/skip clamps (v3).
__global__ void evaluate_marginal_gain_kernel_v3(MapContext m, const float3* __restrict__ positions,
                                                 AncestorSet ancestors, GainResults out,
                                                 KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < THETA_BINS) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov    = max(1, (int)(params.fov_p_rad / params.dphi));
    int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
    int rays_per_candidate = THETA_BINS * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % THETA_BINS;
        int phi_idx   = idx / THETA_BINS;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        if (phi > params.phi_end) continue;

        float sin_phi = sinf(phi);
        float3 ray_dir = aep::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Merge the observed-free spans across every ancestor, in voxel units.
        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        compute_multi_segment_skip_distance(ancestors.cam, ancestors.positions, ancestors.R_rows,
                                            ancestors.depth, ancestors.num, cam_pos, ray_dir, params,
                                            skip_m, &skip_count, MAX_SEGS, &status);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i) {
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size,
                                      skip_m[i].y / params.voxel_size);
        }

        float final_depth;
        float ray_gain = march_marginal_gain(m, cam_pos, ray_dir, sin_phi,
                                             skip_vox, skip_count, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = aep::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = aep::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ancestors.cam.p_width * ancestors.cam.p_height;
    generate_depth_buffer(m, ancestors.cam, positions[candidate], s_best_yaw, params,
                          out.depth + candidate * buffer_rays);
}

// ============================================================================
//  Launchers (extern "C" ABI -- consumed by gain_evaluator.cpp).
// ============================================================================

extern "C" void launch_aep_kernel_single(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float pos_x, float pos_y, float pos_z,
    float* h_result_gain, float* h_result_yaw,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);

    float* d_res_gain;
    float* d_res_yaw;
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));

    int rows = max(1, (int)ceilf(params.fov_p_rad / params.dphi));
    int total_rays = THETA_BINS * rows;

    MapContext m = {d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};
    float3 candidate_pos = make_float3(pos_x, pos_y, pos_z);

    evaluate_aep_kernel_single<<<1, min(total_rays, MAX_THREADS_PER_BLOCK)>>>(
        m, candidate_pos, d_res_gain, d_res_yaw, params);
    cudaDeviceSynchronize();

    cudaMemcpy(h_result_gain, d_res_gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_result_yaw, d_res_yaw, sizeof(float), cudaMemcpyDeviceToHost);

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

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);

    uint8_t* d_map;
    float3* d_positions;
    float* d_results_gain;
    float* d_results_yaw;

    size_t map_size = (size_t)dx * dy * dz * sizeof(uint8_t);
    size_t cand_size = num_candidates * sizeof(float3);
    size_t res_size = num_candidates * sizeof(float);

    cudaMalloc(&d_map, map_size);
    cudaMalloc(&d_positions, cand_size);
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }

    cudaMemcpy(d_map, h_map, map_size, cudaMemcpyHostToDevice);
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    MapContext m = {d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};

    evaluate_aep_kernel<<<num_candidates, min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params);
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

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);

    float3* d_positions;
    float* d_results_gain;
    float* d_results_yaw;

    size_t cand_size = num_candidates * sizeof(float3);
    size_t res_size = num_candidates * sizeof(float);

    cudaMalloc(&d_positions, cand_size);
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    MapContext m = {d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};

    evaluate_aep_kernel<<<num_candidates, min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params);
    cudaDeviceSynchronize();

    cudaMemcpy(h_results_gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_results_yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

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

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);

    int window_width  = floor(params.fov_y_rad / params.dtheta);
    int window_height = floor(params.fov_p_rad / params.dphi);
    int rays_per_candidate = THETA_BINS * window_height;

    size_t buffer_size_all = (size_t)num_candidates * rays_per_candidate * sizeof(float);
    size_t buffer_size = (size_t)num_candidates * window_width * window_height * sizeof(float);

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

    float3* h_positions = new float3[num_candidates];
    for (int i = 0; i < num_candidates; ++i) {
        h_positions[i] = make_float3(h_pos_x[i], h_pos_y[i], h_pos_z[i]);
    }
    cudaMemcpy(d_positions, h_positions, cand_size, cudaMemcpyHostToDevice);

    MapContext m = {d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};

    evaluate_aep_kernel_depth<<<num_candidates, min(rays_per_candidate, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, d_depth_buffer_all, d_depth_buffer, params);
    cudaDeviceSynchronize();

    cudaMemcpy(h_results_gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(h_results_yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);
    if (h_results_depths != nullptr) {
        cudaMemcpy(h_results_depths, d_depth_buffer, buffer_size, cudaMemcpyDeviceToHost);
    }

    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    cudaFree(d_depth_buffer_all);
    cudaFree(d_depth_buffer);
    delete[] h_positions;
}

// Shared setup for the single-parent marginal launchers (v1 and v2): allocate
// device buffers, upload the candidate + parent state, and assemble the kernel
// argument structs. Returns the rays-per-candidate launch width.
static int setup_single_parent_marginal(
    uint8_t* d_map, int dx, int dy, int dz, float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    float h_parent_x, float h_parent_y, float h_parent_z,
    float* h_parent_R, float* h_parent_depth,
    const KernelParams& params, const aep::ParentCameraConfig& cam,
    MapContext* m, float3** d_cand_pos, ParentFrame* parent, GainResults* out) {

    int rows_in_fov = max(1, (int)floor(params.fov_p_rad / params.dphi));
    int rays_per_candidate = THETA_BINS * rows_in_fov;
    size_t buffer_size_all = (size_t)rays_per_candidate * sizeof(float);
    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);

    float* d_res_gain;
    float* d_res_yaw;
    float* d_depth_buffer_all;
    float* d_depth_buffer;
    float* d_parent_depth_buffer;

    cudaMalloc(d_cand_pos, sizeof(float3));
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));
    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);
    cudaMalloc(&d_parent_depth_buffer, buffer_size);

    float3 h_pos = make_float3(h_cand_x, h_cand_y, h_cand_z);
    cudaMemcpy(*d_cand_pos, &h_pos, sizeof(float3), cudaMemcpyHostToDevice);

    if (h_parent_depth != nullptr) {
        cudaMemcpy(d_parent_depth_buffer, h_parent_depth, buffer_size, cudaMemcpyHostToDevice);
    } else {
        cudaMemset(d_parent_depth_buffer, 0, buffer_size);
    }

    *m = MapContext{d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};

    aep::RotationRows R = {
        make_float3(h_parent_R[0], h_parent_R[1], h_parent_R[2]),
        make_float3(h_parent_R[3], h_parent_R[4], h_parent_R[5]),
        make_float3(h_parent_R[6], h_parent_R[7], h_parent_R[8])};
    *parent = ParentFrame{make_float3(h_parent_x, h_parent_y, h_parent_z),
                          d_parent_depth_buffer, R, cam};
    *out = GainResults{d_res_gain, d_res_yaw, d_depth_buffer_all, d_depth_buffer};
    return rays_per_candidate;
}

// Download results then release every device buffer held in a GainResults plus
// the (single) parent depth buffer and candidate position.
static void teardown_single_parent_marginal(
    float3* d_cand_pos, const ParentFrame& parent, const GainResults& out,
    const aep::ParentCameraConfig& cam,
    float* h_result_gain, float* h_result_yaw, float* h_result_depths) {

    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);
    cudaMemcpy(h_result_gain, out.gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_result_yaw, out.yaw, sizeof(float), cudaMemcpyDeviceToHost);
    if (h_result_depths != nullptr) {
        cudaMemcpy(h_result_depths, out.depth, buffer_size, cudaMemcpyDeviceToHost);
    }
    cudaFree(d_cand_pos);
    cudaFree(out.gain);
    cudaFree(out.yaw);
    cudaFree(out.depth_all);
    cudaFree(out.depth);
    cudaFree(const_cast<float*>(parent.depth));
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

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);
    aep::ParentCameraConfig cam = derive_camera_config(gain_range, voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    ParentFrame parent;
    GainResults out;
    int rays = setup_single_parent_marginal(
        d_map, dx, dy, dz, ox, oy, oz, h_cand_x, h_cand_y, h_cand_z,
        h_parent_x, h_parent_y, h_parent_z, h_parent_R, h_parent_depth,
        params, cam, &m, &d_cand_pos, &parent, &out);

    evaluate_marginal_gain_kernel<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, parent, out, params);
    cudaDeviceSynchronize();

    teardown_single_parent_marginal(d_cand_pos, parent, out, cam,
                                    h_result_gain, h_result_yaw, h_result_depths);
}

extern "C" void launch_marginal_gain_kernel_v2(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    float h_parent_x, float h_parent_y, float h_parent_z,
    float h_parent_yaw, float* h_parent_R, float* h_parent_depth,
    float* h_result_gain, float* h_result_yaw, float* h_result_depths,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);
    aep::ParentCameraConfig cam = derive_camera_config(gain_range, voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    ParentFrame parent;
    GainResults out;
    int rays = setup_single_parent_marginal(
        d_map, dx, dy, dz, ox, oy, oz, h_cand_x, h_cand_y, h_cand_z,
        h_parent_x, h_parent_y, h_parent_z, h_parent_R, h_parent_depth,
        params, cam, &m, &d_cand_pos, &parent, &out);

    evaluate_marginal_gain_kernel_v2<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, parent, out, params);
    cudaDeviceSynchronize();

    teardown_single_parent_marginal(d_cand_pos, parent, out, cam,
                                    h_result_gain, h_result_yaw, h_result_depths);
}

extern "C" void launch_marginal_gain_kernel_v3(
    uint8_t* d_map,
    int dx, int dy, int dz,
    float ox, float oy, float oz,
    float h_cand_x, float h_cand_y, float h_cand_z,
    int num_ancestors,
    float* h_parent_pos,    // [3*num_ancestors]  (x,y,z per ancestor)
    float* h_parent_yaw,    // [num_ancestors]
    float* h_parent_R,      // [9*num_ancestors]  (row-major, 3 rows per ancestor)
    float* h_parent_depth,  // [num_ancestors * p_width * p_height], or nullptr
    float* h_result_gain, float* h_result_yaw, float* h_result_depths,
    float voxel_size, float gain_range, float fov_y, float fov_p, float pitch) {

    KernelParams params = make_kernel_params(voxel_size, gain_range, fov_y, fov_p, pitch);
    aep::ParentCameraConfig cam = derive_camera_config(gain_range, voxel_size, params);

    int rows_in_fov = max(1, (int)floor(params.fov_p_rad / params.dphi));
    int rays_per_candidate = THETA_BINS * rows_in_fov;
    size_t buffer_size_all = (size_t)rays_per_candidate * sizeof(float);
    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);
    size_t per = (size_t)cam.p_width * cam.p_height;   // depth elements per ancestor

    // Candidate + output buffers.
    float3* d_cand_pos;
    float* d_res_gain;
    float* d_res_yaw;
    float* d_depth_buffer_all;
    float* d_depth_buffer;
    cudaMalloc(&d_cand_pos, sizeof(float3));
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));
    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);

    // Flattened ancestor state (host layouts match float3 / 3xfloat3 packing).
    float3* d_parent_pos;
    float3* d_parent_R;
    float*  d_parent_yaw;
    float*  d_parent_depth;
    cudaMalloc(&d_parent_pos,   num_ancestors * sizeof(float3));
    cudaMalloc(&d_parent_R,     num_ancestors * 3 * sizeof(float3));
    cudaMalloc(&d_parent_yaw,   num_ancestors * sizeof(float));
    cudaMalloc(&d_parent_depth, num_ancestors * per * sizeof(float));

    float3 h_pos = make_float3(h_cand_x, h_cand_y, h_cand_z);
    cudaMemcpy(d_cand_pos, &h_pos, sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_pos, h_parent_pos, num_ancestors * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_R,   h_parent_R,   num_ancestors * 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_yaw, h_parent_yaw, num_ancestors * sizeof(float),     cudaMemcpyHostToDevice);
    if (h_parent_depth != nullptr) {
        cudaMemcpy(d_parent_depth, h_parent_depth, num_ancestors * per * sizeof(float), cudaMemcpyHostToDevice);
    } else {
        cudaMemset(d_parent_depth, 0, num_ancestors * per * sizeof(float));
    }

    MapContext m = {d_map, make_int3(dx, dy, dz), make_float3(ox, oy, oz)};
    AncestorSet ancestors = {d_parent_pos, d_parent_yaw, d_parent_depth, d_parent_R, num_ancestors, cam};
    GainResults out = {d_res_gain, d_res_yaw, d_depth_buffer_all, d_depth_buffer};

    evaluate_marginal_gain_kernel_v3<<<1, min(rays_per_candidate, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, ancestors, out, params);
    cudaDeviceSynchronize();

    cudaMemcpy(h_result_gain, d_res_gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_result_yaw, d_res_yaw, sizeof(float), cudaMemcpyDeviceToHost);
    if (h_result_depths != nullptr) {
        cudaMemcpy(h_result_depths, d_depth_buffer, buffer_size, cudaMemcpyDeviceToHost);
    }

    cudaFree(d_cand_pos);
    cudaFree(d_res_gain);
    cudaFree(d_res_yaw);
    cudaFree(d_depth_buffer_all);
    cudaFree(d_depth_buffer);
    cudaFree(d_parent_pos);
    cudaFree(d_parent_R);
    cudaFree(d_parent_yaw);
    cudaFree(d_parent_depth);
}

// ============================================================================
//  Thin CUDA memory wrappers (used by gain_evaluator.cpp to own the map buffer).
// ============================================================================

extern "C" void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size) {
    cudaMalloc((void**)dev_ptr, size);
}

extern "C" void wrapper_cuda_free(void* dev_ptr) {
    if (dev_ptr) cudaFree(dev_ptr);
}

extern "C" void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size) {
    cudaMemcpy(dev_ptr, host_ptr, size, cudaMemcpyHostToDevice);
}
