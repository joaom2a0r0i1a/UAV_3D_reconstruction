#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_constants.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <rrt_construction/gpu_raycast_math.cuh>
#include <rrt_construction/gpu_raycast_launch.h>

// ============================================================================
//  gpu_raycaster.cu
//
//  CUDA front end for the information-gain and marginal-gain raycasters.
//  This translation unit holds ONLY:
//    * __global__ kernels  -- thin orchestration; the math lives in
//                             gpu_raycast_math.cuh.
//    * extern "C" launchers -- host glue that packs parameters, manages device
//                             memory and launches the kernels.
//
//  Kernel argument lists are bundled into small structs (MapContext, ParentFrame,
//  AncestorSet, GainResults) to stay well under the JPL 5-argument limit. The
//  extern "C" launcher signatures (declared in gpu_raycast_launch.h, consumed by
//  gain_evaluator.cpp) are likewise grouped into POD structs -- GpuMap, GpuSensor,
//  GpuCandidates, GpuParent, GpuAncestors, GpuResult.
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
static gpuray::ParentCameraConfig derive_camera_config(float gain_range, float voxel_size,
                                                    const KernelParams& params) {
    gpuray::ParentCameraConfig cam;
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
__global__ void evaluate_gain_kernel_single(MapContext m, float3 candidate_pos,
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
        float3 dir = gpuray::spherical_ray_dir(theta, phi);

        float depth;
        MarchRay ray = {candidate_pos, dir, sin_phi};
        float ray_gain = march_gain_basic(m, ray, params, &depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (tid == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        *result_gain = max_gain;
        *result_yaw  = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
    }
}

// One candidate per block, batched over many candidates.
__global__ void evaluate_gain_kernel(MapContext m, const float3* __restrict__ positions,
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
        float3 dir = gpuray::spherical_ray_dir(theta, phi);

        float depth;
        MarchRay ray = {positions[candidate], dir, sin_phi};
        float ray_gain = march_gain_basic(m, ray, params, &depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (ray_id == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        results_gain[candidate] = max_gain;
        results_yaw[candidate]  = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
    }
}

// As above, but also records the per-ray first-hit depth and copies the windowed
// subset that falls inside the chosen yaw FOV.
__global__ void evaluate_gain_kernel_depth(MapContext m, const float3* __restrict__ positions,
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
        float3 dir = gpuray::spherical_ray_dir(theta, phi);

        float final_depth;
        MarchRay ray = {positions[candidate], dir, sin_phi};
        float ray_gain = march_gain_basic(m, ray, params, &final_depth);
        depth_buffer_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (ray_id == 0) {
        int sectors_in_fov = max(1, (int)(params.fov_y_rad / params.dtheta));
        float max_gain;
        int best_start_idx = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        results_gain[candidate] = max_gain;
        results_yaw[candidate]  = gpuray::yaw_window_center_angle(best_start_idx, params.dtheta, params.fov_y_rad);

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
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Project the ray into the parent frustum, convert the observed-free span
        // to voxel units, then march while jumping that span.
        Ray ray = {cam_pos, ray_dir};
        float3 interval = compute_skip_distance(parent, ray, params.gain_range);
        float2 skip = make_float2(
            (interval.x != -1.0f) ? (interval.x / params.voxel_size) : -1.0f,
            (interval.x != -1.0f) ? (interval.y / params.voxel_size) : -1.0f);

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        float ray_gain = march_marginal_gain_single(m, mray, skip, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = parent.cam.p_width * parent.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, parent.cam, pose, params, out.depth + candidate * buffer_rays);
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
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        Ray ray = {cam_pos, ray_dir};
        SkipBuffer skip_out = {skip_m, &skip_count, MAX_SEGS, &status};
        compute_skip_intervals_single(parent, ray, params, skip_out);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i) {
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size,
                                      skip_m[i].y / params.voxel_size);
        }

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        SkipSet skips = {skip_vox, skip_count};
        float ray_gain = march_marginal_gain_suppress(m, mray, skips, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = parent.cam.p_width * parent.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, parent.cam, pose, params, out.depth + candidate * buffer_rays);
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
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Merge the observed-free spans across every ancestor, in voxel units.
        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        Ray ray = {cam_pos, ray_dir};
        SkipBuffer skip_out = {skip_m, &skip_count, MAX_SEGS, &status};
        compute_multi_segment_skip_distance(ancestors, ray, params, skip_out);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i) {
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size,
                                      skip_m[i].y / params.voxel_size);
        }

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        SkipSet skips = {skip_vox, skip_count};
        float ray_gain = march_marginal_gain(m, mray, skips, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ancestors.cam.p_width * ancestors.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, ancestors.cam, pose, params, out.depth + candidate * buffer_rays);
}

// Multi-ancestor marginal gain, traversal variant (v4). Identical to v3 except it
// marches the observed-free spans instead of jumping them (safer, no DDA reseat);
// gain inside a span is suppressed rather than skipped. See march_marginal_gain_traverse.
__global__ void evaluate_marginal_gain_kernel_v4(MapContext m, const float3* __restrict__ positions,
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
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Merge the observed-free spans across every ancestor, in voxel units.
        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        Ray ray = {cam_pos, ray_dir};
        SkipBuffer skip_out = {skip_m, &skip_count, MAX_SEGS, &status};
        compute_multi_segment_skip_distance(ancestors, ray, params, skip_out);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i) {
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size,
                                      skip_m[i].y / params.voxel_size);
        }

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        SkipSet skips = {skip_vox, skip_count};
        float ray_gain = march_marginal_gain_traverse(m, mray, skips, params, &final_depth);

        out.depth_all[candidate * rays_per_candidate + idx] = final_depth;
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain;
        int best = gpuray::best_yaw_start_index(s_yaw_gains, THETA_BINS, sectors_in_fov, &max_gain);
        float center = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ancestors.cam.p_width * ancestors.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, ancestors.cam, pose, params, out.depth + candidate * buffer_rays);
}

// ============================================================================
//  Launchers (extern "C" ABI -- consumed by gain_evaluator.cpp).
// ============================================================================

// Pack the loose host config fields into the device-side launch structs.
static KernelParams params_of(const GpuSensor& cfg) {
    return make_kernel_params(cfg.voxel_size, cfg.gain_range, cfg.fov_y, cfg.fov_p, cfg.pitch);
}
static MapContext context_of(const GpuMap& map) {
    return MapContext{map.d_map, make_int3(map.dx, map.dy, map.dz),
                      make_float3(map.ox, map.oy, map.oz)};
}

// Upload an x/y/z candidate list to a freshly allocated device float3 array.
static float3* upload_candidates(const GpuCandidates& cands) {
    float3* d_positions;
    cudaMalloc(&d_positions, cands.count * sizeof(float3));
    float3* h_positions = new float3[cands.count];
    for (int i = 0; i < cands.count; ++i) {
        h_positions[i] = make_float3(cands.x[i], cands.y[i], cands.z[i]);
    }
    cudaMemcpy(d_positions, h_positions, cands.count * sizeof(float3), cudaMemcpyHostToDevice);
    delete[] h_positions;
    return d_positions;
}

extern "C" void launch_gain_kernel_single(GpuMap map, GpuVec3 cand,
                                         GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);

    float* d_res_gain;
    float* d_res_yaw;
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));

    int rows = max(1, (int)ceilf(params.fov_p_rad / params.dphi));
    int total_rays = THETA_BINS * rows;

    MapContext m = context_of(map);
    float3 candidate_pos = make_float3(cand.x, cand.y, cand.z);

    evaluate_gain_kernel_single<<<1, min(total_rays, MAX_THREADS_PER_BLOCK)>>>(
        m, candidate_pos, d_res_gain, d_res_yaw, params);
    cudaDeviceSynchronize();

    cudaMemcpy(out.gain, d_res_gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_res_yaw, sizeof(float), cudaMemcpyDeviceToHost);

    cudaFree(d_res_gain);
    cudaFree(d_res_yaw);
}

// Self-contained variant: map.d_map is a HOST grid; it is uploaded here.
extern "C" void launch_gain_kernel(GpuMap map, GpuCandidates cands,
                                  GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);

    size_t map_size = (size_t)map.dx * map.dy * map.dz * sizeof(uint8_t);
    size_t res_size = cands.count * sizeof(float);

    uint8_t* d_map;
    cudaMalloc(&d_map, map_size);
    cudaMemcpy(d_map, map.d_map, map_size, cudaMemcpyHostToDevice);

    float3* d_positions = upload_candidates(cands);
    float* d_results_gain;
    float* d_results_yaw;
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    MapContext m = {d_map, make_int3(map.dx, map.dy, map.dz),
                    make_float3(map.ox, map.oy, map.oz)};

    evaluate_gain_kernel<<<cands.count, min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params);
    cudaDeviceSynchronize();

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        printf("CUDA Kernel Error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(out.gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    cudaFree(d_map);
    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
}

extern "C" void launch_gain_kernel_batch(GpuMap map, GpuCandidates cands,
                                        GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    size_t res_size = cands.count * sizeof(float);

    float3* d_positions = upload_candidates(cands);
    float* d_results_gain;
    float* d_results_yaw;
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    MapContext m = context_of(map);

    evaluate_gain_kernel<<<cands.count, min(TOTAL_RAYS, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params);
    cudaDeviceSynchronize();

    cudaMemcpy(out.gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
}

extern "C" void launch_gain_kernel_batch_depth(GpuMap map, GpuCandidates cands,
                                              GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);

    int window_width  = floor(params.fov_y_rad / params.dtheta);
    int window_height = floor(params.fov_p_rad / params.dphi);
    int rays_per_candidate = THETA_BINS * window_height;

    size_t buffer_size_all = (size_t)cands.count * rays_per_candidate * sizeof(float);
    size_t buffer_size = (size_t)cands.count * window_width * window_height * sizeof(float);
    size_t res_size = cands.count * sizeof(float);

    float3* d_positions = upload_candidates(cands);
    float* d_results_gain;
    float* d_results_yaw;
    float* d_depth_buffer_all;
    float* d_depth_buffer;
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);
    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);

    MapContext m = context_of(map);

    evaluate_gain_kernel_depth<<<cands.count, min(rays_per_candidate, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, d_depth_buffer_all, d_depth_buffer, params);
    cudaDeviceSynchronize();

    cudaMemcpy(out.gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);
    if (out.depths != nullptr) {
        cudaMemcpy(out.depths, d_depth_buffer, buffer_size, cudaMemcpyDeviceToHost);
    }

    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    cudaFree(d_depth_buffer_all);
    cudaFree(d_depth_buffer);
}

// Shared setup for the single-parent marginal launchers (v1 and v2): allocate
// device buffers, upload the candidate + parent state, and assemble the kernel
// argument structs. Returns the rays-per-candidate launch width.
static int setup_single_parent_marginal(
    const GpuMap& map, GpuVec3 cand, const GpuParent& parent_in,
    const KernelParams& params, const gpuray::ParentCameraConfig& cam,
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

    float3 h_pos = make_float3(cand.x, cand.y, cand.z);
    cudaMemcpy(*d_cand_pos, &h_pos, sizeof(float3), cudaMemcpyHostToDevice);

    if (parent_in.depth != nullptr) {
        cudaMemcpy(d_parent_depth_buffer, parent_in.depth, buffer_size, cudaMemcpyHostToDevice);
    } else {
        cudaMemset(d_parent_depth_buffer, 0, buffer_size);
    }

    *m = context_of(map);

    const float* R = parent_in.R;
    gpuray::RotationRows rot = {
        make_float3(R[0], R[1], R[2]),
        make_float3(R[3], R[4], R[5]),
        make_float3(R[6], R[7], R[8])};
    *parent = ParentFrame{make_float3(parent_in.pos.x, parent_in.pos.y, parent_in.pos.z),
                          d_parent_depth_buffer, rot, cam};
    *out = GainResults{d_res_gain, d_res_yaw, d_depth_buffer_all, d_depth_buffer};
    return rays_per_candidate;
}

// Download results then release every device buffer held in a GainResults plus
// the (single) parent depth buffer and candidate position.
static void teardown_single_parent_marginal(
    float3* d_cand_pos, const ParentFrame& parent, const GainResults& out,
    const gpuray::ParentCameraConfig& cam, GpuResult result) {

    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);
    cudaMemcpy(result.gain, out.gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(result.yaw, out.yaw, sizeof(float), cudaMemcpyDeviceToHost);
    if (result.depths != nullptr) {
        cudaMemcpy(result.depths, out.depth, buffer_size, cudaMemcpyDeviceToHost);
    }
    cudaFree(d_cand_pos);
    cudaFree(out.gain);
    cudaFree(out.yaw);
    cudaFree(out.depth_all);
    cudaFree(out.depth);
    cudaFree(const_cast<float*>(parent.depth));
}

extern "C" void launch_marginal_gain_kernel(GpuMap map, GpuVec3 cand, GpuParent parent_in,
                                            GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    ParentFrame parent;
    GainResults res;
    int rays = setup_single_parent_marginal(map, cand, parent_in, params, cam,
                                            &m, &d_cand_pos, &parent, &res);

    evaluate_marginal_gain_kernel<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, parent, res, params);
    cudaDeviceSynchronize();

    teardown_single_parent_marginal(d_cand_pos, parent, res, cam, out);
}

extern "C" void launch_marginal_gain_kernel_v2(GpuMap map, GpuVec3 cand, GpuParent parent_in,
                                               GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    ParentFrame parent;
    GainResults res;
    int rays = setup_single_parent_marginal(map, cand, parent_in, params, cam,
                                            &m, &d_cand_pos, &parent, &res);

    evaluate_marginal_gain_kernel_v2<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, parent, res, params);
    cudaDeviceSynchronize();

    teardown_single_parent_marginal(d_cand_pos, parent, res, cam, out);
}

// Shared setup for the multi-ancestor marginal launchers (v3 and v4): allocate
// device buffers, upload the candidate + flattened ancestor chain, and assemble
// the kernel argument structs. Returns the rays-per-candidate launch width.
static int setup_multi_ancestor_marginal(
    const GpuMap& map, GpuVec3 cand, const GpuAncestors& ancestors_in,
    const KernelParams& params, const gpuray::ParentCameraConfig& cam,
    MapContext* m, float3** d_cand_pos, AncestorSet* ancestors, GainResults* out) {

    int n = ancestors_in.count;
    int rows_in_fov = max(1, (int)floor(params.fov_p_rad / params.dphi));
    int rays_per_candidate = THETA_BINS * rows_in_fov;
    size_t buffer_size_all = (size_t)rays_per_candidate * sizeof(float);
    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);
    size_t per = (size_t)cam.p_width * cam.p_height;   // depth elements per ancestor

    // Candidate + output buffers.
    float* d_res_gain;
    float* d_res_yaw;
    float* d_depth_buffer_all;
    float* d_depth_buffer;
    cudaMalloc(d_cand_pos, sizeof(float3));
    cudaMalloc(&d_res_gain, sizeof(float));
    cudaMalloc(&d_res_yaw, sizeof(float));
    cudaMalloc(&d_depth_buffer_all, buffer_size_all);
    cudaMalloc(&d_depth_buffer, buffer_size);

    // Flattened ancestor state (host layouts match float3 / 3xfloat3 packing).
    float3* d_parent_pos;
    float3* d_parent_R;
    float*  d_parent_yaw;
    float*  d_parent_depth;
    cudaMalloc(&d_parent_pos,   n * sizeof(float3));
    cudaMalloc(&d_parent_R,     n * 3 * sizeof(float3));
    cudaMalloc(&d_parent_yaw,   n * sizeof(float));
    cudaMalloc(&d_parent_depth, n * per * sizeof(float));

    float3 h_pos = make_float3(cand.x, cand.y, cand.z);
    cudaMemcpy(*d_cand_pos, &h_pos, sizeof(float3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_pos, ancestors_in.pos, n * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_R,   ancestors_in.R,   n * 9 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_parent_yaw, ancestors_in.yaw, n * sizeof(float),     cudaMemcpyHostToDevice);
    if (ancestors_in.depth != nullptr) {
        cudaMemcpy(d_parent_depth, ancestors_in.depth, n * per * sizeof(float), cudaMemcpyHostToDevice);
    } else {
        cudaMemset(d_parent_depth, 0, n * per * sizeof(float));
    }

    *m = context_of(map);
    *ancestors = AncestorSet{d_parent_pos, d_parent_yaw, d_parent_depth, d_parent_R, n, cam};
    *out = GainResults{d_res_gain, d_res_yaw, d_depth_buffer_all, d_depth_buffer};
    return rays_per_candidate;
}

// Download results then release every device buffer held in a GainResults plus
// the flattened ancestor state and candidate position.
static void teardown_multi_ancestor_marginal(
    float3* d_cand_pos, const AncestorSet& ancestors, const GainResults& out,
    const gpuray::ParentCameraConfig& cam, GpuResult result) {

    size_t buffer_size = (size_t)cam.p_width * cam.p_height * sizeof(float);
    cudaMemcpy(result.gain, out.gain, sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(result.yaw, out.yaw, sizeof(float), cudaMemcpyDeviceToHost);
    if (result.depths != nullptr) {
        cudaMemcpy(result.depths, out.depth, buffer_size, cudaMemcpyDeviceToHost);
    }
    cudaFree(d_cand_pos);
    cudaFree(out.gain);
    cudaFree(out.yaw);
    cudaFree(out.depth_all);
    cudaFree(out.depth);
    cudaFree(const_cast<float3*>(ancestors.positions));
    cudaFree(const_cast<float3*>(ancestors.R_rows));
    cudaFree(const_cast<float*>(ancestors.yaws));
    cudaFree(const_cast<float*>(ancestors.depth));
}

extern "C" void launch_marginal_gain_kernel_v3(GpuMap map, GpuVec3 cand, GpuAncestors ancestors_in,
                                               GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    AncestorSet ancestors;
    GainResults res;
    int rays = setup_multi_ancestor_marginal(map, cand, ancestors_in, params, cam,
                                             &m, &d_cand_pos, &ancestors, &res);

    evaluate_marginal_gain_kernel_v3<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, ancestors, res, params);
    cudaDeviceSynchronize();

    teardown_multi_ancestor_marginal(d_cand_pos, ancestors, res, cam, out);
}

extern "C" void launch_marginal_gain_kernel_v4(GpuMap map, GpuVec3 cand, GpuAncestors ancestors_in,
                                               GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    AncestorSet ancestors;
    GainResults res;
    int rays = setup_multi_ancestor_marginal(map, cand, ancestors_in, params, cam,
                                             &m, &d_cand_pos, &ancestors, &res);

    evaluate_marginal_gain_kernel_v4<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, ancestors, res, params);
    cudaDeviceSynchronize();

    teardown_multi_ancestor_marginal(d_cand_pos, ancestors, res, cam, out);
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
