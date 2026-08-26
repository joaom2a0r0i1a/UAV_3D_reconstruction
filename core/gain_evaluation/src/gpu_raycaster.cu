#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math_constants.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <gain_evaluation/gpu_raycast_math.cuh>
#include <gain_evaluation/gpu_raycast_launch.h>

// Candidates per tile for the split launcher's interval scratch (bounds device memory).
#define SPLIT_CHUNK 128

// gpu_raycaster.cu -- CUDA front end: __global__ kernels (math in gpu_raycast_math.cuh) + extern "C" launchers (ABI in gpu_raycast_launch.h).


/* HOST HELPERS (shared by every launcher) */

// Gain-sphere angular bins: 180 (2 deg) at the 5 m / 0.2 m reference, finer with range and inversely with voxel; bins tile 360 deg.
static void set_angular_resolution(KernelParams& params, float voxel_size, float gain_range) {
    int bins = (int)floorf(180.0f * (gain_range / 5.0f) * (0.2f / voxel_size) + 1e-3f);
    if (bins < 1) bins = 1;
    if (bins > THETA_BINS_MAX) bins = THETA_BINS_MAX;
    
    params.theta_bins = bins;
    params.dtheta = 2.0f * CUDART_PI_F / bins;
    params.dphi = 2.0f * CUDART_PI_F / bins;
}

// Pack the dynamic launch parameters, deriving the angular steps and phi band.
static KernelParams make_kernel_params(float voxel_size, float gain_range,
                                       float fov_y, float fov_p, float pitch) {
    KernelParams params;
    params.voxel_size   = voxel_size;
    params.gain_range   = gain_range;
    params.fov_y_rad    = fov_y;
    params.fov_p_rad    = fov_p;
    params.camera_pitch = pitch;

    set_angular_resolution(params, voxel_size, gain_range);

    float phi_center = (CUDART_PI_F * 0.5f) + params.camera_pitch;
    params.phi_start = phi_center - (params.fov_p_rad * 0.5f);
    params.phi_end   = phi_center + (params.fov_p_rad * 0.5f);

    // Single source of truth for the sample counts (shared with the CPU sweeps).
    params.rows_in_fov    = angular_bins(params.fov_p_rad, params.dphi);
    params.sectors_in_fov = angular_bins(params.fov_y_rad, params.dtheta);
    return params;
}

// Derive the parent depth-image geometry (pixel size + pinhole intrinsics) shared by every marginal kernel.
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

// Pick the FOV window from the per-sector histogram: caller's fixed_yaw (if non-null) or the best window.
__device__ inline void pick_yaw_window(const float* s_yaw_gains, const KernelParams& params,
                                       const float* fixed_yaw, int candidate,
                                       float* out_gain, float* out_center) {
    if (fixed_yaw) {
        float c = fixed_yaw[candidate];
        *out_center = c;
        *out_gain   = gpuray::window_gain_at_yaw(s_yaw_gains, params.theta_bins, params.sectors_in_fov,
                                                 params.dtheta, params.fov_y_rad, c);
    } else {
        float mg;
        int best   = gpuray::best_yaw_start_index(s_yaw_gains, params.theta_bins, params.sectors_in_fov, &mg);
        *out_center = gpuray::yaw_window_center_angle(best, params.dtheta, params.fov_y_rad);
        *out_gain   = mg;
    }
}


/* KERNELS: AEP INFORMATION GAIN (no parent occlusion) */

// One candidate per block, batched over many candidates.
__global__ void evaluate_gain_kernel(MapContext m, const float3* __restrict__ positions,
                                    float* __restrict__ results_gain,
                                    float* __restrict__ results_yaw,
                                    KernelParams params,
                                    const float* __restrict__ fixed_yaw = nullptr) {
    __shared__ float s_yaw_gains[THETA_BINS_MAX];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < params.theta_bins) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % params.theta_bins;
        int phi_idx   = idx / params.theta_bins;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        float sin_phi = sinf(phi);
        float3 dir = gpuray::spherical_ray_dir(theta, phi);

        float depth;
        MarchRay ray = {positions[candidate], dir, sin_phi};
        float ray_gain = march_gain_basic(m, ray, params, &depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    if (ray_id == 0) {
        float max_gain, center;
        pick_yaw_window(s_yaw_gains, params, fixed_yaw, candidate, &max_gain, &center);
        results_gain[candidate] = max_gain;
        results_yaw[candidate]  = center;
    }
}


/* KERNELS: MARGINAL INFORMATION GAIN (subtract what ancestors saw) */

// Single-node marginal gain: traverse-march over an ancestor set (count=1 = single-parent, N = full chain); honors fixed or optimized yaw.
__global__ void evaluate_marginal_gain_single_node(MapContext m, const float3* __restrict__ positions,
                                                   AncestorSet ancestors, GainResults out,
                                                   KernelParams params) {
    __shared__ float s_yaw_gains[THETA_BINS_MAX];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < params.theta_bins) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov    = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % params.theta_bins;
        int phi_idx   = idx / params.theta_bins;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
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
        float max_gain, center;
        pick_yaw_window(s_yaw_gains, params, out.fixed_yaw, candidate, &max_gain, &center);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ancestors.cam.p_width * ancestors.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, ancestors.cam, pose, params, out.depth + candidate * buffer_rays);
}


/* BATCHED MARGINAL GAIN KERNELS (one wavefront per grid) */

// Two architectures share this data view; only the kernel structure differs.

// Option 1 (fused): one block per candidate; each ray does check-then-march (traverse march).
__global__ void evaluate_marginal_gain_batch_fused(MapContext m, const float3* __restrict__ positions,
                                                   AncestorBatchDev ab, GainResults out,
                                                   KernelParams params, int depth_slots) {
    __shared__ float s_yaw_gains[THETA_BINS_MAX];
    int candidate = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < params.theta_bins) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    AncestorSet ancestors = ancestors_for(ab, candidate);

    int rows_in_fov    = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % params.theta_bins;
        int phi_idx   = idx / params.theta_bins;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float phi   = params.phi_start + phi_idx * params.dphi;
        float sin_phi = sinf(phi);
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);
        float3 cam_pos = positions[candidate];

        // Merge observed-free spans across every ancestor, in voxel units.
        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        Ray ray = {cam_pos, ray_dir};
        SkipBuffer skip_out = {skip_m, &skip_count, MAX_SEGS, &status};
        compute_multi_segment_skip_distance(ancestors, ray, params, skip_out);

        float2 skip_vox[MAX_SEGS];
        for (int i = 0; i < skip_count; ++i)
            skip_vox[i] = make_float2(skip_m[i].x / params.voxel_size, skip_m[i].y / params.voxel_size);

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        SkipSet skips = {skip_vox, skip_count};
        float ray_gain = march_marginal_gain_traverse(m, mray, skips, params, &final_depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain, center;
        pick_yaw_window(s_yaw_gains, params, out.fixed_yaw, candidate, &max_gain, &center);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ab.cam.p_width * ab.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, ab.cam, pose, params, out.depth + (candidate % depth_slots) * buffer_rays);
}

// Option 2, stage A: each ray writes its merged skip intervals (voxel units) + count to global scratch (LOCAL block index).
__global__ void marginal_skips_stage(const float3* __restrict__ positions, AncestorBatchDev ab,
                                     int cand_base, KernelParams params, float2* __restrict__ skips_out,
                                     int* __restrict__ counts_out, int max_segs) {
    int candidate = cand_base + blockIdx.x;
    int local = blockIdx.x;
    AncestorSet ancestors = ancestors_for(ab, candidate);

    int rows_in_fov = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;
    float3 cam_pos = positions[candidate];

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        long ray_slot = (long)local * rays_per_candidate + idx;
        int theta_idx = idx % params.theta_bins;
        int phi_idx   = idx / params.theta_bins;
        float phi   = params.phi_start + phi_idx * params.dphi;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);

        const int MAX_SEGS = 32;
        float2 skip_m[MAX_SEGS];
        int skip_count = 0;
        float status = 1.0f;
        Ray ray = {cam_pos, ray_dir};
        SkipBuffer skip_out = {skip_m, &skip_count, MAX_SEGS, &status};
        compute_multi_segment_skip_distance(ancestors, ray, params, skip_out);

        int n = min(skip_count, max_segs);
        float2* dst = skips_out + ray_slot * max_segs;
        for (int i = 0; i < n; ++i) {
            dst[i] = make_float2(skip_m[i].x / params.voxel_size,
                                 skip_m[i].y / params.voxel_size);
        }
        counts_out[ray_slot] = n;
    }
}

// Option 2, stage B: reads stage A's merged skip intervals from global memory and marches (traverse).
__global__ void marginal_march_stage(MapContext m, const float3* __restrict__ positions,
                                     AncestorBatchDev ab, int cand_base, GainResults out,
                                     KernelParams params, const float2* __restrict__ skips_in,
                                     const int* __restrict__ counts_in, int max_segs, int depth_slots) {
    __shared__ float s_yaw_gains[THETA_BINS_MAX];
    int candidate = cand_base + blockIdx.x;
    int local = blockIdx.x;
    int ray_id = threadIdx.x;
    if (ray_id < params.theta_bins) s_yaw_gains[ray_id] = 0.0f;
    __syncthreads();

    int rows_in_fov    = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;
    float3 cam_pos = positions[candidate];

    for (int idx = threadIdx.x; idx < rays_per_candidate; idx += blockDim.x) {
        int theta_idx = idx % params.theta_bins;
        int phi_idx   = idx / params.theta_bins;
        float phi   = params.phi_start + phi_idx * params.dphi;
        float theta = -CUDART_PI_F + theta_idx * params.dtheta;
        float sin_phi = sinf(phi);
        float3 ray_dir = gpuray::spherical_ray_dir(theta, phi);

        long ray_slot = (long)local * rays_per_candidate + idx;
        SkipSet skips = {skips_in + ray_slot * max_segs, counts_in[ray_slot]};

        float final_depth;
        MarchRay mray = {cam_pos, ray_dir, sin_phi};
        float ray_gain = march_marginal_gain_traverse(m, mray, skips, params, &final_depth);
        if (ray_gain > 0.0f) atomicAdd(&s_yaw_gains[theta_idx], ray_gain);
    }
    __syncthreads();

    __shared__ float s_best_yaw;
    if (ray_id == 0) {
        float max_gain, center;
        pick_yaw_window(s_yaw_gains, params, out.fixed_yaw, candidate, &max_gain, &center);
        out.gain[candidate] = max_gain;
        out.yaw[candidate]  = center;
        s_best_yaw = center;
    }
    __syncthreads();

    int buffer_rays = ab.cam.p_width * ab.cam.p_height;
    CameraPose pose = {positions[candidate], s_best_yaw};
    generate_depth_buffer(m, ab.cam, pose, params, out.depth + (candidate % depth_slots) * buffer_rays);
}


/* LAUNCHERS (extern "C" ABI -- consumed by gain_evaluator.cpp) */

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


/* AEP GAIN LAUNCHERS */
extern "C" void launch_absolute_gain_batch(GpuMap map, GpuCandidates cands,
                                        GpuResult out, GpuSensor cfg, float* kernel_ms) {
    KernelParams params = params_of(cfg);
    size_t res_size = cands.count * sizeof(float);

    float3* d_positions = upload_candidates(cands);
    float* d_results_gain;
    float* d_results_yaw;
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);

    MapContext m = context_of(map);

    cudaEvent_t t0, t1; cudaEventCreate(&t0); cudaEventCreate(&t1);
    int total_rays = params.theta_bins * params.rows_in_fov;
    cudaEventRecord(t0);
    evaluate_gain_kernel<<<cands.count, min(total_rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params);
    cudaEventRecord(t1);
    cudaEventSynchronize(t1);
    if (kernel_ms) cudaEventElapsedTime(kernel_ms, t0, t1);
    cudaEventDestroy(t0); cudaEventDestroy(t1);

    cudaMemcpy(out.gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
}

// Fixed-yaw absolute batch: gain of the FOV window at fixed_yaws[i]; out.yaw = input yaw.
extern "C" void launch_absolute_gain_batch_fixed(GpuMap map, GpuCandidates cands,
                                              GpuResult out, GpuSensor cfg,
                                              const float* fixed_yaws, float* kernel_ms) {
    KernelParams params = params_of(cfg);
    size_t res_size = cands.count * sizeof(float);

    float3* d_positions = upload_candidates(cands);
    float *d_results_gain, *d_results_yaw, *d_fixed_yaw;
    cudaMalloc(&d_results_gain, res_size);
    cudaMalloc(&d_results_yaw, res_size);
    cudaMalloc(&d_fixed_yaw, res_size);
    cudaMemcpy(d_fixed_yaw, fixed_yaws, res_size, cudaMemcpyHostToDevice);

    MapContext m = context_of(map);
    int total_rays = params.theta_bins * params.rows_in_fov;
    cudaEvent_t t0, t1; cudaEventCreate(&t0); cudaEventCreate(&t1);
    cudaEventRecord(t0);
    evaluate_gain_kernel<<<cands.count, min(total_rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_positions, d_results_gain, d_results_yaw, params, d_fixed_yaw);
    cudaEventRecord(t1);
    cudaEventSynchronize(t1);
    if (kernel_ms) cudaEventElapsedTime(kernel_ms, t0, t1);
    cudaEventDestroy(t0); cudaEventDestroy(t1);

    cudaMemcpy(out.gain, d_results_gain, res_size, cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw, d_results_yaw, res_size, cudaMemcpyDeviceToHost);

    cudaFree(d_positions);
    cudaFree(d_results_gain);
    cudaFree(d_results_yaw);
    cudaFree(d_fixed_yaw);
}


/* SINGLE-NODE MARGINAL LAUNCHERS (one kernel over an ancestor set: count=1 = single-parent, N = multi-ancestor; optimize or fixed yaw) */

// Shared setup for the single-node marginal launchers: alloc buffers, upload the flattened ancestor chain; returns rays-per-candidate.
static int setup_multi_ancestor_marginal(
    const GpuMap& map, GpuVec3 cand, const GpuAncestors& ancestors_in,
    const KernelParams& params, const gpuray::ParentCameraConfig& cam,
    MapContext* m, float3** d_cand_pos, AncestorSet* ancestors, GainResults* out) {

    int n = ancestors_in.count;
    int rows_in_fov = params.rows_in_fov;
    int rays_per_candidate = params.theta_bins * rows_in_fov;
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

// Download results, then free every GainResults device buffer plus the flattened ancestor state and candidate position.
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

// Optimize-yaw marginal gain over an ancestor set (count=1 = single-parent, N = full chain).
extern "C" void launch_marginal_gain(GpuMap map, GpuVec3 cand, GpuAncestors ancestors_in,
                                            GpuResult out, GpuSensor cfg) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    AncestorSet ancestors;
    GainResults res;
    int rays = setup_multi_ancestor_marginal(map, cand, ancestors_in, params, cam,
                                             &m, &d_cand_pos, &ancestors, &res);

    evaluate_marginal_gain_single_node<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, ancestors, res, params);
    cudaDeviceSynchronize();

    teardown_multi_ancestor_marginal(d_cand_pos, ancestors, res, cam, out);
}

// Fixed-yaw marginal gain over an ancestor set: evaluates the FOV window at `fixed_yaw` (out.yaw = fixed_yaw).
extern "C" void launch_marginal_gain_fixed(GpuMap map, GpuVec3 cand, GpuAncestors ancestors_in,
                                                  GpuResult out, GpuSensor cfg, float fixed_yaw) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m;
    float3* d_cand_pos;
    AncestorSet ancestors;
    GainResults res;
    int rays = setup_multi_ancestor_marginal(map, cand, ancestors_in, params, cam,
                                             &m, &d_cand_pos, &ancestors, &res);

    float* d_fixed_yaw;
    cudaMalloc(&d_fixed_yaw, sizeof(float));
    cudaMemcpy(d_fixed_yaw, &fixed_yaw, sizeof(float), cudaMemcpyHostToDevice);
    res.fixed_yaw = d_fixed_yaw;

    evaluate_marginal_gain_single_node<<<1, min(rays, MAX_THREADS_PER_BLOCK)>>>(
        m, d_cand_pos, ancestors, res, params);
    cudaDeviceSynchronize();

    cudaFree(d_fixed_yaw);
    teardown_multi_ancestor_marginal(d_cand_pos, ancestors, res, cam, out);
}


/* BATCHED MARGINAL LAUNCHERS (fused / split) */

// Shared device-memory setup/teardown for the two batched launchers (types in gpu_raycast_math.cuh).

// Upload the candidate batch + CSR ancestor chains + output buffers and build the AncestorBatchDev view (shared by fused/split).
static AncestorBatchDev setup_batch(const GpuMap& map, const GpuCandidates& cands,
                                    const GpuAncestorBatch& anc, const KernelParams& params,
                                    const gpuray::ParentCameraConfig& cam,
                                    MapContext* m, BatchDeviceMem* mem, GainResults* out,
                                    const float* fixed_yaws = nullptr) {
    int nc = cands.count;
    int total = anc.total;
    size_t per = (size_t)cam.p_width * cam.p_height;
    int rows_in_fov = params.rows_in_fov;
    int rays = params.theta_bins * rows_in_fov;
    int depth_slots = nc;

    float3* h_cand = new float3[nc];
    for (int i = 0; i < nc; ++i) h_cand[i] = make_float3(cands.x[i], cands.y[i], cands.z[i]);
    cudaMalloc(&mem->d_cand, nc * sizeof(float3));
    cudaMemcpy(mem->d_cand, h_cand, nc * sizeof(float3), cudaMemcpyHostToDevice);
    delete[] h_cand;

    cudaMalloc(&mem->d_off, (nc + 1) * sizeof(int));
    cudaMalloc(&mem->d_pos, total * sizeof(float3));
    cudaMalloc(&mem->d_yaw, total * sizeof(float));
    cudaMalloc(&mem->d_R,   total * 3 * sizeof(float3));
    cudaMemcpy(mem->d_off, anc.offsets, (nc + 1) * sizeof(int),            cudaMemcpyHostToDevice);
    cudaMemcpy(mem->d_pos, anc.pos,     (size_t)total * 3 * sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(mem->d_yaw, anc.yaw,     total * sizeof(float),             cudaMemcpyHostToDevice);
    cudaMemcpy(mem->d_R,   anc.R,       (size_t)total * 9 * sizeof(float), cudaMemcpyHostToDevice);

    // Depth: shared pool (num_nodes buffers + per-slot index) or contiguous slots.
    if (anc.depth_idx) {
        size_t pool = (size_t)anc.num_nodes * per;
        cudaMalloc(&mem->d_depth, pool * sizeof(float));
        cudaMemcpy(mem->d_depth, anc.depth, pool * sizeof(float), cudaMemcpyHostToDevice);
        cudaMalloc(&mem->d_depth_idx, total * sizeof(int));
        cudaMemcpy(mem->d_depth_idx, anc.depth_idx, total * sizeof(int), cudaMemcpyHostToDevice);
    } else {
        cudaMalloc(&mem->d_depth, (size_t)total * per * sizeof(float));
        cudaMemcpy(mem->d_depth, anc.depth, (size_t)total * per * sizeof(float), cudaMemcpyHostToDevice);
        mem->d_depth_idx = nullptr;
    }

    cudaMalloc(&mem->d_gain,      nc * sizeof(float));
    cudaMalloc(&mem->d_yaw_out,   nc * sizeof(float));
    cudaMalloc(&mem->d_depth_buf, (size_t)depth_slots * per * sizeof(float));

    // Per-candidate fixed yaw (null -> optimize yaw, the AEP path).
    if (fixed_yaws) {
        cudaMalloc(&mem->d_fixed_yaw, nc * sizeof(float));
        cudaMemcpy(mem->d_fixed_yaw, fixed_yaws, nc * sizeof(float), cudaMemcpyHostToDevice);
    } else {
        mem->d_fixed_yaw = nullptr;
    }

    mem->rays = rays; mem->nc = nc; mem->depth_slots = depth_slots; mem->per = per;

    *m = context_of(map);
    // depth_all (per-ray planar depth) is unused by the batch marginal path.
    *out = GainResults{mem->d_gain, mem->d_yaw_out, nullptr, mem->d_depth_buf};
    out->fixed_yaw = mem->d_fixed_yaw;
    AncestorBatchDev ab = {mem->d_off, mem->d_pos, mem->d_yaw, mem->d_depth,
                           mem->d_R, (int)per, cam, mem->d_depth_idx};
    return ab;
}

static void teardown_batch(const BatchDeviceMem& mem, GpuResult out) {
    cudaMemcpy(out.gain, mem.d_gain,    mem.nc * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(out.yaw,  mem.d_yaw_out, mem.nc * sizeof(float), cudaMemcpyDeviceToHost);
    if (out.depths != nullptr) {   // bounded scratch; the benchmark path passes null
        cudaMemcpy(out.depths, mem.d_depth_buf,
                   (size_t)mem.depth_slots * mem.per * sizeof(float), cudaMemcpyDeviceToHost);
    }
    cudaFree(mem.d_cand); cudaFree(mem.d_off);   cudaFree(mem.d_pos);    cudaFree(mem.d_yaw);
    cudaFree(mem.d_R);    cudaFree(mem.d_depth);
    if (mem.d_depth_idx) cudaFree(mem.d_depth_idx);
    cudaFree(mem.d_gain); cudaFree(mem.d_yaw_out); cudaFree(mem.d_depth_buf);
    if (mem.d_fixed_yaw) cudaFree(mem.d_fixed_yaw);
}

// Option 1 (fused): one kernel, grid=candidates, threads=rays; fixed_yaws (or null) picks each window yaw.
extern "C" void launch_marginal_gain_batch_fused(GpuMap map, GpuCandidates cands,
                                                 GpuAncestorBatch anc, GpuResult out,
                                                 GpuSensor cfg, float* kernel_ms,
                                                 const float* fixed_yaws) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m; BatchDeviceMem mem; GainResults res;
    AncestorBatchDev ab = setup_batch(map, cands, anc, params, cam, &m, &mem, &res, fixed_yaws);

    cudaEvent_t t0, t1; cudaEventCreate(&t0); cudaEventCreate(&t1);
    cudaEventRecord(t0);
    evaluate_marginal_gain_batch_fused<<<mem.nc, min(mem.rays, MAX_THREADS_PER_BLOCK)>>>(
        m, mem.d_cand, ab, res, params, mem.depth_slots);
    cudaEventRecord(t1);
    cudaEventSynchronize(t1);
    if (kernel_ms) cudaEventElapsedTime(kernel_ms, t0, t1);
    cudaEventDestroy(t0); cudaEventDestroy(t1);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) printf("CUDA fused batch error: %s\n", cudaGetErrorString(err));

    teardown_batch(mem, out);
}

// Option 2 (split): stage A writes merged skip intervals to global memory, stage B reads them back and marches; fixed_yaws optional.
extern "C" void launch_marginal_gain_batch_split(GpuMap map, GpuCandidates cands,
                                                 GpuAncestorBatch anc, GpuResult out,
                                                 GpuSensor cfg, float* kernel_ms,
                                                 const float* fixed_yaws) {
    KernelParams params = params_of(cfg);
    gpuray::ParentCameraConfig cam = derive_camera_config(cfg.gain_range, cfg.voxel_size, params);

    MapContext m; BatchDeviceMem mem; GainResults res;
    AncestorBatchDev ab = setup_batch(map, cands, anc, params, cam, &m, &mem, &res, fixed_yaws);

    // Interval scratch bounded to one chunk and reused across chunks (the split must tile; the fused kernel doesn't).
    const int max_segs = 32;                       // merged capacity per ray in scratch
    const int chunk = min(mem.nc, SPLIT_CHUNK);
    size_t nslots = (size_t)chunk * mem.rays;
    float2* d_skips; int* d_counts;
    cudaMalloc(&d_skips,  nslots * max_segs * sizeof(float2));
    cudaMalloc(&d_counts, nslots * sizeof(int));

    int threads = min(mem.rays, MAX_THREADS_PER_BLOCK);
    cudaEvent_t t0, t1; cudaEventCreate(&t0); cudaEventCreate(&t1);
    cudaEventRecord(t0);
    for (int c0 = 0; c0 < mem.nc; c0 += chunk) {
        int blocks = min(chunk, mem.nc - c0);
        marginal_skips_stage<<<blocks, threads>>>(mem.d_cand, ab, c0, params, d_skips, d_counts, max_segs);
        marginal_march_stage<<<blocks, threads>>>(m, mem.d_cand, ab, c0, res, params, d_skips, d_counts, max_segs, mem.depth_slots);
    }
    cudaEventRecord(t1);
    cudaEventSynchronize(t1);
    if (kernel_ms) cudaEventElapsedTime(kernel_ms, t0, t1);
    cudaEventDestroy(t0); cudaEventDestroy(t1);

    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) printf("CUDA split batch error: %s\n", cudaGetErrorString(err));

    cudaFree(d_skips); cudaFree(d_counts);
    teardown_batch(mem, out);
}


/* THIN CUDA MEMORY WRAPPERS (host owns the cached map buffer) */

extern "C" void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size) {
    cudaMalloc((void**)dev_ptr, size);
}

extern "C" void wrapper_cuda_free(void* dev_ptr) {
    if (dev_ptr) cudaFree(dev_ptr);
}

extern "C" void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size) {
    cudaMemcpy(dev_ptr, host_ptr, size, cudaMemcpyHostToDevice);
}
