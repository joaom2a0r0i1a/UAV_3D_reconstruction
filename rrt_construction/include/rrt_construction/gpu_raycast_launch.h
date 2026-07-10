#ifndef RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
#define RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H

#include <stdint.h>
#include <stddef.h>

// Angular sample count = floor(span/step + snap-tolerance); one shared CPU+GPU definition.
static inline int angular_bins(float span, float step) {
    int n = (int)(span / step + 1e-3f);
    return n > 0 ? n : 1;
}


/* GPU LAUNCHER ABI (impl in gpu_raycaster.cu; args bundled into POD structs, passed by value across extern "C") */

// World-space point (host mirror of CUDA float3).
typedef struct {
    float x, y, z;
} GpuVec3;

// Occupancy grid cached on the GPU: device buffer + dimensions + world origin.
typedef struct {
    uint8_t* d_map;     // device occupancy grid (host grid for launch_gain_kernel)
    int dx, dy, dz;     // grid dimensions (voxels)
    float ox, oy, oz;   // world position of voxel (0,0,0)
} GpuMap;

// Sensor + evaluation tunables shared by every launcher.
typedef struct {
    float voxel_size;   // voxel edge length (m)
    float gain_range;   // maximum ray range (m)
    float fov_y;        // horizontal field of view (rad)
    float fov_p;        // vertical field of view (rad)
    float pitch;        // camera pitch (rad)
} GpuSensor;

// Batch of candidate positions as separate x/y/z host arrays.
typedef struct {
    float* x;
    float* y;
    float* z;
    int count;
} GpuCandidates;

// Host output buffers for one or many candidates (depths may be null).
typedef struct {
    float* gain;
    float* yaw;
    float* depths;
} GpuResult;

// Single observing parent frame (v1/v2 marginal gain).
typedef struct {
    GpuVec3 pos;
    float yaw;
    float* R;       // 9 floats, row-major rotation
    float* depth;   // p_width*p_height depth image, or null
} GpuParent;

// Full ancestor chain for multi-frustum marginal gain (v3, the live path).
typedef struct {
    int count;
    float* pos;     // 3*count  (x,y,z per ancestor)
    float* yaw;     // count
    float* R;       // 9*count  (row-major rows per ancestor)
    float* depth;   // count*p_width*p_height, or null
} GpuAncestors;

// Wavefront in CSR form: candidate c owns ancestors [offsets[c], offsets[c+1]); depth_idx (opt) pools depth to O(num_nodes*per).
typedef struct {
    int          num_candidates;
    const int*   offsets;   // [num_candidates+1] prefix sum of per-candidate ancestor counts
    int          total;     // total ancestors across the batch
    const float* pos;       // [3*total]  (x,y,z per ancestor)
    const float* yaw;       // [total]
    const float* R;         // [9*total]  (row-major rows per ancestor)
    const float* depth;     // [total*per] contiguous, or [num_nodes*per] pool if depth_idx set
    const int*   depth_idx; // [total] pool index per ancestor slot, or null (contiguous)
    int          num_nodes; // pool size when depth_idx set
} GpuAncestorBatch;

#ifdef __cplusplus
extern "C" {
#endif


/* EXPECTED-INFORMATION-GAIN LAUNCHERS */
void launch_gain_kernel_single(GpuMap map, GpuVec3 cand, GpuResult out, GpuSensor cfg);
void launch_gain_kernel(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);
void launch_gain_kernel_batch(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);
void launch_gain_kernel_batch_depth(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);


/* MARGINAL-GAIN LAUNCHERS (v3 live; v1/v2 legacy; v4 = v3 but traverses spans instead of jumping) */
void launch_marginal_gain_kernel(GpuMap map, GpuVec3 cand, GpuParent parent,
                                 GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v2(GpuMap map, GpuVec3 cand, GpuParent parent,
                                    GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v3(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                    GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v4(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                    GpuResult out, GpuSensor cfg);


/* BATCHED MARGINAL-GAIN LAUNCHERS (whole wavefront; fused vs split, both v4 march; kernel_ms=device ms; fixed_yaws=NBVP per-candidate or null=AEP) */
void launch_marginal_gain_batch_fused(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms,
                                      const float* fixed_yaws);
void launch_marginal_gain_batch_split(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms,
                                      const float* fixed_yaws);


/* FIXED-YAW VARIANTS (NBVP): eval the FOV window at fixed_yaws[i] instead of optimizing yaw; out.yaw = input yaw */
void launch_gain_kernel_batch_fixed(GpuMap map, GpuCandidates cands, GpuResult out,
                                    GpuSensor cfg, const float* fixed_yaws);
void launch_marginal_gain_kernel_v2_fixed(GpuMap map, GpuVec3 cand, GpuParent parent,
                                          GpuResult out, GpuSensor cfg, float fixed_yaw);


/* THIN DEVICE-MEMORY WRAPPERS (host owns the cached map buffer) */
void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
void wrapper_cuda_free(void* dev_ptr);
void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif  // RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
