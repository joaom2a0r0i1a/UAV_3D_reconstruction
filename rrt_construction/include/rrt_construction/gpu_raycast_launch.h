#ifndef RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
#define RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
//  GPU launcher ABI -- implemented in gpu_raycaster.cu, consumed by host code.
//
//  Launcher arguments are grouped into small POD structs so each entry point
//  stays within the JPL 5-argument limit and shares one definition across the
//  .cu / .cpp boundary. Every struct is a plain C layout and is safe to pass
//  by value across extern "C".
// ============================================================================

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

// A whole wavefront of candidates, each with its own ancestor chain, in CSR form:
// candidate c owns ancestors [offsets[c], offsets[c+1]) in the flattened arrays.
// `total` == offsets[num_candidates]. Depth buffers are packed per=p_width*p_height
// floats per ancestor. All host pointers; the launcher uploads them.
// `depth_idx` (optional) turns `depth` into a shared pool of `num_nodes` buffers:
// ancestor slot i reads pool[depth_idx[i]] instead of contiguous slot i. This keeps
// device memory O(num_nodes*per) instead of O(total*per) for large wavefronts.
// Leave depth_idx null (num_nodes ignored) for the contiguous layout.
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

// --- Expected-information-gain launchers ------------------------------------
void launch_gain_kernel_single(GpuMap map, GpuVec3 cand, GpuResult out, GpuSensor cfg);
void launch_gain_kernel(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);
void launch_gain_kernel_batch(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);
void launch_gain_kernel_batch_depth(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg);

// --- Marginal-gain launchers (v3 is the live path; v1/v2 are legacy) --------
// v4 mirrors v3 but traverses the observed-free spans instead of jumping them.
void launch_marginal_gain_kernel(GpuMap map, GpuVec3 cand, GpuParent parent,
                                 GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v2(GpuMap map, GpuVec3 cand, GpuParent parent,
                                    GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v3(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                    GpuResult out, GpuSensor cfg);
void launch_marginal_gain_kernel_v4(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                    GpuResult out, GpuSensor cfg);

// --- Batched marginal-gain launchers (whole wavefront in one call) ----------
// Two architectures over identical inputs, for head-to-head profiling (both use
// the v4 traverse march):
//   fused = Option 1: one kernel per candidate does check-then-march per ray.
//   split = Option 2: kernel A writes merged skip intervals to global memory,
//           kernel B reads them back and marches.
// map.d_map must already be a DEVICE grid. out.gain/out.yaw are [num_candidates];
// out.depths (or null) is [num_candidates * p_width * p_height]. If kernel_ms is
// non-null it receives the on-device kernel time (ms), excluding host alloc/copy.
void launch_marginal_gain_batch_fused(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms);
void launch_marginal_gain_batch_split(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms);

// --- Thin device-memory wrappers (host owns the cached map buffer) ----------
void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
void wrapper_cuda_free(void* dev_ptr);
void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif  // RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
