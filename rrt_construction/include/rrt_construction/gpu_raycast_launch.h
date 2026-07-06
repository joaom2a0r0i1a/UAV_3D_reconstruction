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

// --- Thin device-memory wrappers (host owns the cached map buffer) ----------
void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
void wrapper_cuda_free(void* dev_ptr);
void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif  // RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
