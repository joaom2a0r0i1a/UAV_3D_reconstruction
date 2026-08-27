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
    uint8_t* d_map;     // device occupancy grid
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

// Full ancestor chain for multi-frustum marginal gain; single-parent passes count=1.
typedef struct {
    int count;
    float* pos;     // 3*count  (x,y,z per ancestor)
    float* yaw;     // count
    float* R;       // 9*count  (row-major rows per ancestor)
    float* depth;   // count*p_width*p_height, or null
} GpuAncestors;

// CSR wavefront: candidate c owns ancestors [offsets[c], offsets[c+1]); depth lives in the persistent pool (d_pool), indexed by each ancestor's GLOBAL depth_idx.
typedef struct {
    int          num_candidates;
    const int*   offsets;   // [num_candidates+1] prefix sum of per-candidate ancestor counts
    int          total;     // total ancestors across the batch
    const float* pos;       // [3*total]  (x,y,z per ancestor)
    const float* yaw;       // [total]
    const float* R;         // [9*total]  (row-major rows per ancestor)
    const int*   depth_idx; // [total] GLOBAL pool slot per ancestor
} GpuAncestorBatch;

#ifdef __cplusplus
extern "C" {
#endif


/* EXPECTED-INFORMATION-GAIN LAUNCHERS */
void launch_absolute_gain_batch(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg, float* kernel_ms);


/* SINGLE-NODE MARGINAL-GAIN LAUNCHER (one kernel over an ancestor set: count=1 = single-parent, N = multi-ancestor) */
void launch_marginal_gain(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                 GpuResult out, GpuSensor cfg);


/* BATCHED MARGINAL-GAIN LAUNCHERS (whole wavefront; fused vs split, both GPU-resident-pool: ancestor depth is
   read in place from d_pool via anc.depth_idx, each candidate writes its render to d_pool[out_slot[c]];
   kernel_ms=device ms; fixed_yaws=NBVP per-candidate or null=AEP) */
void launch_marginal_gain_batch_fused(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms,
                                      const float* fixed_yaws,
                                      float* d_pool, const int* out_slot);
void launch_marginal_gain_batch_split(GpuMap map, GpuCandidates cands,
                                      GpuAncestorBatch anc, GpuResult out,
                                      GpuSensor cfg, float* kernel_ms,
                                      const float* fixed_yaws,
                                      float* d_pool, const int* out_slot);


/* FIXED-YAW VARIANTS (NBVP): eval the FOV window at fixed_yaws[i] instead of optimizing yaw; out.yaw = input yaw */
void launch_absolute_gain_batch_fixed(GpuMap map, GpuCandidates cands, GpuResult out,
                                    GpuSensor cfg, const float* fixed_yaws, float* kernel_ms);
void launch_marginal_gain_fixed(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                       GpuResult out, GpuSensor cfg, float fixed_yaw);


/* PERSISTENT DEPTH-POOL DEVICE MEMORY (host owns *d_pool + *capacity; grow preserves contents, new region = -1). */
void wrapper_depth_pool_ensure(float** d_pool, int* capacity, int need, int per);
void wrapper_depth_pool_free(float* d_pool);
void wrapper_depth_slot_to_host(const float* d_pool, int slot, int per, float* host_out);


/* THIN DEVICE-MEMORY WRAPPERS (host owns the cached map buffer) */
void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
void wrapper_cuda_free(void* dev_ptr);
void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif  // RRT_CONSTRUCTION_GPU_RAYCAST_LAUNCH_H
