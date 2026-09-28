#ifndef GPU_RAYCAST_LAUNCH_H
#define GPU_RAYCAST_LAUNCH_H

#include <stdint.h>
#include <stddef.h>

// Angular sample count, shared by CPU and GPU
static inline int angular_bins(float span, float step) {
    int n = (int)(span / step + 1e-3f);
    return n > 0 ? n : 1;
}

/*           GPU LAUNCHER ARGUMENTS          */

// World Point
typedef struct {
    float x, y, z;
} GpuVec3;

// Occupancy grid cached on the GPU
typedef struct {
    uint8_t* d_map;       // Device occupancy grid
    int      dx, dy, dz;  // Grid dimensions (voxels)
    float    ox, oy, oz;  // World position of voxel (0,0,0)
} GpuMap;

// Sensor and evaluation parameters shared by every launcher
typedef struct {
    float voxel_size;  // Voxel edge length (m)
    float gain_range;  // Maximum ray range (m)
    float fov_y;       // Horizontal field of view (rad)
    float fov_p;       // Vertical field of view (rad)
    float pitch;       // Camera pitch (rad)
} GpuSensor;

// Candidate Positions
typedef struct {
    float* x;
    float* y;
    float* z;
    int    count;
} GpuCandidates;

// Output Buffers
typedef struct {
    float* gain;
    float* yaw;
    float* depths;
} GpuResult;

// Ancestor chain of one candidate (count = 1 for single parent)
typedef struct {
    int    count;  // Number of ancestors
    float* pos;    // [3*count] x,y,z per ancestor
    float* yaw;    // [count] yaw per ancestor
    float* R;      // [9*count] row-major rotation per ancestor
    float* depth;  // [count*p_width*p_height] depth buffers, or null
} GpuAncestors;

// Ancestor chains of a batch: candidate c owns ancestors [offsets[c], offsets[c+1]),
// whose depth buffers live in the persistent depth pool
typedef struct {
    int          num_candidates;  // Number of candidates
    const int*   offsets;         // [num_candidates+1] prefix sum of per-candidate ancestor counts
    int          total;           // Total ancestors across the batch
    const float* pos;             // [3*total] x,y,z per ancestor
    const float* yaw;             // [total] yaw per ancestor
    const float* R;               // [9*total] row-major rotation per ancestor
    const int*   depth_idx;       // [total] depth pool slot per ancestor
} GpuAncestorBatch;

#ifdef __cplusplus
extern "C" {
#endif

/*               ABSOLUTE GAIN               */
void launch_absolute_gain_batch(GpuMap map, GpuCandidates cands, GpuResult out, GpuSensor cfg, float* kernel_ms);

/*           MARGINAL GAIN - SINGLE          */
void launch_marginal_gain(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                          GpuResult out, GpuSensor cfg);

/*          MARGINAL GAIN - BATCHED          */
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

/*                 FIXED YAW                 */
void launch_absolute_gain_batch_fixed(GpuMap map, GpuCandidates cands, GpuResult out,
                                      GpuSensor cfg, const float* fixed_yaws, float* kernel_ms);
void launch_marginal_gain_fixed(GpuMap map, GpuVec3 cand, GpuAncestors ancestors,
                                GpuResult out, GpuSensor cfg, float fixed_yaw);

/*                 DEPTH POOL                */
void wrapper_depth_pool_ensure(float** d_pool, int* capacity, int need, int per);
void wrapper_depth_pool_free(float* d_pool);
void wrapper_depth_slot_to_host(const float* d_pool, int slot, int per, float* host_out);

/*               DEVICE MEMORY               */
void wrapper_cuda_malloc(uint8_t** dev_ptr, size_t size);
void wrapper_cuda_free(void* dev_ptr);
void wrapper_cuda_memcpy(void* dev_ptr, const void* host_ptr, size_t size);

#ifdef __cplusplus
}
#endif

#endif  // GPU_RAYCAST_LAUNCH_H
