#ifndef RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_
#define RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_

// ============================================================================
//  aep_device_math.cuh
//
//  Isolated, pure __device__ math primitives for the AEP / marginal-gain
//  raycasters. Extracted from aep_evaluator.cu to remove duplication and to
//  follow rigorous, deterministic coding paradigms (NASA/JPL "Power of 10",
//  JPL D-60411 LOC-1..4, C++ Core Guidelines F.1/F.2/F.8):
//
//    * One logical operation per function, no side effects (pure helpers),
//      so each can be unit-reasoned and reused across every kernel.
//    * No dynamic allocation; flat index access only (single-level pointers).
//    * Named constants instead of magic numbers (ES.45).
//    * const-correctness; read-only buffers passed as const __restrict__.
//    * Fixed, statically-bounded loops in callers (Power of 10 rule 2);
//      the DDA helpers expose explicit init/step so callers can cap trip count.
//    * Debug-only assertions (device assert() is a hard, context-killing abort
//      and costs occupancy, so it is compiled out under NDEBUG).
//
//  DETERMINISM CONTRACT: every helper reproduces the original inline arithmetic
//  verbatim -- same operand order, same intrinsics, same epsilons, and the same
//  floor()-on-double truncation the original code relied on. Do not "tidy" the
//  expressions: bit-identical output is a requirement, not a nicety.
// ============================================================================

#include <cuda_runtime.h>
#include <math_constants.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>

namespace aep {

// ----------------------------------------------------------------------------
//  Named numerical constants (replace scattered magic numbers).
// ----------------------------------------------------------------------------
// Direction component below which an axis is treated as parallel (no crossing).
__device__ __constant__ const float kDirEpsilon      = 1e-9f;
// "Infinite" parametric step for a parallel axis in a DDA.
__device__ __constant__ const float kTDeltaInfinity  = 1e30f;
// Radial divisor in the volumetric gain integral (dr^3 / 6).
__device__ __constant__ const float kGainCubicDiv    = 6.0f;
// Generous hard cap on DDA iterations -- a runaway backstop only. A ray can
// cross at most ~3 voxel boundaries per voxel of range, so this never trips in
// practice; it exists purely to give every loop a provable upper bound.
__device__ __constant__ const int   kMaxDdaSteps     = 8192;

// ----------------------------------------------------------------------------
//  Small value types (keep parameter counts <= 6, JPL Rule 25).
// ----------------------------------------------------------------------------

// Pinhole camera intrinsics for a parent depth buffer.
struct PinholeIntrinsics {
    float fx;
    float fy;
    float cx;
    float cy;
};

// Full geometry of a parent depth image: pixel dimensions + pinhole intrinsics.
// Bundled so helpers take one config argument instead of six loose floats.
struct ParentCameraConfig {
    int   p_width;
    int   p_height;
    float fx;
    float fy;
    float cx;
    float cy;
};

// World->camera rotation stored as its three rows (matches the host packing
// R0 = R[0..2], R1 = R[3..5], R2 = R[6..8]).
struct RotationRows {
    float3 r0;
    float3 r1;
    float3 r2;
};

// 3D voxel-grid DDA (Amanatides & Woo) traversal state.
struct Dda3 {
    int   ix, iy, iz;
    int   stepX, stepY, stepZ;
    float tDeltaX, tDeltaY, tDeltaZ;
    float tMaxX, tMaxY, tMaxZ;
    float t;
};

// 2D pixel-grid DDA traversal state (used to walk a projected ray across a
// parent depth image).
struct Dda2 {
    int   x, y;
    int   x_end, y_end;
    int   stepX, stepY;
    float tDeltaX, tDeltaY;
    float tMaxX, tMaxY;
};

// ----------------------------------------------------------------------------
//  Geometry primitives.
// ----------------------------------------------------------------------------

// Unit ray direction for spherical angles (theta = azimuth, phi = polar).
// Matches: dir = (cos(theta) sin(phi), sin(theta) sin(phi), cos(phi)).
__device__ inline float3 spherical_ray_dir(float theta, float phi) {
    float sin_phi = sinf(phi);
    float dir_x = cosf(theta) * sin_phi;
    float dir_y = sinf(theta) * sin_phi;
    float dir_z = cosf(phi);
    return make_float3(dir_x, dir_y, dir_z);
}

// Convert a world-space point to fractional voxel coordinates.
__device__ inline float3 world_to_voxel(float3 p, float3 origin, float voxel_size) {
    return make_float3((p.x - origin.x) / voxel_size,
                       (p.y - origin.y) / voxel_size,
                       (p.z - origin.z) / voxel_size);
}

// Apply a row-major rotation (R * v) using the three stored rows.
__device__ inline float3 apply_rotation_rows(const RotationRows& R, float3 v) {
    return make_float3(R.r0.x * v.x + R.r0.y * v.y + R.r0.z * v.z,
                       R.r1.x * v.x + R.r1.y * v.y + R.r1.z * v.z,
                       R.r2.x * v.x + R.r2.y * v.y + R.r2.z * v.z);
}

// Pinhole projection of a camera-frame point to pixel coordinates.
// Caller must ensure p_cam.z != 0 (z-slab clipping guarantees this upstream).
__device__ inline float2 project_pinhole(float3 p_cam, const PinholeIntrinsics& k) {
    float inv_z = 1.0f / p_cam.z;
    return make_float2(k.fx * p_cam.x * inv_z + k.cx,
                       k.fy * p_cam.y * inv_z + k.cy);
}

// ----------------------------------------------------------------------------
//  Voxel grid indexing.
// ----------------------------------------------------------------------------

__device__ inline bool in_bounds(int ix, int iy, int iz, int3 dim) {
    return ix >= 0 && ix < dim.x &&
           iy >= 0 && iy < dim.y &&
           iz >= 0 && iz < dim.z;
}

// Row-major flat index into the volumetric map (z outermost, x innermost).
__device__ inline int voxel_flat_index(int ix, int iy, int iz, int3 dim) {
    return iz * (dim.x * dim.y) + iy * dim.x + ix;
}

// ----------------------------------------------------------------------------
//  Volumetric information-gain integral (radial term only).
// ----------------------------------------------------------------------------
// Returns the unweighted volume element for an unknown segment of length dr at
// radius r: 2 r^2 dr + dr^3 / 6. Callers apply the angular weighting
// (dtheta * sin(phi) * sin(dphi/2)) that varies per ray.
__device__ inline float gain_volume_increment(float r, float dr) {
    float term1 = 2.0f * r * r * dr;
    float term2 = (dr * dr * dr) / kGainCubicDiv;
    return term1 + term2;
}

// ----------------------------------------------------------------------------
//  3D DDA (Amanatides & Woo).
// ----------------------------------------------------------------------------

// Initialise traversal from a fractional voxel start position `g` along `dir`.
// NOTE: uses floor() (double) exactly as the original kernels did.
__device__ inline Dda3 dda3_init(float3 g, float3 dir) {
    Dda3 d;
    d.ix = floor(g.x);
    d.iy = floor(g.y);
    d.iz = floor(g.z);

    d.stepX = (dir.x > 0.0f) ? 1 : ((dir.x < 0.0f) ? -1 : 0);
    d.stepY = (dir.y > 0.0f) ? 1 : ((dir.y < 0.0f) ? -1 : 0);
    d.stepZ = (dir.z > 0.0f) ? 1 : ((dir.z < 0.0f) ? -1 : 0);

    d.tDeltaX = (fabsf(dir.x) > kDirEpsilon) ? fabsf(1.0f / dir.x) : kTDeltaInfinity;
    d.tDeltaY = (fabsf(dir.y) > kDirEpsilon) ? fabsf(1.0f / dir.y) : kTDeltaInfinity;
    d.tDeltaZ = (fabsf(dir.z) > kDirEpsilon) ? fabsf(1.0f / dir.z) : kTDeltaInfinity;

    d.tMaxX = (d.stepX > 0) ? (d.ix + 1.0f - g.x) * d.tDeltaX : (g.x - d.ix) * d.tDeltaX;
    d.tMaxY = (d.stepY > 0) ? (d.iy + 1.0f - g.y) * d.tDeltaY : (g.y - d.iy) * d.tDeltaY;
    d.tMaxZ = (d.stepZ > 0) ? (d.iz + 1.0f - g.z) * d.tDeltaZ : (g.z - d.iz) * d.tDeltaZ;

    d.t = 0.0f;
    return d;
}

// Parametric distance at which the current voxel cell is exited.
__device__ inline float dda3_t_exit(const Dda3& d) {
    return fminf(d.tMaxX, fminf(d.tMaxY, d.tMaxZ));
}

// Advance one voxel along the dominant axis, updating the parametric position.
__device__ inline void dda3_step(Dda3& d) {
    if (d.tMaxX < d.tMaxY && d.tMaxX < d.tMaxZ) {
        d.ix += d.stepX; d.t = d.tMaxX; d.tMaxX += d.tDeltaX;
    } else if (d.tMaxY < d.tMaxZ) {
        d.iy += d.stepY; d.t = d.tMaxY; d.tMaxY += d.tDeltaY;
    } else {
        d.iz += d.stepZ; d.t = d.tMaxZ; d.tMaxZ += d.tDeltaZ;
    }
}

// ----------------------------------------------------------------------------
//  2D DDA over a pixel grid (parent depth image).
// ----------------------------------------------------------------------------
// Initialise a Woo DDA walking the segment (u_start,v_start)->(u_end,v_end),
// clamped to the [0, p_width-1] x [0, p_height-1] pixel range.
__device__ inline Dda2 dda2_init(float u_start, float v_start,
                                 float u_end, float v_end,
                                 int p_width, int p_height) {
    Dda2 d;
    d.x = floor(u_start);
    d.y = floor(v_start);
    d.x_end = floor(u_end);
    d.y_end = floor(v_end);

    d.x = max(0, min(d.x, p_width - 1));
    d.y = max(0, min(d.y, p_height - 1));
    d.x_end = max(0, min(d.x_end, p_width - 1));
    d.y_end = max(0, min(d.y_end, p_height - 1));

    d.stepX = (u_end > u_start) ? 1 : ((u_end < u_start) ? -1 : 0);
    d.stepY = (v_end > v_start) ? 1 : ((v_end < v_start) ? -1 : 0);

    float dx = u_end - u_start;
    float dy = v_end - v_start;
    d.tDeltaX = (dx != 0.0f) ? fabsf(1.0f / dx) : kTDeltaInfinity;
    d.tDeltaY = (dy != 0.0f) ? fabsf(1.0f / dy) : kTDeltaInfinity;

    d.tMaxX = (d.stepX > 0) ? (floor(u_start) + 1.0f - u_start) * d.tDeltaX
                            : (u_start - floor(u_start)) * d.tDeltaX;
    d.tMaxY = (d.stepY > 0) ? (floor(v_start) + 1.0f - v_start) * d.tDeltaY
                            : (v_start - floor(v_start)) * d.tDeltaY;
    return d;
}

// ----------------------------------------------------------------------------
//  Yaw sliding-window optimisation.
// ----------------------------------------------------------------------------
// Slide an FOV-wide window over the per-sector gain histogram; return the index
// of the best window start and write its total gain to *out_gain.
// `s_yaw_gains` has `theta_bins` entries. Single-thread helper.
__device__ inline int best_yaw_start_index(const float* s_yaw_gains,
                                           int theta_bins,
                                           int sectors_in_fov,
                                           float* out_gain) {
    float max_gain = 0.0f;
    int best_start_idx = 0;

    for (int i = 0; i < theta_bins; ++i) {
        float current_window_gain = 0.0f;
        for (int k = 0; k < sectors_in_fov; ++k) {
            int idx = (i + k) % theta_bins;
            current_window_gain += s_yaw_gains[idx];
        }
        if (current_window_gain > max_gain) {
            max_gain = current_window_gain;
            best_start_idx = i;
        }
    }

    *out_gain = max_gain;
    return best_start_idx;
}

// FOV-centre yaw of a window starting at `best_start_idx`, normalised to (-pi, pi].
__device__ inline float yaw_window_center_angle(int best_start_idx,
                                                float dtheta,
                                                float fov_y_rad) {
    float start_angle = -CUDART_PI_F + (best_start_idx * dtheta);
    float center_angle = start_angle + (fov_y_rad * 0.5f);
    if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
    return center_angle;
}

// ----------------------------------------------------------------------------
//  Skip-interval set operations (multi-ancestor occlusion).
// ----------------------------------------------------------------------------
// Insert [lo,hi] into a sorted, non-overlapping interval set, coalescing any
// overlapping/touching intervals. Bounded by `max_intervals`.
__device__ inline void insert_and_merge_interval(float2* intervals, int* count,
                                                 int max_intervals,
                                                 float lo, float hi) {
    if (hi <= lo) return;
    int i = 0;
    while (i < *count && intervals[i].y < lo) i++;        // strictly-before, keep
    int j = i;
    while (j < *count && intervals[j].x <= hi) {          // overlaps/touches -> absorb
        lo = fminf(lo, intervals[j].x);
        hi = fmaxf(hi, intervals[j].y);
        j++;
    }
    int tail = *count - j;
    if (i + 1 + tail > max_intervals) {                   // clamp: drop overflow tail
        tail = max_intervals - (i + 1);
        if (tail < 0) tail = 0;
    }
    for (int k = tail - 1; k >= 0; --k) intervals[i + 1 + k] = intervals[j + k];
    intervals[i] = make_float2(lo, hi);
    *count = i + 1 + tail;
}

}  // namespace aep

#endif  // RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_
