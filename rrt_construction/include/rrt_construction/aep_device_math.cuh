#ifndef RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_
#define RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_

// ============================================================================
//  aep_device_math.cuh
//
//  Device-side library for the AEP / marginal-gain raycasters. Holds every
//  __device__ routine plus the value types and tunables shared by the kernels,
//  so aep_evaluator.cu is left with only __global__ kernels and host launchers.
//
//  Coding rules (NASA/JPL "Power of 10", JPL D-60411, C++ Core Guidelines):
//    * One responsibility per function; pure helpers have no side effects.
//    * Parameters bundled into small structs to stay well under the 6-arg limit.
//    * Named constants instead of magic numbers; const-correct read-only views.
//    * Every loop has a fixed, statically-provable upper bound.
//
//  DETERMINISM CONTRACT: helpers reproduce the original inline arithmetic
//  verbatim -- same operand order, intrinsics, epsilons, and floor() truncation.
//  Bit-identical output is a requirement; do not "tidy" the expressions.
// ============================================================================

#include <cuda_runtime.h>
#include <math_constants.h>
#include <math.h>
#include <stdint.h>

// ----------------------------------------------------------------------------
//  Tunables and grid encoding (shared by every kernel and launcher).
// ----------------------------------------------------------------------------

// Occupancy grid cell states.
#define V_FREE     0
#define V_OCCUPIED 1
#define V_UNKNOWN  2

// Angular discretisation of the gain sphere, in degrees.
#define DTHETA_DEG 2
#define DPHI_DEG   2

// Derived angular bin counts.
#define THETA_BINS (360 / DTHETA_DEG)   // azimuth sectors (180)
#define PHI_BINS   (180 / DPHI_DEG)     // polar rows (90)
#define TOTAL_RAYS (THETA_BINS * PHI_BINS)

#define MAX_THREADS_PER_BLOCK 512

// Dynamic per-launch parameters; identical layout on host and device.
struct KernelParams {
    float voxel_size;
    float gain_range;     // r_max
    float fov_y_rad;      // horizontal FOV
    float fov_p_rad;      // vertical FOV
    float camera_pitch;   // pitch offset

    float dtheta;         // azimuth step (yaw)
    float dphi;           // polar step (pitch)

    float phi_start;
    float phi_end;
};

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
// Hard cap on DDA iterations -- a runaway backstop that never trips in practice.
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
struct ParentCameraConfig {
    int   p_width;
    int   p_height;
    float fx;
    float fy;
    float cx;
    float cy;
};

// World->camera rotation as its three rows (R0 = R[0..2], R1 = R[3..5], ...).
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

// 2D pixel-grid DDA traversal state (walks a projected ray across a depth image).
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

// Pinhole projection of a camera-frame point to pixel coordinates (p_cam.z != 0).
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
// Unweighted volume element for an unknown segment of length dr at radius r:
// 2 r^2 dr + dr^3 / 6. Callers apply the per-ray angular weighting.
__device__ inline float gain_volume_increment(float r, float dr) {
    float term1 = 2.0f * r * r * dr;
    float term2 = (dr * dr * dr) / kGainCubicDiv;
    return term1 + term2;
}

// ----------------------------------------------------------------------------
//  3D DDA (Amanatides & Woo).
// ----------------------------------------------------------------------------

// Initialise traversal from a fractional voxel start position `g` along `dir`.
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
// Woo DDA over (u_start,v_start)->(u_end,v_end), clamped to the pixel range.
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
// Slide an FOV-wide window over the per-sector gain histogram; return the best
// window-start index and write its total gain to *out_gain. Single-thread helper.
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

// ============================================================================
//  Application-level device views (bundle kernel arguments into small structs).
// ============================================================================

// Read-only view of the occupancy grid.
struct MapContext {
    const uint8_t* map;
    int3   dim;
    float3 origin;
};

// A single parent camera frame (single-parent marginal kernels v1/v2).
struct ParentFrame {
    float3                  pos;
    const float*            depth;   // p_width * p_height planar depths
    aep::RotationRows       R;       // world->camera rotation
    aep::ParentCameraConfig cam;
};

// The full ancestor chain of a candidate (multi-ancestor marginal kernel v3).
struct AncestorSet {
    const float3*           positions;   // [num]
    const float*            yaws;        // [num] (parity only; unused in math)
    const float*            depth;       // [num * p_width * p_height]
    const float3*           R_rows;      // [num * 3] (R0,R1,R2 per ancestor)
    int                     num;
    aep::ParentCameraConfig cam;         // shared geometry across ancestors
};

// Per-candidate output buffers.
struct GainResults {
    float* gain;        // [num_candidates]
    float* yaw;         // [num_candidates]
    float* depth_all;   // scratch: one planar depth per cast ray
    float* depth;       // final view depth buffer (p_width * p_height per candidate)
};

// ============================================================================
//  Parent-frustum projection: clip a candidate ray against a depth image and
//  emit the parametric intervals (metres) the parent already observed as free.
// ============================================================================

// Liang-Barsky clip of segment (u0,v0)->(u1,v1) to [w_min,w_max]x[h_min,h_max].
// Returns false if fully outside; updates the normalised entry/exit factors.
__device__ inline bool clip_line_2d(float u0, float v0, float u1, float v1,
                                    float w_min, float w_max, float h_min, float h_max,
                                    float* s0, float* s1) {
    float dx = u1 - u0;
    float dy = v1 - v0;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {u0 - w_min, w_max - u0, v0 - h_min, h_max - v0};

    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0f) {            // parallel to this border
            if (q[i] < 0.0f) return false;
        } else {
            float r = q[i] / p[i];
            if (p[i] < 0.0f) {         // entering
                if (r > *s1) return false;
                if (r > *s0) *s0 = r;
            } else {                   // exiting
                if (r < *s0) return false;
                if (r < *s1) *s1 = r;
            }
        }
    }
    return true;
}

// Single-interval skip distance (legacy v1). Returns (visible_start, hit, status)
// in metres; x = -1 means the ray never enters the parent frustum.
__device__ inline float3 compute_skip_distance(const aep::ParentCameraConfig& cam,
                                               float3 parent_pos, const aep::RotationRows& R,
                                               const float* __restrict__ parent_depth_buffer,
                                               float3 ray_start, float3 ray_dir, float max_dist) {
    const float z_near = 0.1f;
    const float z_far  = max_dist;

    // 1. Ray into the parent camera frame: O = R*(start - parent), D = R*dir.
    float3 diff = make_float3(ray_start.x - parent_pos.x,
                              ray_start.y - parent_pos.y,
                              ray_start.z - parent_pos.z);
    float3 O = aep::apply_rotation_rows(R, diff);
    float3 D = aep::apply_rotation_rows(R, ray_dir);

    // 2. Z-slab clip against the depth range.
    float t0 = 0.0f;
    float t1 = max_dist;
    if (fabsf(D.z) < 1e-3f) {
        if (O.z < z_near || O.z > z_far) return make_float3(-1.0f, -1.0f, 0.0f);
    } else {
        float inv_Dz = 1.0f / D.z;
        float t_near = (z_near - O.z) * inv_Dz;
        float t_far  = (z_far  - O.z) * inv_Dz;
        t0 = fmaxf(t0, fminf(t_near, t_far));
        t1 = fminf(t1, fmaxf(t_near, t_far));
    }
    if (t0 >= t1) return make_float3(-1.0f, -1.0f, 0.0f);

    float3 P_start = make_float3(O.x + t0*D.x, O.y + t0*D.y, O.z + t0*D.z);
    float3 P_end   = make_float3(O.x + t1*D.x, O.y + t1*D.y, O.z + t1*D.z);

    // 3. Project clipped endpoints to pixels.
    float inv_z0 = 1.0f / P_start.z;
    float inv_z1 = 1.0f / P_end.z;
    aep::PinholeIntrinsics intr = {cam.fx, cam.fy, cam.cx, cam.cy};
    float2 px0 = aep::project_pinhole(P_start, intr);
    float2 px1 = aep::project_pinhole(P_end, intr);
    float u0 = px0.x, v0 = px0.y;
    float u1 = px1.x, v1 = px1.y;

    // 4. Screen clip (Liang-Barsky).
    float s_min = 0.0f;
    float s_max = 1.0f;
    float eps = 1e-4f;
    if (!clip_line_2d(u0, v0, u1, v1,
                      eps, (float)cam.p_width - eps,
                      eps, (float)cam.p_height - eps,
                      &s_min, &s_max)) {
        return make_float3(-1.0f, -1.0f, 0.0f);
    }

    // 5. Exact frustum interval: interpolate 1/z, recover metres.
    float w_start = inv_z0 + s_min * (inv_z1 - inv_z0);
    float w_end   = inv_z0 + s_max * (inv_z1 - inv_z0);
    float t_visible_start;
    float t_visible_end;
    if (fabsf(D.z) > 1e-3f) {
        t_visible_start = ((1.0f / w_start) - O.z) / D.z;
        t_visible_end   = ((1.0f / w_end)   - O.z) / D.z;
    } else {
        t_visible_start = t0 + s_min * (t1 - t0);
        t_visible_end   = t0 + s_max * (t1 - t0);
    }

    // 6. Clip the pixel endpoints, then walk them with a Woo DDA.
    float u_start = u0 + s_min * (u1 - u0);
    float v_start = v0 + s_min * (v1 - v0);
    float u_end   = u0 + s_max * (u1 - u0);
    float v_end   = v0 + s_max * (v1 - v0);

    aep::Dda2 dd = aep::dda2_init(u_start, v_start, u_end, v_end, cam.p_width, cam.p_height);
    int x = dd.x, y = dd.y;
    int x_end = dd.x_end, y_end = dd.y_end;
    int stepX = dd.stepX, stepY = dd.stepY;
    float tDeltaX = dd.tDeltaX, tDeltaY = dd.tDeltaY;
    float tMaxX = dd.tMaxX, tMaxY = dd.tMaxY;

    float current_t = 0.0f;
    float w_curr = w_start;
    bool hit_any_limit = false;
    float status = 1.0f;

    // 7. Step pixels until the parent surface occludes the ray.
    while (current_t <= 1.0f) {
        if (x >= 0 && x < cam.p_width && y >= 0 && y < cam.p_height) {
            float t_next_boundary = (tMaxX < tMaxY) ? tMaxX : tMaxY;
            float t_exit = fminf(t_next_boundary, 1.0f);

            float w_entry = w_start + current_t * (w_end - w_start);
            float w_exit  = w_start + t_exit  * (w_end - w_start);
            float z_entry = 1.0f / w_entry;
            float z_exit  = 1.0f / w_exit;

            float parent_z = parent_depth_buffer[y * cam.p_width + x];
            if (parent_z < 0.0f) {
                return make_float3(-1.0f, -1.0f, 0.0f);   // root / uninitialised pixel
            }

            if (parent_z <= z_entry + 0.35f) {
                hit_any_limit = true;
                w_curr = w_entry;
                float px_u = (x + 0.5f - cam.cx) / cam.fx;
                float px_v = (y + 0.5f - cam.cy) / cam.fy;
                float cos_theta_pixel = rsqrtf(px_u*px_u + px_v*px_v + 1.0f);
                if (parent_z < max_dist * cos_theta_pixel) status = -1.0f;
                break;
            } else if (parent_z <= z_exit + 0.35f) {
                hit_any_limit = true;
                float dw = w_end - w_start;
                float t_exact = ((1.0f / parent_z) - w_start) / dw;
                current_t = fmaxf(current_t, fminf(t_exact, t_exit));
                w_curr = w_start + current_t * dw;
                float px_u = (x + 0.5f - cam.cx) / cam.fx;
                float px_v = (y + 0.5f - cam.cy) / cam.fy;
                float cos_theta_pixel = rsqrtf(px_u*px_u + px_v*px_v + 1.0f);
                if (parent_z < max_dist * cos_theta_pixel) status = -1.0f;
                break;
            }
        } else {
            hit_any_limit = true;
            break;
        }

        if (x == x_end && y == y_end) break;
        if (tMaxX < tMaxY) { x += stepX; current_t = tMaxX; tMaxX += tDeltaX; }
        else               { y += stepY; current_t = tMaxY; tMaxY += tDeltaY; }
    }

    if (!hit_any_limit) {
        w_curr = w_end;
        current_t = 1.0f;
    }

    // 8. Recover the hit distance in metres.
    float t_hit;
    if (fabsf(D.z) > 1e-3f) {
        t_hit = ((1.0f / w_curr) - O.z) / D.z;
    } else {
        t_hit = t_visible_start + current_t * (t_visible_end - t_visible_start);
    }
    return make_float3(t_visible_start, t_hit, status);
}

// Multi-ancestor skip distance: for each ancestor, project the candidate ray and
// merge every observed-free interval (metres) into the shared sorted set.
__device__ inline void compute_multi_segment_skip_distance(
    const aep::ParentCameraConfig& cam,
    const float3* parent_positions,                  // [num_ancestors]
    const float3* parent_R_rows,                     // [num_ancestors*3]
    const float* __restrict__ parent_depth_buffers,  // [num_ancestors * p_width * p_height]
    int num_ancestors,
    float3 ray_start, float3 ray_dir,
    const KernelParams& params,
    float2* out_intervals, int* out_count, int max_intervals,
    float* out_status) {

    *out_count = 0;
    *out_status = 1.0f;                 // default: free space

    const float z_near = 0.1f;
    const float z_far  = params.gain_range;

    for (int a = 0; a < num_ancestors; ++a) {
        float3 parent_pos = parent_positions[a];
        aep::RotationRows R = {parent_R_rows[a*3 + 0], parent_R_rows[a*3 + 1], parent_R_rows[a*3 + 2]};
        const float* parent_depth_buffer = parent_depth_buffers + (size_t)a * cam.p_width * cam.p_height;

        // 1. Ray into the parent camera frame.
        float3 diff = make_float3(ray_start.x - parent_pos.x,
                                  ray_start.y - parent_pos.y,
                                  ray_start.z - parent_pos.z);
        float3 O = aep::apply_rotation_rows(R, diff);
        float3 D = aep::apply_rotation_rows(R, ray_dir);

        // 2. Z-slab clip.
        float t0 = 0.0f;
        float t1 = params.gain_range;
        if (fabsf(D.z) < 1e-3f) {
            if (O.z < z_near || O.z > z_far) continue;
        } else {
            float inv_Dz = 1.0f / D.z;
            float t_near = (z_near - O.z) * inv_Dz;
            float t_far  = (z_far  - O.z) * inv_Dz;
            t0 = fmaxf(t0, fminf(t_near, t_far));
            t1 = fminf(t1, fmaxf(t_near, t_far));
        }
        if (t0 >= t1) continue;

        float3 P_start = make_float3(O.x + t0*D.x, O.y + t0*D.y, O.z + t0*D.z);
        float3 P_end   = make_float3(O.x + t1*D.x, O.y + t1*D.y, O.z + t1*D.z);

        // 3. Project clipped endpoints to pixels.
        float inv_z0 = 1.0f / P_start.z;
        float inv_z1 = 1.0f / P_end.z;
        aep::PinholeIntrinsics intr = {cam.fx, cam.fy, cam.cx, cam.cy};
        float2 px0 = aep::project_pinhole(P_start, intr);
        float2 px1 = aep::project_pinhole(P_end, intr);
        float u0 = px0.x, v0 = px0.y;
        float u1 = px1.x, v1 = px1.y;

        // 4. Screen clip (Liang-Barsky).
        float s_min = 0.0f;
        float s_max = 1.0f;
        float eps = 1e-4f;
        if (!clip_line_2d(u0, v0, u1, v1,
                          eps, (float)cam.p_width - eps,
                          eps, (float)cam.p_height - eps,
                          &s_min, &s_max)) {
            continue;
        }

        // 5. Exact frustum interval (metres).
        float w_start = inv_z0 + s_min * (inv_z1 - inv_z0);
        float w_end   = inv_z0 + s_max * (inv_z1 - inv_z0);
        float t_visible_start;
        float t_visible_end;
        if (fabsf(D.z) > 1e-3f) {
            t_visible_start = ((1.0f / w_start) - O.z) / D.z;
            t_visible_end   = ((1.0f / w_end)   - O.z) / D.z;
        } else {
            t_visible_start = t0 + s_min * (t1 - t0);
            t_visible_end   = t0 + s_max * (t1 - t0);
        }

        // 6. Clip the pixel endpoints, then walk them with a Woo DDA.
        float u_start = u0 + s_min * (u1 - u0);
        float v_start = v0 + s_min * (v1 - v0);
        float u_end   = u0 + s_max * (u1 - u0);
        float v_end   = v0 + s_max * (v1 - v0);

        aep::Dda2 dd = aep::dda2_init(u_start, v_start, u_end, v_end, cam.p_width, cam.p_height);
        int x = dd.x, y = dd.y;
        int x_end = dd.x_end, y_end = dd.y_end;
        int stepX = dd.stepX, stepY = dd.stepY;
        float tDeltaX = dd.tDeltaX, tDeltaY = dd.tDeltaY;
        float tMaxX = dd.tMaxX, tMaxY = dd.tMaxY;

        // 7. State machine: open/close a skip interval as the ray crosses the
        //    parent surface, emitting each observed-free span in metres.
        float current_t = 0.0f;
        bool is_building_skip_interval = false;
        float current_segment_start_t = 0.0f;
        float margin = 0.35f * params.voxel_size;
        bool is_first_step = true;

        while (current_t <= 1.0f) {
            bool in_known_space = false;
            float t_exact = current_t;
            float t_next_boundary = (tMaxX < tMaxY) ? tMaxX : tMaxY;
            float t_exit = fminf(t_next_boundary, 1.0f);

            if (x >= 0 && x < cam.p_width && y >= 0 && y < cam.p_height) {
                float w_exit = w_start + t_exit * (w_end - w_start);
                float z_exit = 1.0f / w_exit;
                float parent_z = parent_depth_buffer[y * cam.p_width + x];

                if (parent_z < 0.0f) {
                    in_known_space = false;
                } else {
                    in_known_space = (z_exit <= parent_z + margin);

                    // On a state flip, refine the sub-pixel crossing factor.
                    if (!is_first_step && (is_building_skip_interval != in_known_space)) {
                        float w_target = 1.0f / (parent_z + margin);
                        float dw = w_end - w_start;
                        if (fabsf(dw) > 1e-6f) {
                            t_exact = (w_target - w_start) / dw;
                            t_exact = fmaxf(current_t, fminf(t_exact, t_exit));
                        }
                        float px_u = (x + 0.5f - cam.cx) / cam.fx;
                        float px_v = (y + 0.5f - cam.cy) / cam.fy;
                        float cos_theta = rsqrtf(px_u*px_u + px_v*px_v + 1.0f);
                        if (parent_z < params.gain_range * cos_theta) *out_status = -1.0f;
                    }
                }
            } else {
                in_known_space = false;
            }

            if (is_first_step) {
                is_building_skip_interval = in_known_space;
                current_segment_start_t = 0.0f;
                is_first_step = false;
            }

            if (is_building_skip_interval && !in_known_space) {
                // Left known space -> close and emit the interval.
                if (*out_count < max_intervals) {
                    float t_meters_start;
                    float t_meters_end;
                    if (fabsf(D.z) > 1e-3f) {
                        float w_s = w_start + current_segment_start_t * (w_end - w_start);
                        float w_e = w_start + t_exact * (w_end - w_start);
                        t_meters_start = ((1.0f / w_s) - O.z) / D.z;
                        t_meters_end   = ((1.0f / w_e) - O.z) / D.z;
                    } else {
                        t_meters_start = t_visible_start + current_segment_start_t * (t_visible_end - t_visible_start);
                        t_meters_end   = t_visible_start + t_exact * (t_visible_end - t_visible_start);
                    }
                    if (t_meters_end > t_meters_start + 1e-4f) {
                        aep::insert_and_merge_interval(out_intervals, out_count, max_intervals,
                                                       t_meters_start, t_meters_end);
                    }
                }
                is_building_skip_interval = false;
            } else if (!is_building_skip_interval && in_known_space) {
                is_building_skip_interval = true;
                current_segment_start_t = t_exact;
            }

            if (x == x_end && y == y_end) break;
            if (tMaxX < tMaxY) { x += stepX; current_t = tMaxX; tMaxX += tDeltaX; }
            else               { y += stepY; current_t = tMaxY; tMaxY += tDeltaY; }
        }

        // 8. Close a still-open interval at the frustum exit.
        if (is_building_skip_interval && *out_count < max_intervals) {
            float t_meters_start;
            float t_meters_end;
            if (fabsf(D.z) > 1e-3f) {
                float w_s = w_start + current_segment_start_t * (w_end - w_start);
                float w_e = w_start + 1.0f * (w_end - w_start);
                t_meters_start = ((1.0f / w_s) - O.z) / D.z;
                t_meters_end   = ((1.0f / w_e) - O.z) / D.z;
            } else {
                t_meters_start = t_visible_start + current_segment_start_t * (t_visible_end - t_visible_start);
                t_meters_end   = t_visible_start + 1.0f * (t_visible_end - t_visible_start);
            }
            if (t_meters_end > t_meters_start + 1e-4f) {
                aep::insert_and_merge_interval(out_intervals, out_count, max_intervals,
                                               t_meters_start, t_meters_end);
            }
        }
    }
}

// Single-parent convenience overload: forwards with num_ancestors = 1.
__device__ inline void compute_multi_segment_skip_distance(
    const aep::ParentCameraConfig& cam,
    float3 parent_pos, const aep::RotationRows& R,
    const float* __restrict__ parent_depth_buffer,
    float3 ray_start, float3 ray_dir,
    const KernelParams& params,
    float2* out_intervals, int* out_count, int max_intervals,
    float* out_status) {
    float3 R_rows[3] = {R.r0, R.r1, R.r2};
    compute_multi_segment_skip_distance(cam, &parent_pos, R_rows, parent_depth_buffer, 1,
                                        ray_start, ray_dir, params,
                                        out_intervals, out_count, max_intervals, out_status);
}

// ============================================================================
//  Ray marching: integrate volumetric gain along one candidate ray.
// ============================================================================

// Plain gain march (AEP kernels): integrate UNKNOWN voxels until an OCCUPIED
// voxel or max range. Writes the first-hit planar depth to *out_depth.
__device__ inline float march_gain_basic(const MapContext& m, float3 cam_pos, float3 dir,
                                         float sin_phi, const KernelParams& p, float* out_depth) {
    float3 g = aep::world_to_voxel(cam_pos, m.origin, p.voxel_size);
    aep::Dda3 d = aep::dda3_init(g, dir);
    float ray_gain = 0.0f;
    float max_t = p.gain_range / p.voxel_size;
    *out_depth = p.gain_range;

    for (int s = 0; s < aep::kMaxDdaSteps && d.t < max_t; ++s) {
        if (aep::in_bounds(d.ix, d.iy, d.iz, m.dim)) {
            uint8_t val = m.map[aep::voxel_flat_index(d.ix, d.iy, d.iz, m.dim)];
            if (val == V_OCCUPIED) {
                *out_depth = d.t * p.voxel_size;
                break;
            } else if (val == V_UNKNOWN) {
                float t_exit = aep::dda3_t_exit(d);
                float dr = (t_exit - d.t) * p.voxel_size;
                float r  = d.t * p.voxel_size;
                ray_gain += aep::gain_volume_increment(r, dr) * p.dtheta * sin_phi * sinf(p.dphi * 0.5f);
            }
        }
        aep::dda3_step(d);
    }
    return ray_gain;
}

// Legacy single-interval marginal march (v1): jump the one observed-free span.
__device__ inline float march_marginal_gain_single(const MapContext& m, float3 cam_pos, float3 dir,
                                                   float sin_phi, float skip_start_vox, float skip_end_vox,
                                                   const KernelParams& params, float* out_final_depth) {
    float dir_x = dir.x, dir_y = dir.y, dir_z = dir.z;
    float t = 0.0f;
    float max_t = params.gain_range / params.voxel_size;
    float final_depth = params.gain_range;
    bool has_jumped = false;

    float gx = (cam_pos.x - m.origin.x) / params.voxel_size + (dir_x * t);
    float gy = (cam_pos.y - m.origin.y) / params.voxel_size + (dir_y * t);
    float gz = (cam_pos.z - m.origin.z) / params.voxel_size + (dir_z * t);

    int ix = floorf(gx), iy = floorf(gy), iz = floorf(gz);
    int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
    int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
    int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

    float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
    float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
    float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

    float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
    float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
    float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

    float ray_gain = 0.0f;

    while (t < max_t) {
        if (!has_jumped && skip_start_vox >= 0.0f && t >= skip_start_vox && t < skip_end_vox) {
            if ((unsigned int)ix < (unsigned int)m.dim.x &&
                (unsigned int)iy < (unsigned int)m.dim.y &&
                (unsigned int)iz < (unsigned int)m.dim.z) {
                if (m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix] == V_OCCUPIED) {
                    final_depth = t * params.voxel_size;
                    break;
                }
            }

            has_jumped = true;
            float next_t = skip_end_vox;
            t = next_t;
            gx = (cam_pos.x - m.origin.x) / params.voxel_size + (dir_x * next_t);
            gy = (cam_pos.y - m.origin.y) / params.voxel_size + (dir_y * next_t);
            gz = (cam_pos.z - m.origin.z) / params.voxel_size + (dir_z * next_t);
            ix = floorf(gx); iy = floorf(gy); iz = floorf(gz);

            tMaxX = ((stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX) + t;
            tMaxY = ((stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY) + t;
            tMaxZ = ((stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ) + t;

            if ((unsigned int)ix < (unsigned int)m.dim.x &&
                (unsigned int)iy < (unsigned int)m.dim.y &&
                (unsigned int)iz < (unsigned int)m.dim.z) {
                if (m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix] == V_OCCUPIED) {
                    final_depth = t * params.voxel_size;
                    break;
                }
            }

            if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
            else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
            else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
            continue;
        }

        if (ix >= 0 && ix < m.dim.x && iy >= 0 && iy < m.dim.y && iz >= 0 && iz < m.dim.z) {
            uint8_t val = m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix];
            if (val == V_OCCUPIED) {
                final_depth = t * params.voxel_size;
                break;
            } else if (val == V_UNKNOWN) {
                float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                float dr = (t_exit - t) * params.voxel_size;
                float r = t * params.voxel_size;
                ray_gain += aep::gain_volume_increment(r, dr) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
            }
        }

        if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
        else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
        else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
    }

    *out_final_depth = final_depth;
    return ray_gain;
}

// Legacy multi-segment march (v2): suppress gain inside any observed-free span
// (no jumping). `skip_vox` is the sorted interval set in voxel units.
__device__ inline float march_marginal_gain_suppress(const MapContext& m, float3 cam_pos, float3 dir,
                                                     float sin_phi, const float2* skip_vox, int skip_count,
                                                     const KernelParams& params, float* out_final_depth) {
    float dir_x = dir.x, dir_y = dir.y, dir_z = dir.z;
    int current_skip_idx = 0;
    float t = 0.0f;
    float max_t = params.gain_range / params.voxel_size;
    float final_depth = params.gain_range;

    float gx = (cam_pos.x - m.origin.x) / params.voxel_size + (dir_x * t);
    float gy = (cam_pos.y - m.origin.y) / params.voxel_size + (dir_y * t);
    float gz = (cam_pos.z - m.origin.z) / params.voxel_size + (dir_z * t);

    int ix = floor(gx), iy = floor(gy), iz = floor(gz);
    int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
    int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
    int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

    float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
    float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
    float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

    float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
    float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
    float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

    float ray_gain = 0.0f;

    while (t < max_t) {
        if (ix >= 0 && ix < m.dim.x && iy >= 0 && iy < m.dim.y && iz >= 0 && iz < m.dim.z) {
            uint8_t val = m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix];
            if (val == V_OCCUPIED) {
                final_depth = t * params.voxel_size;
                break;
            } else if (val == V_UNKNOWN) {
                bool parent_sees_free = false;
                while (current_skip_idx < skip_count && t >= skip_vox[current_skip_idx].y) {
                    current_skip_idx++;
                }
                if (current_skip_idx < skip_count &&
                    t >= skip_vox[current_skip_idx].x && t < skip_vox[current_skip_idx].y) {
                    parent_sees_free = true;
                }
                if (!parent_sees_free) {
                    float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                    float dr = (t_exit - t) * params.voxel_size;
                    float r = t * params.voxel_size;
                    ray_gain += aep::gain_volume_increment(r, dr) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }
        }

        if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
        else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
        else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
    }

    *out_final_depth = final_depth;
    return ray_gain;
}

// Canonical multi-ancestor march (v3): jump every observed-free span, clamp the
// gain integral to the sensor range and to the next skip interval's start.
__device__ inline float march_marginal_gain(const MapContext& m, float3 cam_pos, float3 dir,
                                            float sin_phi, const float2* skip_vox, int skip_count,
                                            const KernelParams& params, float* out_final_depth) {
    float dir_x = dir.x, dir_y = dir.y, dir_z = dir.z;
    int current_skip_idx = 0;
    float t = 0.0f;
    float max_t = params.gain_range / params.voxel_size;
    float final_depth = params.gain_range;

    float gx = (cam_pos.x - m.origin.x) / params.voxel_size + (dir_x * t);
    float gy = (cam_pos.y - m.origin.y) / params.voxel_size + (dir_y * t);
    float gz = (cam_pos.z - m.origin.z) / params.voxel_size + (dir_z * t);

    int ix = floorf(gx), iy = floorf(gy), iz = floorf(gz);
    int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
    int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
    int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

    float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
    float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
    float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

    float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
    float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
    float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

    float ray_gain = 0.0f;

    while (t < max_t) {
        // Drop skip intervals the ray has already passed.
        while (current_skip_idx < skip_count && t >= skip_vox[current_skip_idx].y) {
            current_skip_idx++;
        }

        // Inside an observed-free span: jump past it (unless a wall blocks first).
        if (current_skip_idx < skip_count) {
            float skip_start_vox = skip_vox[current_skip_idx].x;
            float skip_end_vox   = skip_vox[current_skip_idx].y;
            if (t >= skip_start_vox && t < skip_end_vox) {
                if ((unsigned int)ix < (unsigned int)m.dim.x &&
                    (unsigned int)iy < (unsigned int)m.dim.y &&
                    (unsigned int)iz < (unsigned int)m.dim.z) {
                    if (m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix] == V_OCCUPIED) {
                        final_depth = t * params.voxel_size;
                        break;
                    }
                }

                float next_t = skip_end_vox;
                current_skip_idx++;
                t = next_t;
                gx = (cam_pos.x - m.origin.x) / params.voxel_size + (dir_x * next_t);
                gy = (cam_pos.y - m.origin.y) / params.voxel_size + (dir_y * next_t);
                gz = (cam_pos.z - m.origin.z) / params.voxel_size + (dir_z * next_t);
                ix = floorf(gx); iy = floorf(gy); iz = floorf(gz);

                tMaxX = ((stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX) + t;
                tMaxY = ((stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY) + t;
                tMaxZ = ((stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ) + t;

                if ((unsigned int)ix < (unsigned int)m.dim.x &&
                    (unsigned int)iy < (unsigned int)m.dim.y &&
                    (unsigned int)iz < (unsigned int)m.dim.z) {
                    if (m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix] == V_OCCUPIED) {
                        final_depth = t * params.voxel_size;
                        break;
                    }
                }

                if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
                else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
                else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
                continue;
            }
        }

        // Standard voxel evaluation.
        if ((unsigned int)ix < (unsigned int)m.dim.x &&
            (unsigned int)iy < (unsigned int)m.dim.y &&
            (unsigned int)iz < (unsigned int)m.dim.z) {
            uint8_t val = m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix];
            if (val == V_OCCUPIED) {
                final_depth = t * params.voxel_size;
                break;
            } else if (val == V_UNKNOWN) {
                float t_exit = fminf(tMaxX, fminf(tMaxY, tMaxZ));
                t_exit = fminf(t_exit, max_t);                            // FIX 1: cap at sensor range
                if (current_skip_idx < skip_count && t_exit > skip_vox[current_skip_idx].x) {
                    t_exit = fminf(t_exit, skip_vox[current_skip_idx].x); // FIX 2: no bleed into skip span
                }
                float dt = t_exit - t;
                if (dt > 0.0f) {
                    float dr = dt * params.voxel_size;
                    float r = t * params.voxel_size;
                    ray_gain += aep::gain_volume_increment(r, dr) * params.dtheta * sin_phi * sinf(params.dphi * 0.5f);
                }
            }
        }

        if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
        else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
        else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
    }

    *out_final_depth = final_depth;
    return ray_gain;
}

// ============================================================================
//  Depth-buffer synthesis (shared by every marginal kernel).
// ============================================================================
// Block-strided fill: cast one ray per pixel at the chosen yaw/pitch and store
// the planar (camera-frame z) depth of the first occupied voxel. `depth_out`
// points at this candidate's slice of the output buffer.
__device__ inline void generate_depth_buffer(const MapContext& m, const aep::ParentCameraConfig& cam,
                                             float3 cam_pos, float yaw, const KernelParams& params,
                                             float* depth_out) {
    float pitch = params.camera_pitch;
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    float cos_p = cosf(pitch), sin_p = sinf(pitch);

    int buffer_rays = cam.p_width * cam.p_height;
    float max_t_vox = params.gain_range / params.voxel_size;

    for (int idx = threadIdx.x; idx < buffer_rays; idx += blockDim.x) {
        int u = idx % cam.p_width;
        int v = idx / cam.p_width;

        float x_cam = (u - cam.cx) / cam.fx;
        float y_cam = (v - cam.cy) / cam.fy;
        float z_cam = 1.0f;

        float dir_x = (z_cam * cos_p - y_cam * sin_p) * cos_y + x_cam * sin_y;
        float dir_y = (z_cam * cos_p - y_cam * sin_p) * sin_y - x_cam * cos_y;
        float dir_z = - z_cam * sin_p - y_cam * cos_p;

        float norm = sqrtf(dir_x*dir_x + dir_y*dir_y + dir_z*dir_z);
        float inv_norm = 1.0f / norm;
        dir_x *= inv_norm; dir_y *= inv_norm; dir_z *= inv_norm;

        float t = 0.0f;
        float final_depth = params.gain_range;

        float gx = (cam_pos.x - m.origin.x) / params.voxel_size;
        float gy = (cam_pos.y - m.origin.y) / params.voxel_size;
        float gz = (cam_pos.z - m.origin.z) / params.voxel_size;

        int ix = floorf(gx), iy = floorf(gy), iz = floorf(gz);
        int stepX = (dir_x > 0.0f) ? 1 : ((dir_x < 0.0f) ? -1 : 0);
        int stepY = (dir_y > 0.0f) ? 1 : ((dir_y < 0.0f) ? -1 : 0);
        int stepZ = (dir_z > 0.0f) ? 1 : ((dir_z < 0.0f) ? -1 : 0);

        float tDeltaX = (fabsf(dir_x) > 1e-9f) ? fabsf(1.0f / dir_x) : 1e30f;
        float tDeltaY = (fabsf(dir_y) > 1e-9f) ? fabsf(1.0f / dir_y) : 1e30f;
        float tDeltaZ = (fabsf(dir_z) > 1e-9f) ? fabsf(1.0f / dir_z) : 1e30f;

        float tMaxX = (stepX > 0) ? (ix + 1.0f - gx) * tDeltaX : (gx - ix) * tDeltaX;
        float tMaxY = (stepY > 0) ? (iy + 1.0f - gy) * tDeltaY : (gy - iy) * tDeltaY;
        float tMaxZ = (stepZ > 0) ? (iz + 1.0f - gz) * tDeltaZ : (gz - iz) * tDeltaZ;

        while (t < max_t_vox) {
            if (ix >= 0 && ix < m.dim.x && iy >= 0 && iy < m.dim.y && iz >= 0 && iz < m.dim.z) {
                if (m.map[iz * (m.dim.x * m.dim.y) + iy * m.dim.x + ix] == V_OCCUPIED) {
                    final_depth = t * params.voxel_size;
                    break;
                }
            }
            if (tMaxX < tMaxY && tMaxX < tMaxZ) { ix += stepX; t = tMaxX; tMaxX += tDeltaX; }
            else if (tMaxY < tMaxZ)             { iy += stepY; t = tMaxY; tMaxY += tDeltaY; }
            else                                { iz += stepZ; t = tMaxZ; tMaxZ += tDeltaZ; }
        }

        float cos_theta = rsqrtf(x_cam*x_cam + y_cam*y_cam + z_cam*z_cam);
        depth_out[idx] = final_depth * cos_theta;
    }
}

#endif  // RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_
