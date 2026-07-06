#ifndef RRT_CONSTRUCTION_GPU_RAYCAST_MATH_CUH_
#define RRT_CONSTRUCTION_GPU_RAYCAST_MATH_CUH_

// ============================================================================
//  gpu_raycast_math.cuh
//
//  Header-only device library for the information-gain and marginal-gain
//  raycasters used by the planners. Sharing
//  __device__ inline routines through a header is the idiomatic way to reuse
//  device code without relocatable device code (-rdc=true).
//
//  Layout (read top to bottom):
//    1. TUNABLES   -- compile-time configuration macros.
//    2. TYPES      -- every struct, grouped before any function.
//    3. PRIMITIVES -- namespace gpuray: pure, single-purpose math helpers.
//    4. ROUTINES   -- higher-level device functions built from the primitives.
//
//  Coding rules (NASA/JPL "Power of 10", JPL D-60411, C++ Core Guidelines):
//    * One responsibility per function; <= 5 parameters (arguments bundled
//      into the structs in section 2).
//    * Named constants, const-correct read-only views, statically-bounded loops.
//
//  DETERMINISM CONTRACT: routines reproduce the original inline arithmetic
//  verbatim -- same operand order, intrinsics, and epsilons. floor() on a float
//  equals floorf(), so the gpuray::Dda3 helpers match the kernels' raw DDA exactly.
// ============================================================================

#include <cuda_runtime.h>
#include <math_constants.h>
#include <math.h>
#include <stdint.h>

// ============================================================================
//  1. TUNABLES
// ============================================================================

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

// ============================================================================
//  2. TYPES (structs first -- functions never define types inline)
// ============================================================================

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

namespace gpuray {

// Full geometry of a parent depth image: pixel dimensions + pinhole intrinsics.
struct ParentCameraConfig {
    int   p_width, p_height;
    float fx, fy, cx, cy;
};

// World->camera rotation as its three rows (R0 = R[0..2], R1 = R[3..5], ...).
struct RotationRows {
    float3 r0, r1, r2;
};

// 3D voxel-grid DDA (Amanatides & Woo) traversal state.
struct Dda3 {
    int   ix, iy, iz;
    int   stepX, stepY, stepZ;
    float tDeltaX, tDeltaY, tDeltaZ;
    float tMaxX, tMaxY, tMaxZ;
    float t;
};

// 2D pixel-grid DDA traversal state.
struct Dda2 {
    int   x, y;
    int   x_end, y_end;
    int   stepX, stepY;
    float tDeltaX, tDeltaY;
    float tMaxX, tMaxY;
};

}  // namespace gpuray

// ----- Application-level views (bundle arguments to keep functions <= 5 args) -

// Read-only view of the occupancy grid.
struct MapContext {
    const uint8_t* map;
    int3   dim;
    float3 origin;
};

// A single parent camera frame (single-parent marginal kernels v1/v2, and one
// ancestor of the v3 chain).
struct ParentFrame {
    float3                  pos;
    const float*            depth;   // p_width * p_height planar depths
    gpuray::RotationRows       R;       // world->camera rotation
    gpuray::ParentCameraConfig cam;
};

// The full ancestor chain of a candidate (multi-ancestor marginal kernel v3).
struct AncestorSet {
    const float3*           positions;   // [num]
    const float*            yaws;        // [num] (parity only; unused in math)
    const float*            depth;       // [num * p_width * p_height]
    const float3*           R_rows;      // [num * 3] (R0,R1,R2 per ancestor)
    int                     num;
    gpuray::ParentCameraConfig cam;         // shared geometry across ancestors
};

// Per-candidate output buffers.
struct GainResults {
    float* gain;        // [num_candidates]
    float* yaw;         // [num_candidates]
    float* depth_all;   // scratch: one planar depth per cast ray
    float* depth;       // final view depth buffer (p_width * p_height per candidate)
};

// A candidate ray (world frame).
struct Ray {
    float3 origin;
    float3 dir;
};

// A ray plus the polar-angle sine used to weight its volumetric gain.
struct MarchRay {
    float3 origin;
    float3 dir;
    float  sin_phi;
};

// A camera pose for depth-buffer synthesis (pitch comes from KernelParams).
struct CameraPose {
    float3 pos;
    float  yaw;
};

// Read-only set of observed-free spans (voxel units) consumed by a marcher.
struct SkipSet {
    const float2* intervals;
    int           count;
};

// Output set of observed-free spans (metres) filled by the projection pass.
struct SkipBuffer {
    float2* intervals;
    int*    count;
    int     capacity;
    float*  status;    // -1 if any ancestor surface really occludes the ray
};

// An axis-aligned pixel rectangle for line clipping.
struct Rect2 {
    float min_x, max_x, min_y, max_y;
};

// A candidate ray projected into one parent depth image.
struct RayProjection {
    bool      valid;                          // false if the ray misses the frustum
    float3    O, D;                           // ray in the parent camera frame
    float     w_start, w_end;                 // inverse depth at the clipped endpoints
    float     t_visible_start, t_visible_end; // metres at clip entry / exit
    gpuray::Dda2 dda;                            // pixel walk over the clipped segment
};

// ============================================================================
//  3. PRIMITIVES (namespace gpuray: pure, single-purpose helpers)
// ============================================================================

namespace gpuray {

// --- Named numerical constants (replace scattered magic numbers) ---
// Direction component below which an axis is treated as parallel (no crossing).
__device__ __constant__ const float kDirEpsilon     = 1e-9f;
// "Infinite" parametric step for a parallel axis in a DDA.
__device__ __constant__ const float kTDeltaInfinity = 1e30f;
// Radial divisor in the volumetric gain integral (dr^3 / 6).
__device__ __constant__ const float kGainCubicDiv   = 6.0f;
// Hard cap on DDA iterations -- a runaway backstop that never trips in practice.
__device__ __constant__ const int   kMaxDdaSteps    = 8192;

// --- Geometry ---

// Unit ray direction for spherical angles (theta = azimuth, phi = polar).
__device__ inline float3 spherical_ray_dir(float theta, float phi) {
    float sin_phi = sinf(phi);
    return make_float3(cosf(theta) * sin_phi, sinf(theta) * sin_phi, cosf(phi));
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
__device__ inline float2 project_pinhole(float3 p_cam, const ParentCameraConfig& k) {
    float inv_z = 1.0f / p_cam.z;
    return make_float2(k.fx * p_cam.x * inv_z + k.cx,
                       k.fy * p_cam.y * inv_z + k.cy);
}

// --- Voxel grid indexing ---

__device__ inline bool in_bounds(int ix, int iy, int iz, int3 dim) {
    return ix >= 0 && ix < dim.x &&
           iy >= 0 && iy < dim.y &&
           iz >= 0 && iz < dim.z;
}

// Row-major flat index into the volumetric map (z outermost, x innermost).
__device__ inline int voxel_flat_index(int ix, int iy, int iz, int3 dim) {
    return iz * (dim.x * dim.y) + iy * dim.x + ix;
}

// --- Volumetric information-gain integral (radial term only) ---
// Unweighted volume element for an unknown segment of length dr at radius r:
// 2 r^2 dr + dr^3 / 6. Callers apply the per-ray angular weighting.
__device__ inline float gain_volume_increment(float r, float dr) {
    return 2.0f * r * r * dr + (dr * dr * dr) / kGainCubicDiv;
}

// --- 3D DDA (Amanatides & Woo) ---

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

// Re-seat the traversal at fractional position `g` and parametric distance `t`
// after a jump. Step directions and tDelta are unchanged.
__device__ inline void dda3_reseat(Dda3& d, float3 g, float t) {
    d.ix = floor(g.x);
    d.iy = floor(g.y);
    d.iz = floor(g.z);
    d.tMaxX = ((d.stepX > 0) ? (d.ix + 1.0f - g.x) * d.tDeltaX : (g.x - d.ix) * d.tDeltaX) + t;
    d.tMaxY = ((d.stepY > 0) ? (d.iy + 1.0f - g.y) * d.tDeltaY : (g.y - d.iy) * d.tDeltaY) + t;
    d.tMaxZ = ((d.stepZ > 0) ? (d.iz + 1.0f - g.z) * d.tDeltaZ : (g.z - d.iz) * d.tDeltaZ) + t;
    d.t = t;
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

// --- 2D DDA over a pixel grid (parent depth image) ---
// Woo DDA over segment start->end, clamped to the pixel range.
__device__ inline Dda2 dda2_init(float2 start, float2 end, int p_width, int p_height) {
    Dda2 d;
    d.x = floor(start.x);
    d.y = floor(start.y);
    d.x_end = floor(end.x);
    d.y_end = floor(end.y);

    d.x = max(0, min(d.x, p_width - 1));
    d.y = max(0, min(d.y, p_height - 1));
    d.x_end = max(0, min(d.x_end, p_width - 1));
    d.y_end = max(0, min(d.y_end, p_height - 1));

    d.stepX = (end.x > start.x) ? 1 : ((end.x < start.x) ? -1 : 0);
    d.stepY = (end.y > start.y) ? 1 : ((end.y < start.y) ? -1 : 0);

    float dx = end.x - start.x;
    float dy = end.y - start.y;
    d.tDeltaX = (dx != 0.0f) ? fabsf(1.0f / dx) : kTDeltaInfinity;
    d.tDeltaY = (dy != 0.0f) ? fabsf(1.0f / dy) : kTDeltaInfinity;

    d.tMaxX = (d.stepX > 0) ? (floor(start.x) + 1.0f - start.x) * d.tDeltaX
                            : (start.x - floor(start.x)) * d.tDeltaX;
    d.tMaxY = (d.stepY > 0) ? (floor(start.y) + 1.0f - start.y) * d.tDeltaY
                            : (start.y - floor(start.y)) * d.tDeltaY;
    return d;
}

// --- Yaw sliding-window optimisation ---

// Slide an FOV-wide window over the per-sector gain histogram; return the best
// window-start index and write its total gain to *out_gain.
__device__ inline int best_yaw_start_index(const float* s_yaw_gains, int theta_bins,
                                           int sectors_in_fov, float* out_gain) {
    float max_gain = 0.0f;
    int best_start_idx = 0;
    for (int i = 0; i < theta_bins; ++i) {
        float window_gain = 0.0f;
        for (int k = 0; k < sectors_in_fov; ++k) {
            window_gain += s_yaw_gains[(i + k) % theta_bins];
        }
        if (window_gain > max_gain) {
            max_gain = window_gain;
            best_start_idx = i;
        }
    }
    *out_gain = max_gain;
    return best_start_idx;
}

// FOV-centre yaw of a window starting at `best_start_idx`, normalised to (-pi, pi].
__device__ inline float yaw_window_center_angle(int best_start_idx, float dtheta, float fov_y_rad) {
    float center_angle = (-CUDART_PI_F + best_start_idx * dtheta) + (fov_y_rad * 0.5f);
    if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
    return center_angle;
}

// --- Skip-interval set operations (multi-ancestor occlusion) ---
// Insert [lo,hi] into a sorted, non-overlapping interval set, coalescing any
// overlapping/touching intervals. Bounded by `max_intervals`.
__device__ inline void insert_and_merge_interval(float2* intervals, int* count,
                                                 int max_intervals, float lo, float hi) {
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

}  // namespace gpuray

// ============================================================================
//  4. ROUTINES (built from the primitives above)
// ============================================================================

// --- Small shared helpers ---

// Map cell value at (ix,iy,iz); out-of-bounds reads as V_FREE so callers can
// treat "outside the grid" and "free" uniformly.
__device__ inline uint8_t voxel_value(const MapContext& m, int ix, int iy, int iz) {
    if (!gpuray::in_bounds(ix, iy, iz, m.dim)) return V_FREE;
    return m.map[gpuray::voxel_flat_index(ix, iy, iz, m.dim)];
}

// Angular-weighted information gain for an unknown span [t_enter, t_exit] (voxels).
__device__ inline float ray_segment_gain(float t_enter, float t_exit, float sin_phi,
                                         const KernelParams& p) {
    float dr = (t_exit - t_enter) * p.voxel_size;
    float r  = t_enter * p.voxel_size;
    return gpuray::gain_volume_increment(r, dr) * p.dtheta * sin_phi * sinf(p.dphi * 0.5f);
}

// Map a normalised segment factor [0,1] back to metres along the candidate ray.
__device__ inline float segment_factor_to_metres(const RayProjection& rp, float factor) {
    if (fabsf(rp.D.z) > 1e-3f) {
        float w = rp.w_start + factor * (rp.w_end - rp.w_start);
        return ((1.0f / w) - rp.O.z) / rp.D.z;
    }
    return rp.t_visible_start + factor * (rp.t_visible_end - rp.t_visible_start);
}

// True if the parent surface at pixel (x,y) is a real hit rather than the sensor
// max range (compares planar depth against the range scaled by the pixel slant).
__device__ inline bool parent_surface_is_real(const gpuray::ParentCameraConfig& cam,
                                              int x, int y, float parent_z, float range) {
    float px_u = (x + 0.5f - cam.cx) / cam.fx;
    float px_v = (y + 0.5f - cam.cy) / cam.fy;
    float cos_theta = rsqrtf(px_u * px_u + px_v * px_v + 1.0f);
    return parent_z < range * cos_theta;
}

// Build the ParentFrame for ancestor `i` of a chain.
__device__ inline ParentFrame ancestor_frame(const AncestorSet& a, int i) {
    ParentFrame p;
    p.pos   = a.positions[i];
    p.R     = {a.R_rows[i * 3 + 0], a.R_rows[i * 3 + 1], a.R_rows[i * 3 + 2]};
    p.depth = a.depth + (size_t)i * a.cam.p_width * a.cam.p_height;
    p.cam   = a.cam;
    return p;
}

// Liang-Barsky clip of segment a->b to `box`. Returns false if fully outside;
// otherwise updates the normalised entry/exit factors *s0,*s1.
__device__ inline bool clip_line_2d(float2 a, float2 b, Rect2 box, float* s0, float* s1) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {a.x - box.min_x, box.max_x - a.x, a.y - box.min_y, box.max_y - a.y};

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

// --- Parent-frustum projection ---

// Clip a candidate ray against one parent depth image and return the frustum
// geometry needed to walk it. rp.valid is false when the ray misses the frustum.
__device__ inline RayProjection project_ray_into_parent(const ParentFrame& parent,
                                                        Ray ray, float max_dist) {
    RayProjection rp;
    rp.valid = false;

    const float z_near = 0.1f;
    const float z_far  = max_dist;

    // 1. Ray into the parent camera frame: O = R*(start - parent), D = R*dir.
    float3 diff = make_float3(ray.origin.x - parent.pos.x,
                              ray.origin.y - parent.pos.y,
                              ray.origin.z - parent.pos.z);
    rp.O = gpuray::apply_rotation_rows(parent.R, diff);
    rp.D = gpuray::apply_rotation_rows(parent.R, ray.dir);

    // 2. Z-slab clip against the depth range.
    float t0 = 0.0f;
    float t1 = max_dist;
    if (fabsf(rp.D.z) < 1e-3f) {
        if (rp.O.z < z_near || rp.O.z > z_far) return rp;
    } else {
        float inv_Dz = 1.0f / rp.D.z;
        float t_near = (z_near - rp.O.z) * inv_Dz;
        float t_far  = (z_far  - rp.O.z) * inv_Dz;
        t0 = fmaxf(t0, fminf(t_near, t_far));
        t1 = fminf(t1, fmaxf(t_near, t_far));
    }
    if (t0 >= t1) return rp;

    float3 P_start = make_float3(rp.O.x + t0 * rp.D.x, rp.O.y + t0 * rp.D.y, rp.O.z + t0 * rp.D.z);
    float3 P_end   = make_float3(rp.O.x + t1 * rp.D.x, rp.O.y + t1 * rp.D.y, rp.O.z + t1 * rp.D.z);

    // 3. Project clipped endpoints to pixels.
    float inv_z0 = 1.0f / P_start.z;
    float inv_z1 = 1.0f / P_end.z;
    float2 px0 = gpuray::project_pinhole(P_start, parent.cam);
    float2 px1 = gpuray::project_pinhole(P_end, parent.cam);

    // 4. Screen clip (Liang-Barsky).
    float s_min = 0.0f;
    float s_max = 1.0f;
    float eps = 1e-4f;
    Rect2 box = {eps, (float)parent.cam.p_width - eps, eps, (float)parent.cam.p_height - eps};
    if (!clip_line_2d(px0, px1, box, &s_min, &s_max)) return rp;

    // 5. Exact frustum interval: interpolate 1/z, recover metres.
    rp.w_start = inv_z0 + s_min * (inv_z1 - inv_z0);
    rp.w_end   = inv_z0 + s_max * (inv_z1 - inv_z0);
    if (fabsf(rp.D.z) > 1e-3f) {
        rp.t_visible_start = ((1.0f / rp.w_start) - rp.O.z) / rp.D.z;
        rp.t_visible_end   = ((1.0f / rp.w_end)   - rp.O.z) / rp.D.z;
    } else {
        rp.t_visible_start = t0 + s_min * (t1 - t0);
        rp.t_visible_end   = t0 + s_max * (t1 - t0);
    }

    // 6. Clip the pixel endpoints, then set up the Woo DDA over them.
    float2 start = make_float2(px0.x + s_min * (px1.x - px0.x), px0.y + s_min * (px1.y - px0.y));
    float2 end   = make_float2(px0.x + s_max * (px1.x - px0.x), px0.y + s_max * (px1.y - px0.y));
    rp.dda = gpuray::dda2_init(start, end, parent.cam.p_width, parent.cam.p_height);
    rp.valid = true;
    return rp;
}

// Single-interval skip distance (legacy v1). Returns (visible_start, hit, status)
// in metres; x = -1 means the ray never enters the parent frustum.
__device__ inline float3 compute_skip_distance(const ParentFrame& parent, Ray ray, float max_dist) {
    RayProjection rp = project_ray_into_parent(parent, ray, max_dist);
    if (!rp.valid) return make_float3(-1.0f, -1.0f, 0.0f);

    gpuray::Dda2 d = rp.dda;
    float current_t = 0.0f;
    float w_curr = rp.w_start;
    bool hit_any_limit = false;
    float status = 1.0f;

    while (current_t <= 1.0f) {
        if (d.x >= 0 && d.x < parent.cam.p_width && d.y >= 0 && d.y < parent.cam.p_height) {
            float t_exit = fminf((d.tMaxX < d.tMaxY) ? d.tMaxX : d.tMaxY, 1.0f);
            float w_entry = rp.w_start + current_t * (rp.w_end - rp.w_start);
            float w_exit  = rp.w_start + t_exit   * (rp.w_end - rp.w_start);
            float z_entry = 1.0f / w_entry;
            float z_exit  = 1.0f / w_exit;

            float parent_z = parent.depth[d.y * parent.cam.p_width + d.x];
            if (parent_z < 0.0f) return make_float3(-1.0f, -1.0f, 0.0f);   // root / uninitialised

            if (parent_z <= z_entry + 0.35f) {
                hit_any_limit = true;
                w_curr = w_entry;
                if (parent_surface_is_real(parent.cam, d.x, d.y, parent_z, max_dist)) status = -1.0f;
                break;
            } else if (parent_z <= z_exit + 0.35f) {
                hit_any_limit = true;
                float dw = rp.w_end - rp.w_start;
                float t_exact = ((1.0f / parent_z) - rp.w_start) / dw;
                current_t = fmaxf(current_t, fminf(t_exact, t_exit));
                w_curr = rp.w_start + current_t * dw;
                if (parent_surface_is_real(parent.cam, d.x, d.y, parent_z, max_dist)) status = -1.0f;
                break;
            }
        } else {
            hit_any_limit = true;
            break;
        }

        if (d.x == d.x_end && d.y == d.y_end) break;
        if (d.tMaxX < d.tMaxY) { d.x += d.stepX; current_t = d.tMaxX; d.tMaxX += d.tDeltaX; }
        else                   { d.y += d.stepY; current_t = d.tMaxY; d.tMaxY += d.tDeltaY; }
    }

    if (!hit_any_limit) { w_curr = rp.w_end; current_t = 1.0f; }

    float t_hit;
    if (fabsf(rp.D.z) > 1e-3f) t_hit = ((1.0f / w_curr) - rp.O.z) / rp.D.z;
    else                       t_hit = rp.t_visible_start + current_t * (rp.t_visible_end - rp.t_visible_start);
    return make_float3(rp.t_visible_start, t_hit, status);
}

// Walk one parent's projected ray, emitting each observed-free span (metres) into
// the shared skip set and latching *status when a real surface occludes the ray.
__device__ inline void accumulate_skip_intervals(const ParentFrame& parent, const RayProjection& rp,
                                                 const KernelParams& params, SkipBuffer skips) {
    gpuray::Dda2 d = rp.dda;
    float margin = 0.35f * params.voxel_size;
    float current_t = 0.0f;
    float segment_start_t = 0.0f;
    bool is_building = false;
    bool is_first_step = true;

    while (current_t <= 1.0f) {
        bool in_known_space = false;
        float t_exact = current_t;
        float t_exit = fminf((d.tMaxX < d.tMaxY) ? d.tMaxX : d.tMaxY, 1.0f);

        if (d.x >= 0 && d.x < parent.cam.p_width && d.y >= 0 && d.y < parent.cam.p_height) {
            float z_exit = 1.0f / (rp.w_start + t_exit * (rp.w_end - rp.w_start));
            float parent_z = parent.depth[d.y * parent.cam.p_width + d.x];
            if (parent_z >= 0.0f) {
                in_known_space = (z_exit <= parent_z + margin);
                // On a state flip, refine the sub-pixel crossing factor.
                if (!is_first_step && (is_building != in_known_space)) {
                    float dw = rp.w_end - rp.w_start;
                    if (fabsf(dw) > 1e-6f) {
                        t_exact = (1.0f / (parent_z + margin) - rp.w_start) / dw;
                        t_exact = fmaxf(current_t, fminf(t_exact, t_exit));
                    }
                    if (parent_surface_is_real(parent.cam, d.x, d.y, parent_z, params.gain_range)) {
                        *skips.status = -1.0f;
                    }
                }
            }
        }

        if (is_first_step) {
            is_building = in_known_space;
            segment_start_t = 0.0f;
            is_first_step = false;
        }

        if (is_building && !in_known_space) {
            // Left known space -> close and emit the interval.
            if (*skips.count < skips.capacity) {
                float a = segment_factor_to_metres(rp, segment_start_t);
                float b = segment_factor_to_metres(rp, t_exact);
                if (b > a + 1e-4f) {
                    gpuray::insert_and_merge_interval(skips.intervals, skips.count, skips.capacity, a, b);
                }
            }
            is_building = false;
        } else if (!is_building && in_known_space) {
            is_building = true;
            segment_start_t = t_exact;
        }

        if (d.x == d.x_end && d.y == d.y_end) break;
        if (d.tMaxX < d.tMaxY) { d.x += d.stepX; current_t = d.tMaxX; d.tMaxX += d.tDeltaX; }
        else                   { d.y += d.stepY; current_t = d.tMaxY; d.tMaxY += d.tDeltaY; }
    }

    // Close a still-open interval at the frustum exit.
    if (is_building && *skips.count < skips.capacity) {
        float a = segment_factor_to_metres(rp, segment_start_t);
        float b = segment_factor_to_metres(rp, 1.0f);
        if (b > a + 1e-4f) {
            gpuray::insert_and_merge_interval(skips.intervals, skips.count, skips.capacity, a, b);
        }
    }
}

// Single-parent skip set (legacy v2): project one parent and accumulate its spans.
__device__ inline void compute_skip_intervals_single(const ParentFrame& parent, Ray ray,
                                                     const KernelParams& params, SkipBuffer skips) {
    *skips.count = 0;
    *skips.status = 1.0f;
    RayProjection rp = project_ray_into_parent(parent, ray, params.gain_range);
    if (rp.valid) accumulate_skip_intervals(parent, rp, params, skips);
}

// Multi-ancestor skip set (v3): merge the observed-free spans of every ancestor.
__device__ inline void compute_multi_segment_skip_distance(const AncestorSet& ancestors, Ray ray,
                                                           const KernelParams& params, SkipBuffer skips) {
    *skips.count = 0;
    *skips.status = 1.0f;
    for (int a = 0; a < ancestors.num; ++a) {
        ParentFrame parent = ancestor_frame(ancestors, a);
        RayProjection rp = project_ray_into_parent(parent, ray, params.gain_range);
        if (rp.valid) accumulate_skip_intervals(parent, rp, params, skips);
    }
}

// --- Ray marching: integrate volumetric gain along one candidate ray ---

// Occupancy-only march: distance (metres) to the first occupied voxel, or the
// sensor range if none is hit.
__device__ inline float march_first_hit(const MapContext& m, float3 origin, float3 dir,
                                        const KernelParams& p) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(origin, m.origin, p.voxel_size), dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        if (voxel_value(m, d.ix, d.iy, d.iz) == V_OCCUPIED) {
            final_depth = d.t * p.voxel_size;
            break;
        }
        gpuray::dda3_step(d);
    }
    return final_depth;
}

// Plain gain march (AEP kernels): integrate UNKNOWN voxels until an OCCUPIED
// voxel or max range. Writes the first-hit planar depth to *out_depth.
__device__ inline float march_gain_basic(const MapContext& m, const MarchRay& ray,
                                         const KernelParams& p, float* out_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float ray_gain = 0.0f;
    *out_depth = p.gain_range;
    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { *out_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN)  ray_gain += ray_segment_gain(d.t, gpuray::dda3_t_exit(d), ray.sin_phi, p);
        gpuray::dda3_step(d);
    }
    return ray_gain;
}

// Legacy single-interval marginal march (v1): jump the one observed-free span.
// `skip` packs (start,end) in voxel units; skip.x < 0 means no interval.
__device__ inline float march_marginal_gain_single(const MapContext& m, const MarchRay& ray,
                                                   float2 skip, const KernelParams& p,
                                                   float* out_final_depth) {
    float3 g0 = gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size);
    gpuray::Dda3 d = gpuray::dda3_init(g0, ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    bool has_jumped = false;
    float ray_gain = 0.0f;

    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        if (!has_jumped && skip.x >= 0.0f && d.t >= skip.x && d.t < skip.y) {
            if (voxel_value(m, d.ix, d.iy, d.iz) == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
            has_jumped = true;
            float3 gj = make_float3(g0.x + ray.dir.x * skip.y, g0.y + ray.dir.y * skip.y, g0.z + ray.dir.z * skip.y);
            gpuray::dda3_reseat(d, gj, skip.y);
            if (voxel_value(m, d.ix, d.iy, d.iz) == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
            gpuray::dda3_step(d);
            continue;
        }

        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN)  ray_gain += ray_segment_gain(d.t, gpuray::dda3_t_exit(d), ray.sin_phi, p);
        gpuray::dda3_step(d);
    }
    *out_final_depth = final_depth;
    return ray_gain;
}

// Legacy multi-segment march (v2): suppress gain inside any observed-free span
// (no jumping).
__device__ inline float march_marginal_gain_suppress(const MapContext& m, const MarchRay& ray,
                                                     SkipSet skips, const KernelParams& p,
                                                     float* out_final_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    int current_skip_idx = 0;
    float ray_gain = 0.0f;

    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN) {
            while (current_skip_idx < skips.count && d.t >= skips.intervals[current_skip_idx].y) current_skip_idx++;
            bool parent_sees_free = (current_skip_idx < skips.count &&
                                     d.t >= skips.intervals[current_skip_idx].x &&
                                     d.t <  skips.intervals[current_skip_idx].y);
            if (!parent_sees_free) ray_gain += ray_segment_gain(d.t, gpuray::dda3_t_exit(d), ray.sin_phi, p);
        }
        gpuray::dda3_step(d);
    }
    *out_final_depth = final_depth;
    return ray_gain;
}

// Canonical multi-ancestor march (v3): jump every observed-free span, clamp the
// gain integral to the sensor range and to the next skip interval's start.
__device__ inline float march_marginal_gain(const MapContext& m, const MarchRay& ray,
                                            SkipSet skips, const KernelParams& p,
                                            float* out_final_depth) {
    float3 g0 = gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size);
    gpuray::Dda3 d = gpuray::dda3_init(g0, ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    int current_skip_idx = 0;
    float ray_gain = 0.0f;

    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        // Drop skip intervals the ray has already passed.
        while (current_skip_idx < skips.count && d.t >= skips.intervals[current_skip_idx].y) current_skip_idx++;

        // Inside an observed-free span: jump past it (unless a wall blocks first).
        if (current_skip_idx < skips.count) {
            float skip_start = skips.intervals[current_skip_idx].x;
            float skip_end   = skips.intervals[current_skip_idx].y;
            if (d.t >= skip_start && d.t < skip_end) {
                if (voxel_value(m, d.ix, d.iy, d.iz) == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
                current_skip_idx++;
                float3 gj = make_float3(g0.x + ray.dir.x * skip_end, g0.y + ray.dir.y * skip_end, g0.z + ray.dir.z * skip_end);
                gpuray::dda3_reseat(d, gj, skip_end);
                if (voxel_value(m, d.ix, d.iy, d.iz) == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
                gpuray::dda3_step(d);
                continue;
            }
        }

        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN) {
            float t_exit = fminf(gpuray::dda3_t_exit(d), max_t);                       // FIX 1: cap at range
            if (current_skip_idx < skips.count && t_exit > skips.intervals[current_skip_idx].x) {
                t_exit = fminf(t_exit, skips.intervals[current_skip_idx].x);        // FIX 2: no bleed into skip span
            }
            if (t_exit > d.t) ray_gain += ray_segment_gain(d.t, t_exit, ray.sin_phi, p);
        }
        gpuray::dda3_step(d);
    }
    *out_final_depth = final_depth;
    return ray_gain;
}

// Multi-ancestor marginal march, traversal variant (v4): same result as v3 in the
// common case, but it NEVER reseats the DDA. It walks every voxel and simply omits
// the gain of any segment that lies inside an observed-free span (traversed, not
// counted). It keeps v3's two clamps (cap at range, no bleed into the next span),
// so the counted t-range matches v3 wherever the ray is unobstructed. Because it
// steps through skipped voxels instead of jumping them, an OCCUPIED voxel found
// inside a span still terminates the ray -- exactly how v2 relates to v1. This is
// the safer path: reseating past a span assumes it is empty, traversal re-checks it.
__device__ inline float march_marginal_gain_traverse(const MapContext& m, const MarchRay& ray,
                                                     SkipSet skips, const KernelParams& p,
                                                     float* out_final_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    int current_skip_idx = 0;
    float ray_gain = 0.0f;

    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        // Drop skip intervals the ray has already passed.
        while (current_skip_idx < skips.count && d.t >= skips.intervals[current_skip_idx].y) current_skip_idx++;

        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { final_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN) {
            // Inside an observed-free span: traverse the voxel but count no gain.
            bool inside = (current_skip_idx < skips.count &&
                           d.t >= skips.intervals[current_skip_idx].x &&
                           d.t <  skips.intervals[current_skip_idx].y);
            if (!inside) {
                float t_exit = fminf(gpuray::dda3_t_exit(d), max_t);                       // FIX 1: cap at range
                if (current_skip_idx < skips.count && t_exit > skips.intervals[current_skip_idx].x) {
                    t_exit = fminf(t_exit, skips.intervals[current_skip_idx].x);        // FIX 2: no bleed into skip span
                }
                if (t_exit > d.t) ray_gain += ray_segment_gain(d.t, t_exit, ray.sin_phi, p);
            }
        }
        gpuray::dda3_step(d);
    }
    *out_final_depth = final_depth;
    return ray_gain;
}

// --- Depth-buffer synthesis (shared by every marginal kernel) ---
// Block-strided fill: cast one ray per pixel at the chosen pose and store the
// planar (camera-frame z) first-hit depth. `depth_out` is this candidate's slice.
__device__ inline void generate_depth_buffer(const MapContext& m, const gpuray::ParentCameraConfig& cam,
                                             CameraPose pose, const KernelParams& p, float* depth_out) {
    float cos_y = cosf(pose.yaw),         sin_y = sinf(pose.yaw);
    float cos_p = cosf(p.camera_pitch),   sin_p = sinf(p.camera_pitch);
    int buffer_rays = cam.p_width * cam.p_height;

    for (int idx = threadIdx.x; idx < buffer_rays; idx += blockDim.x) {
        int u = idx % cam.p_width;
        int v = idx / cam.p_width;

        float x_cam = (u - cam.cx) / cam.fx;
        float y_cam = (v - cam.cy) / cam.fy;
        float z_cam = 1.0f;

        float dir_x = (z_cam * cos_p - y_cam * sin_p) * cos_y + x_cam * sin_y;
        float dir_y = (z_cam * cos_p - y_cam * sin_p) * sin_y - x_cam * cos_y;
        float dir_z = - z_cam * sin_p - y_cam * cos_p;
        float inv_norm = 1.0f / sqrtf(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
        float3 dir = make_float3(dir_x * inv_norm, dir_y * inv_norm, dir_z * inv_norm);

        float final_depth = march_first_hit(m, pose.pos, dir, p);
        float cos_theta = rsqrtf(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam);
        depth_out[idx] = final_depth * cos_theta;
    }
}

#endif  // RRT_CONSTRUCTION_AEP_DEVICE_MATH_CUH_
