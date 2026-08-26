#ifndef RRT_CONSTRUCTION_GPU_RAYCAST_MATH_CUH_
#define RRT_CONSTRUCTION_GPU_RAYCAST_MATH_CUH_

// gpu_raycast_math.cuh -- header-only device raycast library. DETERMINISM: reproduces the original inline arithmetic (operand order, intrinsics, epsilons) verbatim.

#include <cuda_runtime.h>
#include <math_constants.h>
#include <math.h>
#include <stdint.h>


/* TUNABLES */

// Occupancy grid cell states.
#define V_FREE     0
#define V_OCCUPIED 1
#define V_UNKNOWN  2

// Azimuth bins are set at runtime from voxel_size/r_max (see set_angular_resolution); this only caps
// the shared histogram for the finest voxel (~0.05 m).
#define THETA_BINS_MAX 640

#define MAX_THREADS_PER_BLOCK 512


/* TYPES (structs first; functions never define types inline) */

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

    int theta_bins;       // azimuth bins over the full circle = round(2*pi / dtheta)
    int rows_in_fov;      // vertical (phi) sample rows  = angular_bins(fov_p, dphi)
    int sectors_in_fov;   // best-yaw window width (theta) = angular_bins(fov_y, dtheta)
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

// A single parent camera frame (one ancestor of a chain; the single-node kernel reconstructs one of these per ancestor).
struct ParentFrame {
    float3                  pos;
    const float*            depth;   // p_width * p_height planar depths
    gpuray::RotationRows       R;       // world->camera rotation
    gpuray::ParentCameraConfig cam;
};

// Full ancestor chain of a candidate (count=1 = single-parent, N = multi-ancestor); depth_idx (opt) pools depth buffers, else contiguous.
struct AncestorSet {
    const float3*           positions;   // [num]
    const float*            yaws;        // [num] (parity only; unused in math)
    const float*            depth;       // [num*per] (contiguous) or pool base (pooled)
    const float3*           R_rows;      // [num * 3] (R0,R1,R2 per ancestor)
    int                     num;
    gpuray::ParentCameraConfig cam;         // shared geometry across ancestors
    const int*              depth_idx;   // null -> contiguous; else pool index per ancestor
};

// Per-candidate output buffers.
struct GainResults {
    float* gain;        // [num_candidates]
    float* yaw;         // [num_candidates]
    float* depth_all;   // scratch: one planar depth per cast ray
    float* depth;       // final view depth buffer (p_width * p_height per candidate)
    const float* fixed_yaw;  // [num_candidates] fixed yaw per candidate, or null -> optimize yaw
};

// Device CSR view of a wavefront's ancestor chains; per = p_width*p_height; sliced per block by ancestors_for().
struct AncestorBatchDev {
    const int*    offsets;   // [num_candidates+1]
    const float3* pos;       // [total]
    const float*  yaw;       // [total]
    const float*  depth;     // [total*per] (contiguous) or [num_nodes*per] pool (pooled)
    const float3* R;         // [total*3]
    int           per;
    gpuray::ParentCameraConfig cam;
    const int*    depth_idx; // null -> contiguous depth; else pool index per ancestor slot
};

// Host-side bookkeeping for a batched launch: device allocations + launch dims (used only by the extern "C" launchers).
struct BatchDeviceMem {
    float3* d_cand;
    int*    d_off;
    float3* d_pos;
    float*  d_yaw;
    float3* d_R;
    float*  d_depth;
    int*    d_depth_idx;   // pooled depth index (null when contiguous)
    float*  d_gain;
    float*  d_yaw_out;
    float*  d_depth_buf;   // bounded output scratch (depth_slots * per)
    float*  d_fixed_yaw;   // per-candidate fixed yaw (null when optimizing yaw)
    int     rays;          // rays per candidate
    int     nc;
    int     depth_slots;   // number of output depth-buffer slots (bounds memory)
    size_t  per;           // p_width*p_height
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


/* PRIMITIVES (namespace gpuray: pure, single-purpose helpers) */

namespace gpuray {


/* Named numerical constants (replace scattered magic numbers) */

// Direction component below which an axis is treated as parallel (no crossing).
__device__ __constant__ const float kDirEpsilon     = 1e-9f;
// "Infinite" parametric step for a parallel axis in a DDA.
__device__ __constant__ const float kTDeltaInfinity = 1e30f;
// Radial divisor in the volumetric gain integral (dr^3 / 6).
__device__ __constant__ const float kGainCubicDiv   = 6.0f;
// Hard cap on DDA iterations -- a runaway backstop that never trips in practice.
__device__ __constant__ const int   kMaxDdaSteps    = 8192;


/* Geometry */

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


/* Voxel grid indexing */

__device__ inline bool in_bounds(int ix, int iy, int iz, int3 dim) {
    return ix >= 0 && ix < dim.x &&
           iy >= 0 && iy < dim.y &&
           iz >= 0 && iz < dim.z;
}

// Row-major flat index into the volumetric map (z outermost, x innermost).
__device__ inline int voxel_flat_index(int ix, int iy, int iz, int3 dim) {
    return iz * (dim.x * dim.y) + iy * dim.x + ix;
}


/* Volumetric information-gain integral (radial term only) */

// Unweighted volume element for an unknown segment (dr at radius r): 2 r^2 dr + dr^3/6; callers apply angular weighting.
__device__ inline float gain_volume_increment(float r, float dr) {
    return 2.0f * r * r * dr + (dr * dr * dr) / kGainCubicDiv;
}


/* 3D DDA (Amanatides & Woo) */

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

// Re-seat the traversal at fractional position g and parametric distance t after a jump (step dirs and tDelta unchanged).
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


/* 2D DDA over a pixel grid (parent depth image) */

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


/* Yaw sliding-window optimisation */

// Slide an FOV-wide window over the per-sector gain histogram; return the best window-start index (total gain to *out_gain).
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

// Start bin of the FOV window centred on `yaw` (rad) -- integer inverse of yaw_window_center_angle.
__device__ inline int window_start_bin_at_yaw(float yaw, float dtheta, float fov_y_rad, int theta_bins) {
    int start = (int)floorf((yaw - 0.5f * fov_y_rad + CUDART_PI_F) / dtheta + 0.5f);
    start %= theta_bins; if (start < 0) start += theta_bins;   // positive modulo (wraps at +-pi)
    return start;
}

// Sum the FOV window centred on `yaw` from the per-sector histogram (fixed-yaw analog of best window).
__device__ inline float window_gain_at_yaw(const float* s_yaw_gains, int theta_bins, int sectors_in_fov,
                                            float dtheta, float fov_y_rad, float yaw) {
    int start = window_start_bin_at_yaw(yaw, dtheta, fov_y_rad, theta_bins);
    float g = 0.0f;
    for (int k = 0; k < sectors_in_fov; ++k) g += s_yaw_gains[(start + k) % theta_bins];
    return g;
}


/* Skip-interval set operations (multi-ancestor occlusion) */

// Insert [lo,hi] into a sorted non-overlapping interval set, coalescing overlaps; bounded by max_intervals.
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


/* ROUTINES (built from the primitives above) */


/* Small shared helpers */

// Map cell value at (ix,iy,iz); out-of-bounds reads as V_FREE (outside == free).
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

// True if the parent surface at pixel (x,y) is a real hit vs the sensor max range (planar depth vs range scaled by pixel slant).
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
    size_t di = a.depth_idx ? (size_t)a.depth_idx[i] : (size_t)i;
    p.depth = a.depth + di * a.cam.p_width * a.cam.p_height;
    p.cam   = a.cam;
    return p;
}

// Slice one candidate's AncestorSet from the batched CSR view (pooled or contiguous depth).
__device__ inline AncestorSet ancestors_for(const AncestorBatchDev& ab, int candidate) {
    int base = ab.offsets[candidate];
    AncestorSet s;
    s.positions = ab.pos + base;
    s.yaws      = ab.yaw + base;
    s.R_rows    = ab.R + (size_t)base * 3;
    s.num       = ab.offsets[candidate + 1] - base;
    s.cam       = ab.cam;
    if (ab.depth_idx) {
        s.depth     = ab.depth;
        s.depth_idx = ab.depth_idx + base;
    } else {
        s.depth     = ab.depth + (size_t)base * ab.per;
        s.depth_idx = nullptr;
    }
    return s;
}

// Liang-Barsky clip of segment a->b to box; false if fully outside, else updates factors *s0,*s1.
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


/* Parent-frustum projection */

// Clip a candidate ray against one parent depth image; rp.valid is false when the ray misses the frustum.
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

// Walk one parent's projected ray, emitting each observed-free span (metres) and latching *status on real occlusion.
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

// Skip set: merge the observed-free spans of every ancestor (count=1 = single-parent, N = full chain).
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


/* Ray marching: integrate volumetric gain along one candidate ray */

// Occupancy-only march: distance (metres) to the first occupied voxel, or the sensor range if none hit.
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

// Plain gain march (AEP): integrate UNKNOWN voxels until OCCUPIED or max range; writes first-hit depth to *out_depth.
__device__ inline float march_gain_basic(const MapContext& m, const MarchRay& ray,
                                         const KernelParams& p, float* out_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float ray_gain = 0.0f;
    *out_depth = p.gain_range;
    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) { *out_depth = d.t * p.voxel_size; break; }
        if (val == V_UNKNOWN) {
            float t_exit = fminf(gpuray::dda3_t_exit(d), max_t);   // cap last voxel at range (match CPU + traverse march)
            if (t_exit > d.t) ray_gain += ray_segment_gain(d.t, t_exit, ray.sin_phi, p);
        }
        gpuray::dda3_step(d);
    }
    return ray_gain;
}

// Single-node marginal march: traverse every voxel (never reseats the DDA), omit gain inside observed-free spans, stop on OCCUPIED inside a span.
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


/* Depth-buffer synthesis (shared by every marginal kernel) */

// Block-strided fill: cast one ray per pixel at the pose, store planar first-hit depth into this candidate's depth_out slice.
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
