#ifndef GPU_RAYCAST_MATH_CUH
#define GPU_RAYCAST_MATH_CUH

// Device raycast library, header only

#include <cuda_runtime.h>
#include <math_constants.h>
#include <math.h>
#include <stdint.h>

/*                  TUNABLES                 */

// Occupancy grid cell states.
#define V_FREE     0
#define V_OCCUPIED 1
#define V_UNKNOWN  2

// Histogram Bin Cap
#define THETA_BINS_MAX 640

#define MAX_THREADS_PER_BLOCK 512

/*                   TYPES                   */

// Per-launch parameters
struct KernelParams {
    float voxel_size;      // Voxel edge length (m)
    float gain_range;      // Maximum ray range r_max (m)
    float fov_y_rad;       // Horizontal field of view (rad)
    float fov_p_rad;       // Vertical field of view (rad)
    float camera_pitch;    // Camera pitch offset (rad)

    float dtheta;          // Azimuth step, yaw (rad)
    float dphi;            // Polar step, pitch (rad)

    float phi_start;       // First polar angle of the sampled band (rad)
    float phi_end;         // Last polar angle of the sampled band (rad)

    int   theta_bins;      // Azimuth bins over the full circle, round(2*pi / dtheta)
    int   rows_in_fov;     // Vertical sample rows, angular_bins(fov_p, dphi)
    int   sectors_in_fov;  // Best-yaw window width, angular_bins(fov_y, dtheta)
};

namespace gpuray {

// Parent camera geometry
struct ParentCameraConfig {
    int   p_width, p_height;
    float fx, fy, cx, cy;
};

// World to camera rotation rows
struct RotationRows {
    float3 r0, r1, r2;
};

// 3D voxel DDA state
struct Dda3 {
    int   ix, iy, iz;
    int   stepX, stepY, stepZ;
    float tDeltaX, tDeltaY, tDeltaZ;
    float tMaxX, tMaxY, tMaxZ;
    float t;
};

// 2D pixel DDA state
struct Dda2 {
    int   x, y;
    int   x_end, y_end;
    int   stepX, stepY;
    float tDeltaX, tDeltaY;
    float tMaxX, tMaxY;
};

}

/*             APPLICATION VIEWS             */

// Occupancy grid view
struct MapContext {
    const uint8_t* map;
    int3           dim;
    float3         origin;
};

// One parent camera frame
struct ParentFrame {
    float3                     pos;    // Parent camera position
    const float*               depth;  // [p_width*p_height] planar depths
    gpuray::RotationRows       R;      // World to camera rotation
    gpuray::ParentCameraConfig cam;    // Depth image geometry
};

// Ancestor chain of a candidate
struct AncestorSet {
    const float3*              positions;  // [num] ancestor positions
    const float*               yaws;       // [num] ancestor yaws (parity only, unused in math)
    const float*               depth;      // [num*per] contiguous depths, or the pool base when pooled
    const float3*              R_rows;     // [num*3] rotation rows R0,R1,R2 per ancestor
    int                        num;        // Number of ancestors
    gpuray::ParentCameraConfig cam;        // Depth image geometry shared by all ancestors
    const int*                 depth_idx;  // Pool slot per ancestor, or null for contiguous depths
};

// Per-candidate output buffers.
struct GainResults {
    float*       gain;       // [num_candidates] gain per candidate
    float*       yaw;        // [num_candidates] chosen yaw per candidate
    float*       depth_all;  // Scratch, one planar depth per cast ray
    float*       depth;      // [p_width*p_height] final depth buffer per candidate
    const float* fixed_yaw;  // [num_candidates] fixed yaw per candidate, or null to optimize yaw
};

// Batched ancestor chains
struct AncestorBatchDev {
    const int*                 offsets;    // [num_candidates+1] prefix sum of ancestor counts
    const float3*              pos;        // [total] ancestor positions
    const float*               yaw;        // [total] ancestor yaws
    const float*               depth;      // [total*per] contiguous depths, or the depth pool when pooled
    const float3*              R;          // [total*3] rotation rows per ancestor
    int                        per;        // Pixels per depth buffer, p_width*p_height
    gpuray::ParentCameraConfig cam;        // Depth image geometry
    const int*                 depth_idx;  // Pool slot per ancestor, or null for contiguous depths
};

// Device buffers of a batched launch
struct BatchDeviceMem {
    float3* d_cand;       // Candidate positions [nc]
    int*    d_off;        // Ancestor offsets [nc+1]
    float3* d_pos;        // Ancestor positions [total]
    float*  d_yaw;        // Ancestor yaws [total]
    float3* d_R;          // Ancestor rotation rows [total*3]
    int*    d_depth_idx;  // Depth pool slot per ancestor [total]
    float*  d_gain;       // Output gain per candidate [nc]
    float*  d_yaw_out;    // Output yaw per candidate [nc]
    float*  d_fixed_yaw;  // Fixed yaw per candidate, null when optimizing yaw
    int*    d_out_slot;   // Depth pool write slot per candidate
    int     rays;         // Rays per candidate
    int     nc;           // Number of candidates
    size_t  per;          // Pixels per depth buffer, p_width*p_height
};

// A candidate ray (world frame).
struct Ray {
    float3 origin;
    float3 dir;
};

// Ray with gain weight
struct MarchRay {
    float3 origin;
    float3 dir;
    float  sin_phi;
};

// Pose for depth synthesis
struct CameraPose {
    float3 pos;
    float  yaw;
};

// Observed free spans (voxels)
struct SkipSet {
    const float2* intervals;
    int           count;
};

// Observed free spans (metres)
struct SkipBuffer {
    float2* intervals;
    int*    count;
    int     capacity;
    float*  status;     // -1 if any ancestor surface occludes the ray
};

// Pixel clip rectangle
struct Rect2 {
    float min_x, max_x, min_y, max_y;
};

// Ray projected into parent image
struct RayProjection {
    bool         valid;                           // False if the ray misses the frustum
    float3       O, D;                            // Ray in the parent camera frame
    float        w_start, w_end;                  // Inverse depth at the clipped endpoints
    float        t_visible_start, t_visible_end;  // Metres at clip entry and exit
    gpuray::Dda2 dda;                             // Pixel walk over the clipped segment
};

/*                 PRIMITIVES                */

namespace gpuray {

/*                 CONSTANTS                 */

// Parallel axis threshold
__device__ __constant__ const float kDirEpsilon     = 1e-9f;
// Infinite DDA step
__device__ __constant__ const float kTDeltaInfinity = 1e30f;
// Gain integral divisor
__device__ __constant__ const float kGainCubicDiv   = 6.0f;
// DDA iteration cap
__device__ __constant__ const int   kMaxDdaSteps    = 8192;

/*                  GEOMETRY                 */

// Unit ray direction from spherical angles
__device__ inline float3 spherical_ray_dir(float theta, float phi) {
    float sin_phi = sinf(phi);
    return make_float3(cosf(theta) * sin_phi, sinf(theta) * sin_phi, cosf(phi));
}

// World point to voxel coordinates
__device__ inline float3 world_to_voxel(float3 p, float3 origin, float voxel_size) {
    return make_float3((p.x - origin.x) / voxel_size,
                       (p.y - origin.y) / voxel_size,
                       (p.z - origin.z) / voxel_size);
}

// Rotate a vector by stored rows
__device__ inline float3 apply_rotation_rows(const RotationRows& R, float3 v) {
    return make_float3(R.r0.x * v.x + R.r0.y * v.y + R.r0.z * v.z,
                       R.r1.x * v.x + R.r1.y * v.y + R.r1.z * v.z,
                       R.r2.x * v.x + R.r2.y * v.y + R.r2.z * v.z);
}

// Pinhole projection to pixels
__device__ inline float2 project_pinhole(float3 p_cam, const ParentCameraConfig& k) {
    float inv_z = 1.0f / p_cam.z;
    return make_float2(k.fx * p_cam.x * inv_z + k.cx,
                       k.fy * p_cam.y * inv_z + k.cy);
}

/*               VOXEL INDEXING              */

__device__ inline bool in_bounds(int ix, int iy, int iz, int3 dim) {
    return ix >= 0 && ix < dim.x &&
           iy >= 0 && iy < dim.y &&
           iz >= 0 && iz < dim.z;
}

// Flat map index
__device__ inline int voxel_flat_index(int ix, int iy, int iz, int3 dim) {
    return iz * (dim.x * dim.y) + iy * dim.x + ix;
}

/*               GAIN INTEGRAL               */

// Volume element of an unknown segment
__device__ inline float gain_volume_increment(float r, float dr) {
    return 2.0f * r * r * dr + (dr * dr * dr) / kGainCubicDiv;
}

/*                   3D DDA                  */

// Start a 3D DDA traversal
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

// Restart a 3D DDA after a jump
__device__ inline void dda3_reseat(Dda3& d, float3 g, float t) {
    d.ix = floor(g.x);
    d.iy = floor(g.y);
    d.iz = floor(g.z);
    d.tMaxX = ((d.stepX > 0) ? (d.ix + 1.0f - g.x) * d.tDeltaX : (g.x - d.ix) * d.tDeltaX) + t;
    d.tMaxY = ((d.stepY > 0) ? (d.iy + 1.0f - g.y) * d.tDeltaY : (g.y - d.iy) * d.tDeltaY) + t;
    d.tMaxZ = ((d.stepZ > 0) ? (d.iz + 1.0f - g.z) * d.tDeltaZ : (g.z - d.iz) * d.tDeltaZ) + t;
    d.t = t;
}

// Exit distance of current voxel
__device__ inline float dda3_t_exit(const Dda3& d) {
    return fminf(d.tMaxX, fminf(d.tMaxY, d.tMaxZ));
}

// Step to the next voxel
__device__ inline void dda3_step(Dda3& d) {
    if (d.tMaxX < d.tMaxY && d.tMaxX < d.tMaxZ) {
        d.ix += d.stepX;
        d.t = d.tMaxX;
        d.tMaxX += d.tDeltaX;
    } else if (d.tMaxY < d.tMaxZ) {
        d.iy += d.stepY;
        d.t = d.tMaxY;
        d.tMaxY += d.tDeltaY;
    } else {
        d.iz += d.stepZ;
        d.t = d.tMaxZ;
        d.tMaxZ += d.tDeltaZ;
    }
}

/*                   2D DDA                  */

// Start a 2D pixel DDA
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

/*                 YAW WINDOW                */

// Best yaw window over the sector histogram
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

// Centre yaw of a window
__device__ inline float yaw_window_center_angle(int best_start_idx, float dtheta, float fov_y_rad) {
    float center_angle = (-CUDART_PI_F + best_start_idx * dtheta) + (fov_y_rad * 0.5f);
    if (center_angle > CUDART_PI_F) center_angle -= (2.0f * CUDART_PI_F);
    return center_angle;
}

// Window start bin for a yaw
__device__ inline int window_start_bin_at_yaw(float yaw, float dtheta, float fov_y_rad, int theta_bins) {
    int start = (int)floorf((yaw - 0.5f * fov_y_rad + CUDART_PI_F) / dtheta + 0.5f);
    start %= theta_bins;
    if (start < 0) start += theta_bins;
    return start;
}

// Window gain at a fixed yaw
__device__ inline float window_gain_at_yaw(const float* s_yaw_gains, int theta_bins, int sectors_in_fov,
                                           float dtheta, float fov_y_rad, float yaw) {
    int start = window_start_bin_at_yaw(yaw, dtheta, fov_y_rad, theta_bins);
    float g = 0.0f;
    for (int k = 0; k < sectors_in_fov; ++k) g += s_yaw_gains[(start + k) % theta_bins];
    return g;
}

/*               SKIP INTERVALS              */

// Insert an interval into a sorted set
__device__ inline void insert_and_merge_interval(float2* intervals, int* count,
                                                 int max_intervals, float lo, float hi) {
    if (hi <= lo) return;
    int i = 0;
    while (i < *count && intervals[i].y < lo) i++;
    int j = i;
    while (j < *count && intervals[j].x <= hi) {
        lo = fminf(lo, intervals[j].x);
        hi = fmaxf(hi, intervals[j].y);
        j++;
    }
    int tail = *count - j;
    if (i + 1 + tail > max_intervals) {
        tail = max_intervals - (i + 1);
        if (tail < 0) tail = 0;
    }
    for (int k = tail - 1; k >= 0; --k) intervals[i + 1 + k] = intervals[j + k];
    intervals[i] = make_float2(lo, hi);
    *count = i + 1 + tail;
}

}

/*                  ROUTINES                 */

/*               SHARED HELPERS              */

// Map cell value, free outside
__device__ inline uint8_t voxel_value(const MapContext& m, int ix, int iy, int iz) {
    if (!gpuray::in_bounds(ix, iy, iz, m.dim)) return V_FREE;
    return m.map[gpuray::voxel_flat_index(ix, iy, iz, m.dim)];
}

// Weighted gain of an unknown span
__device__ inline float ray_segment_gain(float t_enter, float t_exit, float sin_phi,
                                         const KernelParams& p) {
    float dr = (t_exit - t_enter) * p.voxel_size;
    float r  = t_enter * p.voxel_size;
    return gpuray::gain_volume_increment(r, dr) * p.dtheta * sin_phi * sinf(p.dphi * 0.5f);
}

// Segment factor to metres
__device__ inline float segment_factor_to_metres(const RayProjection& rp, float factor) {
    if (fabsf(rp.D.z) > 1e-3f) {
        float w = rp.w_start + factor * (rp.w_end - rp.w_start);
        return ((1.0f / w) - rp.O.z) / rp.D.z;
    }
    return rp.t_visible_start + factor * (rp.t_visible_end - rp.t_visible_start);
}

// Is the parent pixel a real hit
__device__ inline bool parent_surface_is_real(const gpuray::ParentCameraConfig& cam,
                                              int x, int y, float parent_z, float range) {
    float px_u = (x + 0.5f - cam.cx) / cam.fx;
    float px_v = (y + 0.5f - cam.cy) / cam.fy;
    float cos_theta = rsqrtf(px_u * px_u + px_v * px_v + 1.0f);
    return parent_z < range * cos_theta;
}

// Parent frame of ancestor i
__device__ inline ParentFrame ancestor_frame(const AncestorSet& a, int i) {
    ParentFrame p;
    p.pos   = a.positions[i];
    p.R     = {a.R_rows[i * 3 + 0], a.R_rows[i * 3 + 1], a.R_rows[i * 3 + 2]};
    size_t di = a.depth_idx ? (size_t)a.depth_idx[i] : (size_t)i;
    p.depth = a.depth + di * a.cam.p_width * a.cam.p_height;
    p.cam   = a.cam;
    return p;
}

// Ancestor set of one candidate
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

// Liang-Barsky segment clip
__device__ inline bool clip_line_2d(float2 a, float2 b, Rect2 box, float* s0, float* s1) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float p[4] = {-dx, dx, -dy, dy};
    float q[4] = {a.x - box.min_x, box.max_x - a.x, a.y - box.min_y, box.max_y - a.y};

    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0f) {
            if (q[i] < 0.0f) return false;
        } else {
            float r = q[i] / p[i];
            if (p[i] < 0.0f) {
                if (r > *s1) return false;
                if (r > *s0) *s0 = r;
            } else {
                if (r < *s0) return false;
                if (r < *s1) *s1 = r;
            }
        }
    }
    return true;
}

/*             PARENT PROJECTION             */

// Project a ray into a parent image
__device__ inline RayProjection project_ray_into_parent(const ParentFrame& parent,
                                                        Ray ray, float max_dist) {
    RayProjection rp;
    rp.valid = false;

    const float z_near = 0.1f;
    const float z_far  = max_dist;

    // 1) Ray into Camera Frame
    float3 diff = make_float3(ray.origin.x - parent.pos.x,
                              ray.origin.y - parent.pos.y,
                              ray.origin.z - parent.pos.z);
    rp.O = gpuray::apply_rotation_rows(parent.R, diff);
    rp.D = gpuray::apply_rotation_rows(parent.R, ray.dir);

    // 2) Depth Range Clip
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

    // 3) Project Endpoints
    float inv_z0 = 1.0f / P_start.z;
    float inv_z1 = 1.0f / P_end.z;
    float2 px0 = gpuray::project_pinhole(P_start, parent.cam);
    float2 px1 = gpuray::project_pinhole(P_end, parent.cam);

    // 4) Screen Clip
    float s_min = 0.0f;
    float s_max = 1.0f;
    float eps = 1e-4f;
    Rect2 box = {eps, (float)parent.cam.p_width - eps, eps, (float)parent.cam.p_height - eps};
    if (!clip_line_2d(px0, px1, box, &s_min, &s_max)) return rp;

    // 5) Frustum Interval
    rp.w_start = inv_z0 + s_min * (inv_z1 - inv_z0);
    rp.w_end   = inv_z0 + s_max * (inv_z1 - inv_z0);
    if (fabsf(rp.D.z) > 1e-3f) {
        rp.t_visible_start = ((1.0f / rp.w_start) - rp.O.z) / rp.D.z;
        rp.t_visible_end   = ((1.0f / rp.w_end)   - rp.O.z) / rp.D.z;
    } else {
        rp.t_visible_start = t0 + s_min * (t1 - t0);
        rp.t_visible_end   = t0 + s_max * (t1 - t0);
    }

    // 6) Pixel DDA Setup
    float2 start = make_float2(px0.x + s_min * (px1.x - px0.x), px0.y + s_min * (px1.y - px0.y));
    float2 end   = make_float2(px0.x + s_max * (px1.x - px0.x), px0.y + s_max * (px1.y - px0.y));
    rp.dda = gpuray::dda2_init(start, end, parent.cam.p_width, parent.cam.p_height);
    rp.valid = true;
    return rp;
}

// Observed free spans from one parent
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
                // Refine Crossing
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
            // Close Interval
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
        if (d.tMaxX < d.tMaxY) {
            d.x += d.stepX;
            current_t = d.tMaxX;
            d.tMaxX += d.tDeltaX;
        } else {
            d.y += d.stepY;
            current_t = d.tMaxY;
            d.tMaxY += d.tDeltaY;
        }
    }

    // Close at Frustum Exit
    if (is_building && *skips.count < skips.capacity) {
        float a = segment_factor_to_metres(rp, segment_start_t);
        float b = segment_factor_to_metres(rp, 1.0f);
        if (b > a + 1e-4f) {
            gpuray::insert_and_merge_interval(skips.intervals, skips.count, skips.capacity, a, b);
        }
    }
}

// Skip set over all ancestors
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

/*                RAY MARCHING               */

// Distance to first occupied voxel
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

// Absolute gain along one ray
__device__ inline float march_gain_basic(const MapContext& m, const MarchRay& ray,
                                         const KernelParams& p, float* out_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float ray_gain = 0.0f;
    *out_depth = p.gain_range;
    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) {
            *out_depth = d.t * p.voxel_size;
            break;
        }
        if (val == V_UNKNOWN) {
            float t_exit = fminf(gpuray::dda3_t_exit(d), max_t);
            if (t_exit > d.t) ray_gain += ray_segment_gain(d.t, t_exit, ray.sin_phi, p);
        }
        gpuray::dda3_step(d);
    }
    return ray_gain;
}

// Marginal gain along one ray
__device__ inline float march_marginal_gain_traverse(const MapContext& m, const MarchRay& ray,
                                                     SkipSet skips, const KernelParams& p,
                                                     float* out_final_depth) {
    gpuray::Dda3 d = gpuray::dda3_init(gpuray::world_to_voxel(ray.origin, m.origin, p.voxel_size), ray.dir);
    float max_t = p.gain_range / p.voxel_size;
    float final_depth = p.gain_range;
    int current_skip_idx = 0;
    float ray_gain = 0.0f;

    for (int s = 0; s < gpuray::kMaxDdaSteps && d.t < max_t; ++s) {
        // Drop Passed Intervals
        while (current_skip_idx < skips.count && d.t >= skips.intervals[current_skip_idx].y) current_skip_idx++;

        uint8_t val = voxel_value(m, d.ix, d.iy, d.iz);
        if (val == V_OCCUPIED) {
            final_depth = d.t * p.voxel_size;
            break;
        }
        if (val == V_UNKNOWN) {
            // Inside Observed Span
            bool inside = (current_skip_idx < skips.count &&
                           d.t >= skips.intervals[current_skip_idx].x &&
                           d.t <  skips.intervals[current_skip_idx].y);
            if (!inside) {
                float t_exit = fminf(gpuray::dda3_t_exit(d), max_t);
                if (current_skip_idx < skips.count && t_exit > skips.intervals[current_skip_idx].x) {
                    t_exit = fminf(t_exit, skips.intervals[current_skip_idx].x);
                }
                if (t_exit > d.t) ray_gain += ray_segment_gain(d.t, t_exit, ray.sin_phi, p);
            }
        }
        gpuray::dda3_step(d);
    }
    *out_final_depth = final_depth;
    return ray_gain;
}

/*              DEPTH SYNTHESIS              */

// Depth buffer of one pose
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
        float dir_z = -z_cam * sin_p - y_cam * cos_p;
        float inv_norm = 1.0f / sqrtf(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
        float3 dir = make_float3(dir_x * inv_norm, dir_y * inv_norm, dir_z * inv_norm);

        float final_depth = march_first_hit(m, pose.pos, dir, p);
        float cos_theta = rsqrtf(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam);
        depth_out[idx] = final_depth * cos_theta;
    }
}

#endif  // GPU_RAYCAST_MATH_CUH
