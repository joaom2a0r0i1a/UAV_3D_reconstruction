// ============================================================================
//  gpu_tester -- head-to-head benchmark for the two batched marginal-gain
//  architectures (both use the single-node traverse march):
//    fused  (Option 1): one kernel per candidate, check-then-march per ray.
//    split  (Option 2): kernel A writes merged skip intervals to global memory,
//                       kernel B reads them back and marches (tiled).
//
//  Self-contained: builds a synthetic-but-representative wavefront in memory,
//  runs both architectures on the identical inputs, reports device kernel time
//  (CUDA events) and a gain-parity check, swept over candidate-batch sizes.
//
//  Modes:
//    gpu_tester [nc] [max_depth] [grid]   synthetic sweep 16..nc  (default nc=500000)
//    gpu_tester --verify [N]              cross-check the batched kernels against
//                                         the per-candidate single-node launcher
//
//  Depth buffers come from a SHARED POOL indexed per ancestor (see depth_idx),
//  so device memory stays O(pool + slots) rather than O(candidates*depth*per) --
//  that is what lets the sweep reach 500k candidates on an 8 GB GPU.
// ============================================================================

#include <ros/ros.h>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

#include "gain_evaluation/gpu_raycast_launch.h"

// One synthetic wavefront held in host memory.
struct Batch {
    int32_t dx, dy, dz;
    float   ox, oy, oz;
    float   voxel, r_max, fov_y, fov_p, pitch;
    int32_t p_width, p_height;
    int32_t num_candidates, total_ancestors, num_nodes;
    std::vector<uint8_t> map;                 // dx*dy*dz occupancy
    std::vector<float>   cx, cy, cz;          // candidate positions
    std::vector<int32_t> offsets;             // CSR prefix sum (num_candidates+1)
    std::vector<float>   apos, ayaw, aR;      // per-ancestor-slot pose (3/1/9 each)
    std::vector<float>   adepth;              // depth pool (num_nodes * per)
    std::vector<int32_t> depth_idx;           // pool index per ancestor slot (total)
    size_t per() const { return (size_t)p_width * p_height; }
};

// Build a synthetic-but-representative POOLED batch (fixed seed -> reproducible).
// Depth buffers are drawn from a shared pool of num_nodes buffers indexed per
// ancestor slot, keeping memory O(pool + total_slots) instead of O(total*per).
// (Realism caveat: the shared pool gives the parent-buffer reads more L2 reuse
// than truly unique per-node buffers; both architectures read the same pool, so
// the comparison stays fair.)
static void synthesize(Batch& b, int nc, int max_depth, int D) {
    b.voxel = 0.2f; b.r_max = 5.0f;
    b.fov_y = 1.51844f; b.fov_p = 1.0123f; b.pitch = 0.1745f;
    b.dx = b.dy = b.dz = D;
    b.ox = b.oy = b.oz = 0.0f;
    b.p_width  = (int)ceil((2.0f * b.r_max * tanf(b.fov_y * 0.5f)) / b.voxel);
    b.p_height = (int)ceil((2.0f * b.r_max * tanf(b.fov_p * 0.5f)) / b.voxel);
    size_t per = b.per();

    std::mt19937 rng(12345);
    size_t ncells = (size_t)D * D * D;
    b.map.assign(ncells, 2 /*V_UNKNOWN*/);
    std::uniform_int_distribution<size_t> cell(0, ncells - 1);
    for (size_t i = 0; i < ncells / 40; ++i) b.map[cell(rng)] = 1 /*V_OCCUPIED*/;

    const float c = D * b.voxel * 0.5f;
    std::uniform_real_distribution<float> jitter(-D * b.voxel * 0.2f, D * b.voxel * 0.2f);
    b.cx.resize(nc); b.cy.resize(nc); b.cz.resize(nc);
    for (int i = 0; i < nc; ++i) { b.cx[i] = c + jitter(rng); b.cy[i] = c + jitter(rng); b.cz[i] = c + jitter(rng); }

    b.offsets.assign(nc + 1, 0);
    std::uniform_int_distribution<int> depthd(1, max_depth);
    for (int i = 0; i < nc; ++i) b.offsets[i + 1] = b.offsets[i] + depthd(rng);
    int total = b.offsets[nc];
    b.num_candidates = nc;
    b.total_ancestors = total;

    // Distinct depth buffers = tree nodes, so the realistic pool is num_nodes == nc
    // (each candidate is a node with its own buffer). Keep it that way up to a
    // VRAM-derived cap, so the ancestor-buffer reads have a representative working
    // set (L2 -> DRAM) instead of an artificially tiny, cache-resident pool.
    const size_t POOL_BUDGET = (size_t)5 * 1024 * 1024 * 1024;   // ~5 GB for the pool
    int pool_cap = (int)std::min<size_t>((size_t)nc, POOL_BUDGET / (per * sizeof(float)));
    b.num_nodes = std::max(1, pool_cap);
    b.adepth.resize((size_t)b.num_nodes * per);
    std::uniform_real_distribution<float> depthv(b.r_max * 0.3f, b.r_max * 0.9f);
    for (size_t j = 0; j < b.adepth.size(); ++j) b.adepth[j] = depthv(rng);

    b.apos.resize(3 * total); b.ayaw.resize(total); b.aR.resize(9 * total);
    b.depth_idx.resize(total);
    std::uniform_real_distribution<float> yawd(-M_PI, M_PI);
    std::uniform_real_distribution<float> near(-0.5f, 0.5f);
    std::uniform_int_distribution<int> poold(0, b.num_nodes - 1);
    for (int ci = 0; ci < nc; ++ci) {
        for (int k = b.offsets[ci]; k < b.offsets[ci + 1]; ++k) {
            b.apos[3 * k + 0] = b.cx[ci] + near(rng);
            b.apos[3 * k + 1] = b.cy[ci] + near(rng);
            b.apos[3 * k + 2] = b.cz[ci] + near(rng);
            b.ayaw[k] = yawd(rng);
            b.aR[9 * k + 0] = 1; b.aR[9 * k + 1] = 0; b.aR[9 * k + 2] = 0;
            b.aR[9 * k + 3] = 0; b.aR[9 * k + 4] = 1; b.aR[9 * k + 5] = 0;
            b.aR[9 * k + 6] = 0; b.aR[9 * k + 7] = 0; b.aR[9 * k + 8] = 1;
            b.depth_idx[k] = poold(rng);              // scattered read into the pool
        }
    }
}

// Assemble the GpuAncestorBatch view over the first N candidates of b (depth lives in the device pool).
static GpuAncestorBatch ancestor_batch(const Batch& b, int N, int total_N) {
    return GpuAncestorBatch{N, b.offsets.data(), total_N, b.apos.data(), b.ayaw.data(),
                            b.aR.data(), b.depth_idx.data()};
}

// Upload the synthetic ancestor depth (b.adepth) into a device pool [0..num_nodes) and give each of the N
// candidates a tail write-slot (num_nodes + c). Returns the pool; caller frees with wrapper_depth_pool_free.
static float* make_pool(const Batch& b, int N, std::vector<int>& out_slot) {
    int per_i = (int)b.per();
    float* d_pool = nullptr; int cap = 0;
    wrapper_depth_pool_ensure(&d_pool, &cap, b.num_nodes + N, per_i);          // alloc + fill -1
    wrapper_cuda_memcpy(d_pool, b.adepth.data(), (size_t)b.num_nodes * per_i * sizeof(float));
    out_slot.resize(N);
    for (int c = 0; c < N; ++c) out_slot[c] = b.num_nodes + c;                 // candidates write to the tail
    return d_pool;
}

// Fewer timed iterations as N grows (each run is longer and re-uploads more).
static int iters_for(int N) {
    if (N <= 4096)  return 20;
    if (N <= 65536) return 8;
    return 3;
}

// Run fused and split on the first N candidates, time each, report parity.
static void run_and_report(uint8_t* d_map, const Batch& b, int N, int iters) {
    N = std::min(N, b.num_candidates);
    int total_N = b.offsets[N];
    GpuMap map = {d_map, b.dx, b.dy, b.dz, b.ox, b.oy, b.oz};
    GpuSensor cfg = {b.voxel, b.r_max, b.fov_y, b.fov_p, b.pitch};
    GpuCandidates cands = {(float*)b.cx.data(), (float*)b.cy.data(), (float*)b.cz.data(), N};
    GpuAncestorBatch anc = ancestor_batch(b, N, total_N);

    std::vector<float> gF(N), yF(N), gS(N), yS(N);
    GpuResult oF = {gF.data(), yF.data(), nullptr};
    GpuResult oS = {gS.data(), yS.data(), nullptr};
    std::vector<int> out_slot;
    float* d_pool = make_pool(b, N, out_slot);

    float ms = 0.0f; double sF = 0, sS = 0;
    launch_marginal_gain_batch_fused(map, cands, anc, oF, cfg, &ms, nullptr, d_pool, out_slot.data());   // warm-up
    launch_marginal_gain_batch_split(map, cands, anc, oS, cfg, &ms, nullptr, d_pool, out_slot.data());
    for (int it = 0; it < iters; ++it) { launch_marginal_gain_batch_fused(map, cands, anc, oF, cfg, &ms, nullptr, d_pool, out_slot.data()); sF += ms; }
    for (int it = 0; it < iters; ++it) { launch_marginal_gain_batch_split(map, cands, anc, oS, cfg, &ms, nullptr, d_pool, out_slot.data()); sS += ms; }
    double f = sF / iters, s = sS / iters;

    double d = 0;
    for (int i = 0; i < N; ++i) d = std::max(d, fabs((double)gF[i] - gS[i]));
    printf("%7d | %9d | %10.4f | %10.4f | %-6s | %.2e\n",
           N, total_N, f, s, f <= s ? "fused" : "split", d);
    wrapper_depth_pool_free(d_pool);
}

// Cross-check the batched kernels against the per-candidate reference launcher.
// Both batched kernels use the single-node traverse march, so they must match the
// single-node reference to ~atomic-ordering noise.
static void verify_against_reference(uint8_t* d_map, const Batch& b, int N) {
    N = std::min(N, b.num_candidates);
    size_t per = b.per();
    GpuMap map = {d_map, b.dx, b.dy, b.dz, b.ox, b.oy, b.oz};
    GpuSensor cfg = {b.voxel, b.r_max, b.fov_y, b.fov_p, b.pitch};

    int total_N = b.offsets[N];
    GpuCandidates cands = {(float*)b.cx.data(), (float*)b.cy.data(), (float*)b.cz.data(), N};
    GpuAncestorBatch anc = ancestor_batch(b, N, total_N);
    std::vector<float> gF(N), yF(N), gS(N), yS(N);
    GpuResult oF = {gF.data(), yF.data(), nullptr};
    GpuResult oS = {gS.data(), yS.data(), nullptr};
    std::vector<int> out_slot;
    float* d_pool = make_pool(b, N, out_slot);
    float ms = 0.0f;
    launch_marginal_gain_batch_fused(map, cands, anc, oF, cfg, &ms, nullptr, d_pool, out_slot.data());
    launch_marginal_gain_batch_split(map, cands, anc, oS, cfg, &ms, nullptr, d_pool, out_slot.data());
    wrapper_depth_pool_free(d_pool);

    // Per-candidate reference: materialize each chain with a CONTIGUOUS depth
    // block gathered from the pool (the single-node launcher expects count*per depths).
    double mF = 0, mS = 0, mFixed = 0, mSingle = 0;
    int nSingle = 0;
    printf("\n  cand | anc |  single  |  fused   |  split\n");
    printf("-------+-----+----------+----------+----------\n");
    for (int c = 0; c < N; ++c) {
        int base = b.offsets[c], cnt = b.offsets[c + 1] - base;
        std::vector<float> pos(3 * cnt), yaw(cnt), R(9 * cnt), dep((size_t)cnt * per);
        for (int k = 0; k < cnt; ++k) {
            pos[3 * k + 0] = b.apos[3 * (base + k) + 0];
            pos[3 * k + 1] = b.apos[3 * (base + k) + 1];
            pos[3 * k + 2] = b.apos[3 * (base + k) + 2];
            yaw[k] = b.ayaw[base + k];
            for (int j = 0; j < 9; ++j) R[9 * k + j] = b.aR[9 * (base + k) + j];
            size_t src = (size_t)b.depth_idx[base + k] * per;
            for (size_t j = 0; j < per; ++j) dep[(size_t)k * per + j] = b.adepth[src + j];
        }
        GpuVec3 cand = {b.cx[c], b.cy[c], b.cz[c]};
        GpuAncestors ga = {cnt, pos.data(), yaw.data(), R.data(), dep.data()};
        float g4 = 0, y4 = 0;
        GpuResult o4 = {&g4, &y4, nullptr};
        launch_marginal_gain(map, cand, ga, o4, cfg);
        mF = std::max(mF, fabs((double)gF[c] - g4));
        mS = std::max(mS, fabs((double)gS[c] - g4));

        // Fixed-yaw launcher parity: evaluating the FOV window at the optimizer's own
        // yaw must reproduce its gain (validates launch_marginal_gain_fixed + pick_yaw_window).
        float gfx = 0, yfx = 0;
        GpuResult ofx = {&gfx, &yfx, nullptr};
        launch_marginal_gain_fixed(map, cand, ga, ofx, cfg, y4);
        mFixed = std::max(mFixed, fabs((double)gfx - g4));

        // Single-parent (count=1) explicit track vs the batched path.
        if (cnt == 1) {
            mSingle = std::max(mSingle, std::max(fabs((double)gF[c] - g4), fabs((double)gS[c] - g4)));
            ++nSingle;
        }
        if (c < 12)
            printf("%6d | %3d | %8.3f | %8.3f | %8.3f\n", c, cnt, g4, gF[c], gS[c]);
    }
    printf("\nmax|batched - single| : fused=%.2e  split=%.2e   (expect ~1e-5, atomic order)\n", mF, mS);
    printf("single-parent (count=1): %d candidates,  max|batched - single| = %.2e\n", nSingle, mSingle);
    printf("fixed-yaw launcher parity: max|fixed(opt yaw) - optimize| = %.2e   (expect ~0, same window)\n", mFixed);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gpu_arch_bench");

    if (argc >= 2 && strcmp(argv[1], "--verify") == 0) {
        int N = (argc > 2) ? atoi(argv[2]) : 64;
        Batch b; synthesize(b, std::max(N, 64), 8, 200);
        uint8_t* d_map = nullptr;
        wrapper_cuda_malloc(&d_map, b.map.size() * sizeof(uint8_t));
        wrapper_cuda_memcpy(d_map, b.map.data(), b.map.size() * sizeof(uint8_t));
        printf("verify: N=%d candidates (pooled synthetic), per=%zu\n", N, b.per());
        verify_against_reference(d_map, b, N);
        wrapper_cuda_free(d_map);
        return 0;
    }

    int nc = (argc > 1) ? atoi(argv[1]) : 500000;
    int md = (argc > 2) ? atoi(argv[2]) : 8;
    int D  = (argc > 3) ? atoi(argv[3]) : 200;
    Batch b; synthesize(b, nc, md, D);
    double pool_gb = (double)b.num_nodes * b.per() * sizeof(float) / 1e9;
    printf("synthetic(pooled): nc=%d total_anc=%d pool=%d buffers (%.2f GB)%s grid=%d^3 per=%zu\n",
           nc, b.total_ancestors, b.num_nodes, pool_gb,
           b.num_nodes < nc ? " [CAPPED by VRAM -> depth reads more L2-optimistic than a real tree this size]" : "",
           D, b.per());

    uint8_t* d_map = nullptr;
    wrapper_cuda_malloc(&d_map, b.map.size() * sizeof(uint8_t));
    wrapper_cuda_memcpy(d_map, b.map.data(), b.map.size() * sizeof(uint8_t));

    printf("\n      N |  anc(<=N) |   fused ms |   split ms | best   | dparity\n");
    printf("--------+-----------+------------+------------+--------+---------\n");
    for (int N = 16; ; N *= 2) {
        run_and_report(d_map, b, std::min(N, nc), iters_for(N));
        if (N >= nc) break;
    }

    wrapper_cuda_free(d_map);
    return 0;
}
