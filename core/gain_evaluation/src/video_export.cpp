// Offline scenario exporter for the supplementary video.

#include <ros/ros.h>
#include <voxblox_ros/esdf_server.h>
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "gain_evaluation/gain_evaluator.h"
#include <rrt_construction/rrt_star_kd.h>

namespace {

constexpr uint8_t V_FREE = 0, V_OCCUPIED = 1, V_UNKNOWN = 2;

struct Args {
    std::string map_path, out_dir;
    double rx = 10.924, ry = -7.026, rz = 1.209, ryaw = -1.823;
    int n_max = 250, seed = 12345;
    int min_depth = 3, max_depth = 6;
    double min_hit = 0.08, max_hit = 0.40;
    double min_parent_hit = 0.12;
    double min_branch_hit = 0.05;
    double min_spacing = 1.0;
    int select_id = -1;
    std::string tag;
    double step_size = 1.5, uav_radius = 0.8, min_edge = 0.2;
    double min_x = -20, max_x = 20, min_y = -20, max_y = 20, min_z = 0.0, max_z = 2.5;
    double box_min_z = 0.5, box_max_z = 1.8;
    double voxel = 0.1;
};

bool getFlag(int argc, char** argv, const std::string& key, std::string& out) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (key == argv[i]) {
            out = argv[i + 1];
            return true;
        }
    }
    return false;
}
template <typename T>
void getNum(int argc, char** argv, const std::string& key, T& dst) {
    std::string s;
    if (getFlag(argc, argv, key, s)) {
        dst = (T)atof(s.c_str());
    }
}

/*              FLAT MAP HELPERS             */

struct Grid {
    const std::vector<uint8_t>* m;
    Eigen::Vector3d origin;
    Eigen::Vector3i dim;
    double vs;

    inline uint8_t at(int x, int y, int z) const {
        if (x < 0 || y < 0 || z < 0 || x >= dim.x() || y >= dim.y() || z >= dim.z()) {
            return V_UNKNOWN;
        }
        return (*m)[(size_t)(z * dim.y() + y) * dim.x() + x];
    }
    inline Eigen::Vector3i idx(const Eigen::Vector3d& p) const {
        return Eigen::Vector3i((int)std::floor((p.x() - origin.x()) / vs),
                               (int)std::floor((p.y() - origin.y()) / vs),
                               (int)std::floor((p.z() - origin.z()) / vs));
    }
    inline Eigen::Vector3d centre(int x, int y, int z) const {
        return Eigen::Vector3d(origin.x() + (x + 0.5) * vs,
                               origin.y() + (y + 0.5) * vs,
                               origin.z() + (z + 0.5) * vs);
    }
};

// Is there a wall between a and b
bool wallBetween(const Grid& g, const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    const Eigen::Vector3d d = b - a;
    const int n = std::max(1, (int)std::ceil(d.norm() / (g.vs * 0.5)));
    for (int i = 0; i <= n; ++i) {
        const Eigen::Vector3i c = g.idx(a + d * ((double)i / n));
        if (g.at(c.x(), c.y(), c.z()) == V_OCCUPIED) {
            return true;
        }
    }
    return false;
}

// Fraction of rays that hit a voxel
double hitFraction(const std::vector<float>& db, int w, int h,
                   double fx, double fy, double cx, double cy, double r_max) {
    size_t hits = 0;
    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            const double xc = (u - cx) / fx, yc = (v - cy) / fy;
            const double shell = r_max / std::sqrt(xc * xc + yc * yc + 1.0);
            if (db[(size_t)v * w + u] < shell - 1e-3) {
                ++hits;
            }
        }
    }
    return (double)hits / (double)(w * h);
}

std::vector<rrt_star::Node*> ancestorsOf(rrt_star::Node* n) {
    std::vector<rrt_star::Node*> a;
    for (rrt_star::Node* p = n->parent; p; p = p->parent) {
        a.push_back(p);
    }
    return a;
}
int depthOf(rrt_star::Node* n) {
    return (int)ancestorsOf(n).size();
}

void writeFloats(const std::string& path, const std::vector<float>& v) {
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(v.data()), (std::streamsize)(v.size() * sizeof(float)));
}

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_export");
    ros::NodeHandle nh, nhp("~");

    Args A;
    getFlag(argc, argv, "--map", A.map_path);
    getFlag(argc, argv, "--out", A.out_dir);
    getNum(argc, argv, "--rx", A.rx);
    getNum(argc, argv, "--ry", A.ry);
    getNum(argc, argv, "--rz", A.rz);
    getNum(argc, argv, "--ryaw", A.ryaw);
    getNum(argc, argv, "--nmax", A.n_max);
    getNum(argc, argv, "--seed", A.seed);
    getNum(argc, argv, "--step", A.step_size);
    getNum(argc, argv, "--radius", A.uav_radius);
    getNum(argc, argv, "--mindepth", A.min_depth);
    getNum(argc, argv, "--maxdepth", A.max_depth);
    getNum(argc, argv, "--minhit", A.min_hit);
    getNum(argc, argv, "--maxhit", A.max_hit);
    getNum(argc, argv, "--minparenthit", A.min_parent_hit);
    getNum(argc, argv, "--minbranchhit", A.min_branch_hit);
    getNum(argc, argv, "--minedge", A.min_edge);
    getNum(argc, argv, "--minspacing", A.min_spacing);
    getNum(argc, argv, "--selectid", A.select_id);
    getFlag(argc, argv, "--tag", A.tag);
    if (A.map_path.empty() || A.out_dir.empty()) {
        fprintf(stderr, "usage: video_export --map <f.vxblx> --out <dir> [--rx .. --nmax ..]\n");
        return 2;
    }

    /*                 PARAMETERS                */
    nhp.setParam("gain_evaluation/min_x", A.min_x);
    nhp.setParam("gain_evaluation/max_x", A.max_x);
    nhp.setParam("gain_evaluation/min_y", A.min_y);
    nhp.setParam("gain_evaluation/max_y", A.max_y);
    nhp.setParam("gain_evaluation/min_z", A.min_z);
    nhp.setParam("gain_evaluation/max_z", A.max_z);
    nhp.setParam("camera_intrinsics/h_fov", 1.51844);
    nhp.setParam("camera_intrinsics/v_fov", 1.01229);
    nhp.setParam("camera_intrinsics/max_distance", 5.0);
    nhp.setParam("camera_intrinsics/yaw_samples", 15);
    nhp.setParam("camera_intrinsics/pitch", 10.0);
    nhp.setParam("tsdf_voxel_size", A.voxel);
    nhp.setParam("tsdf_voxels_per_side", 16);
    nhp.setParam("publish_esdf_map", false);
    nhp.setParam("publish_tsdf_map", false);

    /*                    MAP                    */
    voxblox::EsdfServer server(nh, nhp);
    ROS_INFO("[video_export] loading %s", A.map_path.c_str());
    if (!server.loadMap(A.map_path)) {
        ROS_ERROR("loadMap failed");
        return 1;
    }
    ROS_INFO("[video_export] building ESDF from TSDF (batch)...");
    server.updateEsdfBatch(true);

    GainEvaluator ev(nhp);
    ev.setTsdfLayer(server.getTsdfMapPtr()->getTsdfLayerPtr());
    ev.setEsdfMap(server.getEsdfMapPtr());

    const double h_fov = 1.51844;
    const double v_fov_cam = ev.getVerticalFoV(h_fov, 1080, 720);
    ev.setCameraModelParametersFoV(h_fov, v_fov_cam, 0.2, 5.0);

    // Camera Extrinsics
    {
        Eigen::Affine3d B_C = Eigen::Affine3d::Identity();
        B_C.translation() = Eigen::Vector3d(0.16, 0.0, -0.089);
        B_C.linear() = Eigen::Quaterniond(0.9962, 0.0, 0.0872, 0.0).normalized().toRotationMatrix();
        Eigen::Affine3d link_depth = Eigen::Affine3d::Identity();
        link_depth.translation() = Eigen::Vector3d(0.0, -0.0115, 0.0);
        const Eigen::Affine3d B_D = B_C * link_depth;
        const Eigen::Affine3d D_B = B_D.inverse();
        voxblox::Transformation::Vector3 t(
            (float)D_B.translation().x(), (float)D_B.translation().y(), (float)D_B.translation().z());
        Eigen::Quaterniond q(D_B.linear());
        voxblox::Transformation T_C_B(
            t, voxblox::Rotation(voxblox::Rotation::Implementation(
                   (float)q.w(), (float)q.x(), (float)q.y(), (float)q.z())));
        ev.setCameraExtrinsics(T_C_B);
    }

    Eigen::Vector3d origin;
    Eigen::Vector3i dim;
    std::vector<uint8_t> flat = ev.flattenMap(origin, dim);
    if (flat.empty()) {
        ROS_ERROR("flattenMap empty");
        return 1;
    }
    ev.cacheMapOnGPU(flat, origin, dim);
    Grid grid{&flat, origin, dim, ev.getVoxelSize()};

    size_t n_occ = 0, n_unk = 0, n_free = 0;
    for (uint8_t c : flat) {
        (c == V_OCCUPIED ? n_occ : (c == V_UNKNOWN ? n_unk : n_free))++;
    }
    const int p_w = (int)std::ceil((2.0 * 5.0 * std::tan(1.51844 * 0.5)) / ev.getVoxelSize());
    const int p_h = (int)std::ceil((2.0 * 5.0 * std::tan(1.01229 * 0.5)) / ev.getVoxelSize());
    ROS_INFO("[video_export] grid %dx%dx%d voxel %.3f | occ %zu unk %zu free %zu | buffer %dx%d = %d rays",
             dim.x(), dim.y(), dim.z(), ev.getVoxelSize(), n_occ, n_unk, n_free, p_w, p_h, p_w * p_h);

    /*                    TREE                   */
    rrt_star T;
    T.setEdgeCollisionChecker([&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        const Eigen::Vector3d d = b - a;
        const int n = std::max(1, (int)std::ceil(d.norm() / 0.1));
        for (int i = 0; i <= n; ++i) {
            double dist = 0.0;
            const Eigen::Vector3d p = a + d * ((double)i / n);
            if (!server.getEsdfMapPtr()->getDistanceAtPosition(p, &dist) || dist < A.uav_radius) {
                return false;
            }
        }
        return true;
    });

    auto clearance = [&](const Eigen::Vector3d& p) {
        double dist = 0.0;
        if (!server.getEsdfMapPtr()->getDistanceAtPosition(p, &dist)) {
            return 0.0;
        }
        return dist;
    };

    T.clearKDTree();
    rrt_star::Node* root = T.addKDTreeNode(
        std::make_unique<rrt_star::Node>(Eigen::Vector4d(A.rx, A.ry, A.rz, A.ryaw)));

    const double bounded_radius = std::sqrt(std::pow(A.min_x - A.max_x, 2.0) +
                                            std::pow(A.min_y - A.max_y, 2.0) +
                                            std::pow(A.box_min_z - A.box_max_z, 2.0));
    std::mt19937 rng(A.seed);
    int added = 0, tries = 0;
    long rj_steer = 0, rj_box = 0, rj_clear = 0, rj_edge = 0;
    const int max_tries = A.n_max * 2000;
    while (added < A.n_max && tries < max_tries) {
        ++tries;
        // Seeded Sampling
        Eigen::Vector3d rand_point;
        {
            std::uniform_real_distribution<double> dis(-bounded_radius, bounded_radius);
            do {
                rand_point = Eigen::Vector3d(dis(rng), dis(rng), dis(rng));
            } while (rand_point.norm() > bounded_radius);
        }
        rand_point += root->point.head(3);
        // Box Check After Steering
        rrt_star::Node* nearest = nullptr;
        T.findNearestKD(rand_point, nearest);
        if (!nearest) {
            continue;
        }
        std::unique_ptr<rrt_star::Node> nn;
        T.steer_parent(nearest, rand_point, A.step_size, nn, false, A.min_edge);
        if (!nn) {
            ++rj_steer;
            continue;
        }
        const Eigen::Vector4d& q = nn->point;
        if (q.x() < A.min_x || q.x() > A.max_x || q.y() < A.min_y || q.y() > A.max_y ||
            q.z() < A.box_min_z || q.z() > A.box_max_z) {
            ++rj_box;
            continue;
        }
        if (clearance(q.head(3)) < A.uav_radius) {
            ++rj_clear;
            continue;
        }
        if (!T.getNodes().empty()) {
            const Eigen::Vector3d a = nearest->point.head(3), b = q.head(3);
            const Eigen::Vector3d d = b - a;
            const int ns = std::max(1, (int)std::ceil(d.norm() / 0.1));
            bool ok = true;
            for (int i = 0; i <= ns && ok; ++i) {
                if (clearance(a + d * ((double)i / ns)) < A.uav_radius) {
                    ok = false;
                }
            }
            if (!ok) {
                ++rj_edge;
                continue;
            }
        }
        nn->gain = 0.0;
        nn->score = 0.0;
        ev.computeCost(nn.get());
        T.addKDTreeNode(std::move(nn));
        ++added;
    }
    ROS_INFO("[video_export] tree: %d nodes from %d samples | rejected: steer %ld, box %ld, clearance %ld, edge %ld",
             added, tries, rj_steer, rj_box, rj_clear, rj_edge);

    std::vector<rrt_star::Node*> nodes;
    for (const auto& up : T.getNodes()) {
        if (up.get() != root) {
            nodes.push_back(up.get());
        }
    }

    /*                   GAINS                   */
    GainEvaluator::GainConfig cfg;
    cfg.marginal_gain = true;
    cfg.optimize_yaw = true;
    cfg.eval_compute = "gpu";
    cfg.marginal_split = false;
    cfg.track_absolute = true;
    ROS_INFO("[video_export] evaluating gains (marginal + absolute)...");
    float marg_ms = 0.0f, abs_ms = 0.0f;
    ev.evaluateGains(nodes, flat, cfg, marg_ms, abs_ms);
    ROS_INFO("[video_export] GPU kernel: marginal %.2f ms, absolute %.2f ms", marg_ms, abs_ms);

    // Proposed Marginal Gain
    std::map<rrt_star::Node*, double> g_all_db;
    for (rrt_star::Node* n : nodes) {
        g_all_db[n] = n->gain;
    }

    // Single Parent Gain
    std::map<rrt_star::Node*, double> g_sp_db;
    {
        ev.computeMarginalGains(nodes, /*optimize_yaw=*/false, /*one_parent_only=*/true);
        for (rrt_star::Node* n : nodes) {
            g_sp_db[n] = n->gain;
        }
        for (rrt_star::Node* n : nodes) {
            n->gain = g_all_db[n];
        }
    }

    // Absolute Gain
    std::map<rrt_star::Node*, double> g_abs_db;
    {
        std::vector<double> px, py, pz;
        std::vector<float> fy;
        px.reserve(nodes.size());
        py.reserve(nodes.size());
        pz.reserve(nodes.size());
        fy.reserve(nodes.size());
        for (rrt_star::Node* n : nodes) {
            px.push_back(n->point[0]);
            py.push_back(n->point[1]);
            pz.push_back(n->point[2]);
            fy.push_back((float)n->point[3]);
        }
        const auto res = ev.computeGainBatchGPU(px, py, pz, &fy, nullptr);
        for (size_t i = 0; i < nodes.size(); ++i) {
            g_abs_db[nodes[i]] = res[i].first;
        }
    }

    // Single Parent at Same Yaw
    std::vector<rrt_star::Node*> by_depth = nodes;
    rrt_star::sortByDepth(by_depth);
    // Four Gains at the Selected Yaw
    std::map<rrt_star::Node*, double> g_sp_cpu, g_abs_cpu, g_exact;
    // Commit Ancestors Shallow First
    for (rrt_star::Node* n : by_depth) {
        g_sp_cpu[n] = ev.computeMarginalGainCPU_AllAncestors(flat, n, n->point[3], true, false).first;
        g_exact[n] = ev.computeMarginalGainCPU_AllAncestors(flat, n, n->point[3], false, true).first;
        // Absolute at Same Yaw
        g_abs_cpu[n] = ev.computeGainCPU_FlatMap(flat, n->point, n->point[3]).first;
    }

    // Node Ids
    std::map<rrt_star::Node*, int> id;
    {
        int k = 0;
        id[root] = k++;
        for (rrt_star::Node* n : nodes) {
            id[n] = k++;
        }
    }
    auto id_of_tmp = [&](rrt_star::Node* n) { return id[n]; };

    /*              BRANCH SELECTION             */
    struct Cand {
        rrt_star::Node* n;
        double nonimm, total, g_all, g_sp, g_abs;
        int depth;
        bool wall;
        int wall_anc;
        double hitfrac, parenthit, minbranchhit, minspacing;
    };
    std::vector<Cand> cands;

    // Depth Buffer Cache
    std::map<rrt_star::Node*, double> hitcache;
    const double FX = (p_w / 2.0) / std::tan(1.51844 * 0.5);
    const double FY = (p_h / 2.0) / std::tan(1.01229 * 0.5);
    auto hitOf = [&](rrt_star::Node* nd) -> double {
        auto it = hitcache.find(nd);
        if (it != hitcache.end()) {
            return it->second;
        }
        const std::vector<float> db = ev.computeDepthBufferCPU(
            nd->point, flat, ev.parentCamRows((float)nd->point[3]));
        const double h = hitFraction(db, p_w, p_h, FX, FY, p_w / 2.0, p_h / 2.0, 5.0);
        hitcache[nd] = h;
        return h;
    };
    for (rrt_star::Node* n : nodes) {
        const int d = depthOf(n);
        // Branch Depth Bounds
        if (d < std::max(2, A.min_depth) || d > A.max_depth) {
            continue;
        }
        const double ga = g_all_db[n], gb = g_abs_db[n], gs = g_sp_db[n];
        if (gb <= 0.0) {
            continue;
        }
        // Wall Behind an Ancestor
        const std::vector<rrt_star::Node*> anc = ancestorsOf(n);
        bool wall = false;
        int which = -1;
        for (size_t k = 1; k < std::min<size_t>(4, anc.size()); ++k) {
            if (wallBetween(grid, n->point.head(3), anc[k]->point.head(3))) {
                wall = true;
                which = (int)k + 1;
                break;
            }
        }
        // Every Viewpoint Shows Structure
        const double hf = hitOf(n);
        const double hp = hitOf(n->parent);
        double hmin = std::min(hf, hp);
        for (rrt_star::Node* a : anc) {
            hmin = std::min(hmin, hitOf(a));
        }
        // Viewpoint Spacing
        double sp_min = 1e9;
        for (rrt_star::Node* q = n; q && q->parent; q = q->parent) {
            sp_min = std::min(sp_min, (q->point.head(3) - q->parent->point.head(3)).norm());
        }
        cands.push_back({n, gs - ga, gb - ga, ga, gs, gb, d, wall, which, hf, hp, hmin, sp_min});
    }
    // Rank by Non-Immediate Overlap
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
        return a.nonimm > b.nonimm;
    });

    ROS_INFO("[video_export] candidates restricted to depth in [%d,%d]", A.min_depth, A.max_depth);
    ROS_INFO("[video_export] top branches (nonimm = g_sp - g_exact = what SINGLE-PARENT still overcounts):");
    for (size_t i = 0; i < std::min<size_t>(15, cands.size()); ++i) {
        const Cand& c = cands[i];
        ROS_INFO("  #%zu depth=%2d wall=%d(anc-%d)  g_abs=%7.3f g_sp=%7.3f g_all=%7.3f | nonimm=%7.3f total=%7.3f  cand=%5.1f%% par=%5.1f%% min=%5.1f%% sp=%4.2fm  @(%6.2f,%6.2f,%5.2f)",
                 i, c.depth, (int)c.wall, c.wall_anc, c.g_abs, c.g_sp, c.g_all, c.nonimm, c.total,
                 100.0 * c.hitfrac, 100.0 * c.parenthit, 100.0 * c.minbranchhit, c.minspacing,
                 c.n->point[0], c.n->point[1], c.n->point[2]);
    }
    {
        std::string mk0 = "mkdir -p " + A.out_dir;
        if (system(mk0.c_str())) {
        }
        std::ofstream cf(A.out_dir + "/candidates.csv");
        cf << "tag,id,depth,x,y,z,yaw,g_abs,g_sp,g_all,drop_abs,drop_nonimm,"
              "cand_hit,parent_hit,min_branch_hit,min_spacing,wall,wall_anc\n";
        for (const Cand& c : cands) {
            cf << A.tag << ',' << id_of_tmp(c.n) << ',' << c.depth << ','
               << c.n->point[0] << ',' << c.n->point[1] << ',' << c.n->point[2] << ','
               << c.n->point[3] << ',' << c.g_abs << ',' << c.g_sp << ',' << c.g_all << ','
               << c.total << ',' << c.nonimm << ',' << c.hitfrac << ',' << c.parenthit << ','
               << c.minbranchhit << ',' << c.minspacing << ',' << (c.wall ? 1 : 0) << ','
               << c.wall_anc << '\n';
        }
        ROS_INFO("[video_export] wrote %zu candidates to candidates.csv", cands.size());
    }

    if (cands.empty()) {
        ROS_ERROR("no candidate with depth>=2");
        return 1;
    }
    // Prefer a Wall
    auto renderable = [&](const Cand& c) {
        return c.hitfrac >= A.min_hit && c.hitfrac <= A.max_hit &&
               c.parenthit >= A.min_parent_hit && c.minbranchhit >= A.min_branch_hit &&
               c.minspacing >= A.min_spacing;
    };
    size_t pick = SIZE_MAX;
    if (A.select_id >= 0) {
        for (size_t i = 0; i < cands.size(); ++i) {
            if (id[cands[i].n] == A.select_id) {
                pick = i;
                break;
            }
        }
        if (pick == SIZE_MAX) {
            ROS_ERROR("[video_export] --selectid %d not among candidates", A.select_id);
            return 4;
        }
        ROS_INFO("[video_export] forced --selectid %d", A.select_id);
    }
    for (size_t i = 0; pick == SIZE_MAX && i < cands.size(); ++i) {
        if (renderable(cands[i]) && cands[i].wall) {
            pick = i;
            break;
        }
    }
    if (pick == SIZE_MAX) {
        for (size_t i = 0; i < cands.size(); ++i) {
            if (renderable(cands[i])) {
                pick = i;
                break;
            }
        }
    }
    if (pick == SIZE_MAX) {
        ROS_WARN(
            "[video_export] NO branch satisfies cand hits in [%.0f,%.0f]%%, parent >=%.0f%%, "
            "every viewpoint >=%.0f%%. Relax the thresholds or try another map/run -- NOT "
            "falling back silently.",
            100 * A.min_hit, 100 * A.max_hit,
            100 * A.min_parent_hit, 100 * A.min_branch_hit);
        return 3;
    }
    const Cand& best = cands[pick];
    ROS_INFO("[video_export] SELECTED #%zu: depth=%d wall=%d(anc-%d) nonimm=%.3f cand_hits=%.1f%% parent_hits=%.1f%% min_branch=%.1f%%  %s",
             pick, best.depth, (int)best.wall, best.wall_anc, best.nonimm,
             100.0 * best.hitfrac, 100.0 * best.parenthit, 100.0 * best.minbranchhit,
             best.wall ? "grandparent-overlap ACROSS A WALL (Fig. 3 case)"
                       : "grandparent overlap, NO wall found (fallback)");

    /*                   EXPORT                  */
    const std::string O = A.out_dir;
    std::string mk = "mkdir -p " + O;
    if (system(mk.c_str())) {
    }

    std::vector<rrt_star::Node*> branch;
    for (rrt_star::Node* p = best.n; p; p = p->parent) {
        branch.push_back(p);
    }
    std::reverse(branch.begin(), branch.end());

    {
        std::ofstream f(O + "/tree.csv");
        f << "id,parent,x,y,z,yaw,depth,g_abs,g_sp,g_all,g_exact_cpu,on_branch\n";
        auto row = [&](rrt_star::Node* n) {
            const bool ob = std::find(branch.begin(), branch.end(), n) != branch.end();
            f << id[n] << ',' << (n->parent ? id[n->parent] : -1) << ','
              << n->point[0] << ',' << n->point[1] << ',' << n->point[2] << ',' << n->point[3] << ','
              << depthOf(n) << ',' << (n == root ? 0.0 : g_abs_db[n]) << ','
              << (n == root ? 0.0 : g_sp_db[n]) << ',' << (n == root ? 0.0 : g_all_db[n]) << ','
              << (n == root ? 0.0 : g_exact[n]) << ',' << (ob ? 1 : 0) << '\n';
        };
        row(root);
        for (rrt_star::Node* n : nodes) {
            row(n);
        }
    }

    // Depth Buffers and Voxels
    {
        std::ofstream f(O + "/branch.csv");
        f << "order,id,x,y,z,yaw,g_abs,g_sp,g_all,g_exact_cpu,depth_buffer\n";
        for (size_t i = 0; i < branch.size(); ++i) {
            rrt_star::Node* n = branch[i];
            const std::vector<float> R = ev.parentCamRows((float)n->point[3]);
            const std::vector<float> db = ev.computeDepthBufferCPU(n->point, flat, R);
            char name[64];
            snprintf(name, sizeof(name), "depth_%02zu.f32", i);
            writeFloats(O + "/" + name, db);
            std::ofstream rf(O + "/" + std::string(name).substr(0, 8) + "_R.txt");
            for (float v : R) {
                rf << v << '\n';
            }
            f << i << ',' << id[n] << ',' << n->point[0] << ',' << n->point[1] << ','
              << n->point[2] << ',' << n->point[3] << ','
              << (n == root ? 0.0 : g_abs_db[n]) << ',' << (n == root ? 0.0 : g_sp_db[n]) << ','
              << (n == root ? 0.0 : g_all_db[n]) << ',' << (n == root ? 0.0 : g_exact[n]) << ','
              << name << '\n';

            // Exact Observed Voxels
            {
                std::vector<uint64_t> keys;
                keys.reserve(n->observed_unknown_voxels.size());
                for (const auto& kv : n->observed_unknown_voxels) {
                    keys.push_back(kv.first);
                }
                char on[64];
                snprintf(on, sizeof(on), "obs_%02zu.u64", i);
                std::ofstream of(O + "/" + on, std::ios::binary);
                of.write(reinterpret_cast<const char*>(keys.data()),
                         (std::streamsize)(keys.size() * sizeof(uint64_t)));
                ROS_INFO("[video_export]   node %zu observed %zu voxels (%.3f m3)",
                         i, keys.size(), keys.size() * ev.getVoxelSize() * ev.getVoxelSize() * ev.getVoxelSize());
            }

            voxblox::Pointcloud vox;
            ev.visualizeGain(n->point, vox);
            std::vector<float> flatv;
            flatv.reserve(vox.size() * 3);
            for (const auto& p : vox) {
                flatv.push_back(p.x());
                flatv.push_back(p.y());
                flatv.push_back(p.z());
            }
            char vn[64];
            snprintf(vn, sizeof(vn), "voxels_%02zu.f32", i);
            writeFloats(O + "/" + vn, flatv);
        }
    }

    // Local Map Slab
    {
        double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
        for (rrt_star::Node* n : branch) {
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], n->point[k]);
                hi[k] = std::max(hi[k], n->point[k]);
            }
        }
        const double pad = 6.0;
        std::ofstream fo(O + "/map_occupied.f32", std::ios::binary), fu(O + "/map_unknown.f32", std::ios::binary);
        size_t no = 0, nu = 0;
        for (int z = 0; z < dim.z(); ++z) {
            for (int y = 0; y < dim.y(); ++y) {
                for (int x = 0; x < dim.x(); ++x) {
                    const uint8_t c = grid.at(x, y, z);
                    if (c == V_FREE) {
                        continue;
                    }
                    const Eigen::Vector3d p = grid.centre(x, y, z);
                    if (p.x() < lo[0] - pad || p.x() > hi[0] + pad ||
                        p.y() < lo[1] - pad || p.y() > hi[1] + pad ||
                        p.z() < lo[2] - pad || p.z() > hi[2] + pad) {
                        continue;
                    }
                    const float b[3] = {(float)p.x(), (float)p.y(), (float)p.z()};
                    if (c == V_OCCUPIED) {
                        fo.write((const char*)b, 12);
                        ++no;
                    } else {
                        fu.write((const char*)b, 12);
                        ++nu;
                    }
                }
            }
        }
        ROS_INFO("[video_export] local slab: %zu occupied, %zu unknown voxels", no, nu);
    }

    // Top-Down Map Projection
    {
        std::ofstream f(O + "/topdown.u8", std::ios::binary);
        const int z0 = std::max(0, (int)std::floor((0.4 - origin.z()) / grid.vs));
        const int z1 = std::min(dim.z() - 1, (int)std::ceil((2.0 - origin.z()) / grid.vs));
        std::vector<uint8_t> col((size_t)dim.x() * dim.y(), V_UNKNOWN);
        for (int y = 0; y < dim.y(); ++y) {
            for (int x = 0; x < dim.x(); ++x) {
                bool occ = false, fre = false;
                for (int z = z0; z <= z1; ++z) {
                    const uint8_t c = grid.at(x, y, z);
                    if (c == V_OCCUPIED) {
                        occ = true;
                        break;
                    }
                    if (c == V_FREE) {
                        fre = true;
                    }
                }
                col[(size_t)y * dim.x() + x] = occ ? V_OCCUPIED : (fre ? V_FREE : V_UNKNOWN);
            }
        }
        f.write((const char*)col.data(), (std::streamsize)col.size());
        ROS_INFO("[video_export] topdown %dx%d written (z band %d..%d)", dim.x(), dim.y(), z0, z1);
    }

    {
        std::ofstream f(O + "/scenario.json");
        const double fx = (p_w / 2.0) / std::tan(1.51844 * 0.5);
        const double fy = (p_h / 2.0) / std::tan(1.01229 * 0.5);
        f << "{\n"
          << "  \"map\": \"" << A.map_path << "\",\n"
          << "  \"root_pose\": [" << A.rx << ", " << A.ry << ", " << A.rz << ", " << A.ryaw << "],\n"
          << "  \"voxel_size\": " << ev.getVoxelSize() << ",\n"
          << "  \"grid_origin\": [" << origin.x() << ", " << origin.y() << ", " << origin.z() << "],\n"
          << "  \"grid_dim\": [" << dim.x() << ", " << dim.y() << ", " << dim.z() << "],\n"
          << "  \"depth_buffer\": {\"width\": " << p_w << ", \"height\": " << p_h
          << ", \"rays\": " << p_w * p_h << ", \"fx\": " << fx << ", \"fy\": " << fy
          << ", \"cx\": " << p_w / 2.0 << ", \"cy\": " << p_h / 2.0 << "},\n"
          << "  \"sensor\": {\"h_fov\": 1.51844, \"v_fov\": 1.01229, \"r_max\": 5.0, \"pitch_deg\": 10.0},\n"
          << "  \"tree\": {\"nodes\": " << added << ", \"seed\": " << A.seed
          << ", \"step\": " << A.step_size << ", \"uav_radius\": " << A.uav_radius << "},\n"
          << "  \"selected\": {\"id\": " << id[best.n] << ", \"depth\": " << best.depth
          << ", \"wall_between_candidate_and_nonimmediate_ancestor\": " << (best.wall ? "true" : "false")
          << ", \"g_abs\": " << best.g_abs << ", \"g_sp\": " << best.g_sp
          << ", \"g_all\": " << best.g_all << ", \"nonimmediate_overcount\": " << best.nonimm
          << ", \"branch_len\": " << branch.size() << "}\n"
          << "}\n";
    }

    int bad = 0;
    for (rrt_star::Node* n : nodes) {
        if (!(g_all_db[n] <= g_sp_db[n] + 1e-3 && g_sp_db[n] <= g_abs_db[n] + 1e-3)) {
            ++bad;
        }
    }
    ROS_INFO("[video_export] invariant g_all(depth-buffer) <= g_sp <= g_abs violated on %d / %zu nodes", bad, nodes.size());
    double ratio_agg = 0.0, ratio_med = 0.0;
    int ratio_n = 0;
    {
        double sp = 0.0, se = 0.0;
        std::vector<double> r;
        for (rrt_star::Node* n : nodes) {
            sp += g_all_db[n];
            se += g_exact[n];
            if (g_exact[n] > 0.5) {
                r.push_back(g_all_db[n] / g_exact[n]);
            }
        }
        std::sort(r.begin(), r.end());
        const double med = r.empty() ? 0.0 : r[r.size() / 2];
        ratio_agg = se > 0 ? sp / se : 0.0;
        ratio_med = med;
        ratio_n = (int)r.size();
        ROS_INFO("[video_export] proposed vs exact: aggregate sum ratio = %.4f | median per-node ratio = %.4f (n=%zu, g_exact>0.5)",
                 ratio_agg, ratio_med, r.size());
    }
    {
        std::ofstream f(O + "/validation.json");
        f << "{\"proposed_vs_exact_aggregate\": " << ratio_agg
          << ", \"proposed_vs_exact_median\": " << ratio_med
          << ", \"n_nodes\": " << ratio_n
          << ", \"invariant_violations\": " << bad
          << ", \"tree_nodes\": " << added << "}\n";
    }
    ROS_INFO("[video_export] wrote %s", O.c_str());
    return 0;
}
