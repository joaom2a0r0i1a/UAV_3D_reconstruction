#include "motion_planning/planner_helpers.h"

#include <ros/ros.h>
#include <mrs_lib/geometry/misc.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <unordered_map>
#include <cstdlib>

namespace planner_helpers {

using vec3_t = mrs_lib::geometry::vec_t<3>;

double getMapDistance(const voxblox::EsdfServer& server, const Eigen::Vector3d& position) {
    if (!server.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!server.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool isPathCollisionFree(const voxblox::EsdfServer& server, const std::vector<rrt_star::Node*>& path, double uav_radius) {
    for (rrt_star::Node* node : path) {
        if (getMapDistance(server, node->point.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

bool isEdgeCollisionFree(const voxblox::EsdfServer& server, const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         double uav_radius, double resolution, bool optimistic_edges) {
    const Eigen::Vector3d d = to - from;
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / resolution)));
    for (int i = 0; i <= n; ++i) {
        const Eigen::Vector3d p = from + d * (static_cast<double>(i) / n);
        if (optimistic_edges) {
            const auto esdf = server.getEsdfMapPtr();
            double dist = 0.0;
            if (esdf && esdf->getDistanceAtPosition(p, &dist) && dist < uav_radius) return false;
        } else if (getMapDistance(server, p) < uav_radius) {
            return false;   // unobserved (0.0) or too close to a mapped obstacle
        }
    }
    return true;
}

double distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {
    return mrs_lib::geometry::dist(vec3_t(waypoint->position.x, waypoint->position.y, waypoint->position.z),
                                   vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

std::vector<rrt_star::Node*> collectTreeNodes(rrt_star& tree) {
    std::vector<rrt_star::Node*> nodes;
    const auto& all = tree.getNodes();
    nodes.reserve(all.size());
    for (const auto& up : all) if (up->parent) nodes.push_back(up.get());   // skip root
    return nodes;
}

bool inBoundingBox(const Eigen::Vector4d& p, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
    return p[0] >= min_x && p[0] <= max_x &&
           p[1] >= min_y && p[1] <= max_y &&
           p[2] >= min_z && p[2] <= max_z;
}

void logTreeNodes(rrt_star& tree, double lambda) {
    for (const auto& up : tree.getNodes())
        if (up->parent)
            ROS_INFO("[Node] gain=%.3f score_contribution=%.3f score=%.3f",
                     up->gain, up->gain * exp(-lambda * up->cost), up->score);
}

void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    const std::vector<rrt_star::Node*>& nodes) {
    visualization_msgs::Marker edges;
    edges.header.stamp = ros::Time::now();
    edges.header.frame_id = ns + "/" + frame_id;
    edges.ns = "tree_branches";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.06;
    edges.color.r = 1.0; edges.color.g = 0.3; edges.color.b = 0.7; edges.color.a = 1.0;
    edges.lifetime = ros::Duration(30.0);

    visualization_msgs::Marker pts;
    pts.header = edges.header;
    pts.ns = "nodes";
    pts.id = 0;
    pts.type = visualization_msgs::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = pts.scale.y = pts.scale.z = 0.2;
    pts.color.r = 0.4; pts.color.g = 0.7; pts.color.b = 0.2; pts.color.a = 1.0;
    pts.lifetime = ros::Duration(30.0);

    for (rrt_star::Node* node : nodes) {
        geometry_msgs::Point p;
        p.x = node->point[0]; p.y = node->point[1]; p.z = node->point[2];
        pts.points.push_back(p);
        if (node->parent) {
            geometry_msgs::Point pp;
            pp.x = node->parent->point[0]; pp.y = node->parent->point[1]; pp.z = node->parent->point[2];
            edges.points.push_back(pp);
            edges.points.push_back(p);
        }
    }

    if (!nodes.empty()) {
        rrt_star::Node* root = nodes[0];
        while (root->parent) root = root->parent;
        geometry_msgs::Point rp;
        rp.x = root->point[0]; rp.y = root->point[1]; rp.z = root->point[2];
        pts.points.push_back(rp);
    }
    pub_markers.publish(edges);
    pub_markers.publish(pts);
}

void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    rrt_star::Node* node, int& path_id_counter) {
    rrt_star::Node* current = node;

    while (current->parent) {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = path_id_counter;
        p.header.frame_id = ns + "/" + frame_id;
        p.id = path_id_counter;
        p.ns = "path";
        p.type = visualization_msgs::Marker::ARROW;
        p.action = visualization_msgs::Marker::ADD;
        p.pose.position.x = current->parent->point[0];
        p.pose.position.y = current->parent->point[1];
        p.pose.position.z = current->parent->point[2];

        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        Eigen::Vector3d dir(current->point[0] - current->parent->point[0],
                            current->point[1] - current->parent->point[1],
                            current->point[2] - current->parent->point[2]);
        q.setFromTwoVectors(init, dir);
        q.normalize();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();

        p.scale.x = dir.norm();
        p.scale.y = 0.07;
        p.scale.z = 0.07;

        p.color.r = 0.7;
        p.color.g = 0.7;
        p.color.b = 0.3;
        p.color.a = 1.0;

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter++;
    }
}


void clear_all_voxels(ros::Publisher& pub_voxels) {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void clearMarkers(ros::Publisher& pub_markers, int& node_id_counter, int& edge_id_counter, int& path_id_counter) {
    visualization_msgs::Marker clear_nodes;
    clear_nodes.header.stamp = ros::Time::now();
    clear_nodes.ns = "nodes";
    clear_nodes.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_nodes);

    visualization_msgs::Marker clear_edges;
    clear_edges.header.stamp = ros::Time::now();
    clear_edges.ns = "tree_branches";
    clear_edges.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_edges);

    visualization_msgs::Marker clear_path;
    clear_path.header.stamp = ros::Time::now();
    clear_path.ns = "path";
    clear_path.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_path);

    node_id_counter = 0;
    edge_id_counter = 0;
    path_id_counter = 0;
}

// Nodes shallow-first + the tree root, so committed ancestor views subtract in order (X1/X2 CPU marginal).
namespace {
struct DepthContext {
    std::vector<rrt_star::Node*> depth_nodes;
    rrt_star::Node* root = nullptr;
};
DepthContext makeDepthContext(const std::vector<rrt_star::Node*>& nodes) {
    DepthContext c;
    c.depth_nodes = nodes;
    rrt_star::sortByDepth(c.depth_nodes);
    c.root = c.depth_nodes.empty() ? nullptr : c.depth_nodes.front();
    while (c.root && c.root->parent) c.root = c.root->parent;
    return c;
}
void clearObserved(const DepthContext& c) {
    if (c.root) c.root->observed_unknown_voxels.clear();
    for (rrt_star::Node* nd : c.depth_nodes) nd->observed_unknown_voxels.clear();
}
}  // namespace

// [correctness] Batched-pool marginal gain vs the independent layered reference. Yaw-optimizing; self-contained.
void benchmarkGpuCorrectness(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                             bool optimize_yaw, bool marginal_split, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();
    std::vector<double> sg(n), sy(n);
    for (size_t i = 0; i < n; ++i) { sg[i] = nodes[i]->gain; sy[i] = nodes[i]->point[3]; }
    long ref_flips = 0;
    auto d = seg.checkMarginalBatchedAgainstReference(nodes, optimize_yaw, marginal_split, ref_flips);   // {max|dGain|, max|dYaw|}
    ROS_INFO("[bench_correctness] phase=%s nodes=%zu max|dGain|=%.3e yaw_flips=%ld max|dYaw|=%.4f",
             phase, n, d.first, ref_flips, d.second);
    for (size_t i = 0; i < n; ++i) { nodes[i]->gain = sg[i]; nodes[i]->point[3] = sy[i]; }
}

// [X1] Per-node gain VALUES: CPU {abs, 1-parent, all} + GPU {abs, all} -> NBV_X1_CSV for R². Accuracy, not time.
void benchmarkX1Accuracy(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                         const std::vector<uint8_t>& flat_map, bool optimize_yaw, int replan_count, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();

    std::vector<double> saved_gain(n), saved_yaw(n);
    for (size_t i = 0; i < n; ++i) { saved_gain[i] = nodes[i]->gain; saved_yaw[i] = nodes[i]->point[3]; }

    auto eval = [&](bool marginal, const std::string& compute) {
        GainEvaluator::GainConfig cfg{marginal, optimize_yaw, compute, false, /*track_absolute=*/false};
        float m0 = 0.0f, m1 = 0.0f;
        seg.evaluateGains(nodes, flat_map, cfg, m0, m1);
    };

    // GPU G_all FIRST: production point[3]/pool still intact (batched G_all breaks if ancestor yaws change).
    float k = 0.0f;
    seg.computeMarginalGainsBatched(nodes, optimize_yaw, /*marginal_split=*/false, k);
    std::vector<double> g_gall_gpu(n);
    for (size_t i = 0; i < n; ++i) { g_gall_gpu[i] = nodes[i]->gain; nodes[i]->gain = saved_gain[i]; nodes[i]->point[3] = saved_yaw[i]; }

    // GPU absolute.
    eval(false, "gpu");
    std::vector<double> g_abs_gpu(n);
    for (size_t i = 0; i < n; ++i) { g_abs_gpu[i] = nodes[i]->gain; nodes[i]->gain = saved_gain[i]; nodes[i]->point[3] = saved_yaw[i]; }

    // GPU 1-parent (host pool, does not touch the GPU G_all pool).
    seg.computeMarginalGains(nodes, optimize_yaw, /*one_parent_only=*/true);
    std::vector<double> g_g1p_gpu(n);
    for (size_t i = 0; i < n; ++i) { g_g1p_gpu[i] = nodes[i]->gain; nodes[i]->gain = saved_gain[i]; nodes[i]->point[3] = saved_yaw[i]; }

    // CPU values: absolute, then depth-sequential 1-parent and all-ancestors.
    eval(false, "cpu");
    std::vector<double> g_abs_cpu(n);
    for (size_t i = 0; i < n; ++i) g_abs_cpu[i] = nodes[i]->gain;

    DepthContext dc = makeDepthContext(nodes);
    std::unordered_map<rrt_star::Node*, double> g1p_of, gall_of;
    clearObserved(dc);
    for (rrt_star::Node* nd : dc.depth_nodes)
        g1p_of[nd] = seg.computeMarginalGainCPU_AllAncestors(flat_map, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/true, /*commit_observed=*/true).first;
    clearObserved(dc);
    for (rrt_star::Node* nd : dc.depth_nodes)
        gall_of[nd] = seg.computeMarginalGainCPU_AllAncestors(flat_map, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/false, /*commit_observed=*/true).first;
    std::vector<double> g_g1p_cpu(n), g_gall_cpu(n);
    for (size_t i = 0; i < n; ++i) { g_g1p_cpu[i] = g1p_of[nodes[i]]; g_gall_cpu[i] = gall_of[nodes[i]]; }

    // Per-node CSV: replan,depth,abs_cpu,abs_gpu,p1_cpu,p1_gpu,all_cpu,all_gpu.
    const char* x1_csv_path = std::getenv("NBV_X1_CSV");
    std::ofstream x1;
    if (x1_csv_path) x1.open(x1_csv_path, std::ios::app);
    for (size_t i = 0; i < n; ++i) {
        rrt_star::Node* nd = nodes[i];
        int depth = 0; for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) ++depth;
        if (x1.is_open())
            x1 << replan_count << ',' << depth << ',' << g_abs_cpu[i] << ',' << g_abs_gpu[i] << ','
               << g_g1p_cpu[i] << ',' << g_g1p_gpu[i] << ',' << g_gall_cpu[i] << ',' << g_gall_gpu[i] << '\n';
        else
            ROS_INFO("[X1][%s] abs c/g=%7.3f/%7.3f | p1 c/g=%7.3f/%7.3f | all c/g=%7.3f/%7.3f",
                     phase, g_abs_cpu[i], g_abs_gpu[i], g_g1p_cpu[i], g_g1p_gpu[i], g_gall_cpu[i], g_gall_gpu[i]);
    }

    // Restore production gain+yaw; the GPU G_all pass (run first) already left the pool consistent with point[3].
    for (size_t i = 0; i < n; ++i) { nodes[i]->gain = saved_gain[i]; nodes[i]->point[3] = saved_yaw[i]; }
}

// [X2] Cost: absolute + G_all only, CPU and GPU, timed on the SAME tree. Self-contained (one restore+resync at the end).
void benchmarkX2Timing(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                       const std::vector<uint8_t>& flat_map, BenchAccum& acc,
                       bool optimize_yaw, bool marginal_split, int replan_count, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();
    (void)replan_count;

    std::vector<double> saved_gain(n), saved_yaw(n);
    for (size_t i = 0; i < n; ++i) { saved_gain[i] = nodes[i]->gain; saved_yaw[i] = nodes[i]->point[3]; }

    float last_marg_ms = 0.0f, last_abs_ms = 0.0f;
    auto time_eval = [&](bool marginal, const std::string& compute, bool split) {
        GainEvaluator::GainConfig cfg{marginal, optimize_yaw, compute, split, /*track_absolute=*/false};
        auto t0 = std::chrono::high_resolution_clock::now();
        seg.evaluateGains(nodes, flat_map, cfg, last_marg_ms, last_abs_ms);
        return std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0).count();
    };

    // GPU: both formulations, compute/transfer split (transfer = total - kernel).
    double t_gall_gpu = time_eval(true,  "gpu", marginal_split);
    double k_gall     = last_marg_ms;
    double t_abs_gpu  = time_eval(false, "gpu", false);
    double k_abs      = last_abs_ms;

    // CPU absolute (own-view).
    double t_abs_cpu = time_eval(false, "cpu", false);

    // CPU G_all (depth-sequential all-ancestors). Restore gain first: the timed passes overwrote it.
    for (size_t i = 0; i < n; ++i) nodes[i]->gain = saved_gain[i];
    DepthContext dc = makeDepthContext(nodes);
    clearObserved(dc);
    auto t0_gall = std::chrono::high_resolution_clock::now();
    for (rrt_star::Node* nd : dc.depth_nodes)
        seg.computeMarginalGainCPU_AllAncestors(flat_map, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/false, /*commit_observed=*/true);
    double t_gall_cpu = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0_gall).count();

    acc.ms_gall_gpu += t_gall_gpu; acc.kernel_gall_gpu += k_gall;
    acc.ms_abs_gpu  += t_abs_gpu;  acc.kernel_abs_gpu  += k_abs;
    acc.ms_abs_cpu  += t_abs_cpu;  acc.ms_gall_cpu     += t_gall_cpu;
    acc.nodes += (int)n;

    ROS_INFO("[X2rep] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_gall_gpu, k_gall, t_gall_gpu - k_gall);
    ROS_INFO("[X2repABS] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_abs_gpu, k_abs, t_abs_gpu - k_abs);
    ROS_INFO("[X2cpu] nodes=%zu cpu_absolute_ms=%.3f cpu_gain_all_ms=%.3f", n, t_abs_cpu, t_gall_cpu);

    // Restore production gain+yaw; the GPU G_all pass (run first) already left the pool consistent with point[3].
    for (size_t i = 0; i < n; ++i) { nodes[i]->gain = saved_gain[i]; nodes[i]->point[3] = saved_yaw[i]; }
}

// Run whichever suite(s) the comma-separated string names ("correctness"/"x1"/"x2").
void runBenchSuite(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                   const std::vector<uint8_t>& flat_map, BenchAccum& acc, const std::string& suite,
                   bool optimize_yaw, bool marginal_split, int replan_count, const char* phase) {
    if (nodes.empty()) return;
    if (suite.find("correctness") != std::string::npos) benchmarkGpuCorrectness(seg, nodes, optimize_yaw, marginal_split, phase);
    if (suite.find("x1") != std::string::npos)          benchmarkX1Accuracy(seg, nodes, flat_map, optimize_yaw, replan_count, phase);
    if (suite.find("x2") != std::string::npos)          benchmarkX2Timing(seg, nodes, flat_map, acc, optimize_yaw, marginal_split, replan_count, phase);
}

void logBenchSummary(const BenchAccum& a) {
    if (a.nodes <= 0) return;
    double bn = (double)a.nodes;
    ROS_INFO("\n=== X2 GAIN TIMING (%d nodes) ===\n"
             "GPU G_all : %9.3f ms | %7.4f ms/node total (kernel %9.3f ms | transfer %9.3f ms)\n"
             "GPU abs   : %9.3f ms | %7.4f ms/node total (kernel %9.3f ms | transfer %9.3f ms)\n"
             "CPU G_all : %9.3f ms | %7.4f ms/node\n"
             "CPU abs   : %9.3f ms | %7.4f ms/node\n"
             "==================================================",
             a.nodes,
             a.ms_gall_gpu, a.ms_gall_gpu/bn, a.kernel_gall_gpu, a.ms_gall_gpu - a.kernel_gall_gpu,
             a.ms_abs_gpu,  a.ms_abs_gpu/bn,  a.kernel_abs_gpu,  a.ms_abs_gpu  - a.kernel_abs_gpu,
             a.ms_gall_cpu, a.ms_gall_cpu/bn,
             a.ms_abs_cpu,  a.ms_abs_cpu/bn);
}

}  // namespace planner_helpers
