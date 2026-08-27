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

// Canonical (RH-NBVP) gain benchmark: time all variants, dump the X1 CSV, log X2 lines, check pool-vs-reference; restores gains, accumulates into acc.
void benchmarkGains(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                    const std::vector<uint8_t>& flat_map, BenchAccum& acc,
                    bool optimize_yaw, bool marginal_split, int replan_count, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();

    auto snapshot_gains = [&]() {
        std::vector<double> g(n);
        for (size_t i = 0; i < n; ++i) g[i] = nodes[i]->gain;
        return g;
    };
    const std::vector<double> saved_gain = snapshot_gains();

    // Correctness (benchmark-only): batched pool gain vs the independent layered reference. Snapshot gain+yaw so timing is undisturbed.
    {
        std::vector<double> sg(n), sy(n);
        for (size_t i = 0; i < n; ++i) { sg[i] = nodes[i]->gain; sy[i] = nodes[i]->point[3]; }
        long ref_flips = 0;
        double ref_maxd = seg.checkMarginalBatchedAgainstReference(nodes, optimize_yaw, marginal_split, ref_flips);
        ROS_INFO("[bench_correctness] batched-vs-reference nodes=%zu max|dGain|=%.3e yaw_flips=%ld", n, ref_maxd, ref_flips);
        for (size_t i = 0; i < n; ++i) { nodes[i]->gain = sg[i]; nodes[i]->point[3] = sy[i]; }
    }

    float last_marg_ms = 0.0f, last_abs_ms = 0.0f;
    auto time_eval = [&](bool marginal, const std::string& compute, bool split) {
        GainEvaluator::GainConfig cfg{marginal, optimize_yaw, compute, split, /*track_absolute=*/false};
        auto t0 = std::chrono::high_resolution_clock::now();
        seg.evaluateGains(nodes, flat_map, cfg, last_marg_ms, last_abs_ms);
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();
    };

    double t_gall_gpu        = time_eval(true,  "gpu", false);
    double t_gall_gpu_kernel = last_marg_ms;
    std::vector<double> g_gall_gpu = snapshot_gains();

    double t_gall_split = time_eval(true, "gpu", true);
    std::vector<double> g_gall_split = snapshot_gains();

    double t_g1p_cpu_hashmap = time_eval(true, "cpu", false);   // legacy HashMap pass, console summary only

    double t_abs_gpu        = time_eval(false, "gpu", false);
    double t_abs_gpu_kernel = last_abs_ms;
    std::vector<double> g_abs_gpu = snapshot_gains();

    double t_abs_cpu = time_eval(false, "cpu", false);
    std::vector<double> g_abs_cpu = snapshot_gains();

    acc.ms_gall_gpu   += t_gall_gpu;
    acc.ms_gall_split += t_gall_split;
    acc.ms_g1p_cpu    += t_g1p_cpu_hashmap;
    acc.ms_abs_gpu    += t_abs_gpu;
    acc.ms_abs_cpu    += t_abs_cpu;
    acc.kernel_gall_gpu += t_gall_gpu_kernel;
    acc.kernel_abs_gpu  += t_abs_gpu_kernel;
    acc.nodes += (int)n;

    ROS_INFO("[X2rep] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_gall_gpu, t_gall_gpu_kernel, t_gall_gpu - t_gall_gpu_kernel);
    ROS_INFO("[X2repABS] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_abs_gpu, t_abs_gpu_kernel, t_abs_gpu - t_abs_gpu_kernel);

    for (size_t i = 0; i < n; ++i) nodes[i]->gain = saved_gain[i];

    // CPU marginal baselines, depth-sequential so each node subtracts its ancestors' committed views.
    std::vector<rrt_star::Node*> depth_nodes = nodes;
    seg.sortByDepth(depth_nodes);
    rrt_star::Node* tree_root = depth_nodes.empty() ? nullptr : depth_nodes.front();
    while (tree_root && tree_root->parent) tree_root = tree_root->parent;

    auto clear_observed = [&]() {
        if (tree_root) tree_root->observed_unknown_voxels.clear();
        for (rrt_star::Node* nd : depth_nodes) nd->observed_unknown_voxels.clear();
    };

    clear_observed();
    auto t0_g1p = std::chrono::high_resolution_clock::now();
    for (rrt_star::Node* nd : depth_nodes)
        seg.computeMarginalGainCPU_AllAncestors(flat_map, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/true, /*commit_observed=*/true);
    double t_g1p_cpu = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0_g1p).count();

    std::unordered_map<rrt_star::Node*, double> gall_of;
    clear_observed();
    auto t0_gall = std::chrono::high_resolution_clock::now();
    for (rrt_star::Node* nd : depth_nodes)
        gall_of[nd] = seg.computeMarginalGainCPU_AllAncestors(flat_map, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/false, /*commit_observed=*/true).first;
    double t_gall_cpu = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0_gall).count();

    std::vector<double> g_gall_cpu(n);
    for (size_t i = 0; i < n; ++i) g_gall_cpu[i] = gall_of[nodes[i]];   // back to input order for the CSV

    ROS_INFO("[X2cpu] nodes=%zu cpu_absolute_ms=%.3f cpu_gain_1parent_ms=%.3f", n, t_abs_cpu, t_g1p_cpu);
    ROS_INFO("[X1cpu] nodes=%zu cpu_gain_all_ms=%.3f", n, t_gall_cpu);

    // Per-node dump. cols: replan,depth,abs_cpu,abs_gpu,p1_cpu,p1_gpu,all_cpu,all_gpu
    const char* x1_csv_path = std::getenv("NBV_X1_CSV");
    std::ofstream x1;
    if (x1_csv_path) x1.open(x1_csv_path, std::ios::app);
    for (size_t i = 0; i < n; ++i) {
        rrt_star::Node* nd = nodes[i];
        double ref_y; double g1p_gpu = seg.computeReferenceMarginalGain(nd, ref_y, /*one_parent_only=*/true, /*fixed_mode=*/!optimize_yaw);
        double saved = nd->gain;
        if (nd->parent && nd->parent->parent) seg.populateParentHistory(flat_map, nd->parent);
        double g1p_cpu = seg.computeMarginalGainCPU_HashMap(flat_map, nd, nd->point[3]).first;
        nd->gain = saved;

        double err = std::fabs(g1p_gpu - g1p_cpu);
        acc.g1p_err_sum += err;
        if (err > acc.g1p_err_max) acc.g1p_err_max = err;

        if (x1.is_open()) {
            int depth = 0; for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) ++depth;
            x1 << replan_count << ',' << depth << ',' << g_abs_cpu[i] << ',' << g_abs_gpu[i] << ','
               << g1p_cpu << ',' << g1p_gpu << ',' << g_gall_cpu[i] << ',' << g_gall_gpu[i] << '\n';
        } else {
            ROS_INFO("[BENCH][%s] gall_gpu=%7.3f split=%7.3f | g1p_gpu=%7.3f g1p_cpu=%7.3f err=%.4f | abs_gpu=%7.3f abs_cpu=%7.3f",
                     phase, g_gall_gpu[i], g_gall_split[i], g1p_gpu, g1p_cpu, err, g_abs_gpu[i], g_abs_cpu[i]);
        }
    }
}

void logBenchSummary(const BenchAccum& a) {
    if (a.nodes <= 0) return;
    double bn = (double)a.nodes;
    ROS_INFO("\n=== GAIN BENCHMARK (%d nodes) ===\n"
             "marginal-gpu-G_all : %9.3f ms | %7.4f ms/node  total\n"
             "  |- gain computation : %9.3f ms | %7.4f ms/node  (CPU->GPU transfer = %9.3f ms)\n"
             "marginal-gpu-split : %9.3f ms | %7.4f ms/node\n"
             "marginal-cpu-1parent : %9.3f ms | %7.4f ms/node\n"
             "absolute-gpu       : %9.3f ms | %7.4f ms/node  total\n"
             "  |- gain computation : %9.3f ms | %7.4f ms/node  (CPU->GPU transfer = %9.3f ms)\n"
             "absolute-cpu       : %9.3f ms | %7.4f ms/node\n"
             "g1p GPU-ref vs cpu-hash: mean err %.4f | max err %.4f\n"
             "==================================================",
             a.nodes,
             a.ms_gall_gpu, a.ms_gall_gpu/bn,
             a.kernel_gall_gpu, a.kernel_gall_gpu/bn, a.ms_gall_gpu - a.kernel_gall_gpu,
             a.ms_gall_split, a.ms_gall_split/bn,
             a.ms_g1p_cpu, a.ms_g1p_cpu/bn,
             a.ms_abs_gpu, a.ms_abs_gpu/bn,
             a.kernel_abs_gpu, a.kernel_abs_gpu/bn, a.ms_abs_gpu - a.kernel_abs_gpu,
             a.ms_abs_cpu, a.ms_abs_cpu/bn, a.g1p_err_sum/bn, a.g1p_err_max);
}

}  // namespace planner_helpers
