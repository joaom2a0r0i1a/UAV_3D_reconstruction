#ifndef MOTION_PLANNING_PLANNER_HELPERS_H
#define MOTION_PLANNING_PLANNER_HELPERS_H

// Stateless helpers shared by AEP and RH-NBVP (Plan 3, step 2). Free functions that take the
// needed state explicitly, so both planners' methods are thin wrappers over one implementation.

#include <vector>
#include <memory>
#include <string>
#include <cstdint>
#include <Eigen/Core>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>
#include <rrt_construction/rrt_star_kd.h>
#include <gain_evaluation/gain_evaluator.h>
#include <mrs_msgs/Reference.h>
#include <geometry_msgs/Pose.h>

namespace planner_helpers {

// ESDF clearance at a world position (0.0 if no ESDF or the point is unmapped).
double getMapDistance(const voxblox::EsdfServer& server, const Eigen::Vector3d& position);

// True iff every node on the path clears uav_radius.
bool isPathCollisionFree(const voxblox::EsdfServer& server, const std::vector<rrt_star::Node*>& path, double uav_radius);

// Sample the straight segment from->to at `resolution` spacing; require clearance >= uav_radius at each.
// optimistic_edges: treat unobserved space as free (bootstrap away from spawn) vs. blocking it.
bool isEdgeCollisionFree(const voxblox::EsdfServer& server, const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         double uav_radius, double resolution, bool optimistic_edges);

// Distance between a Reference waypoint and a Pose.
double distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose);

// All non-root nodes of the tree.
std::vector<rrt_star::Node*> collectTreeNodes(rrt_star& tree);

// True iff p is inside the axis-aligned bounded box.
bool inBoundingBox(const Eigen::Vector4d& p, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

// Log each non-root node's gain / score-contribution / score.
void logTreeNodes(rrt_star& tree, double lambda);

// --- RViz marker helpers. Publisher / frame / marker-namespace passed explicitly; id counters mutated in place. ---
void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    const std::vector<rrt_star::Node*>& nodes);
void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    rrt_star::Node* node, int& path_id_counter);
void clear_all_voxels(ros::Publisher& pub_voxels);
void clearMarkers(ros::Publisher& pub_markers, int& node_id_counter, int& edge_id_counter, int& path_id_counter);

// --- Gain benchmark: three suites (correctness | x1 | x2), selected by the benchmark/suite string; HIL runs x2. ---

// Per-cycle X2 timing accumulators (abs + G_all only); reset with `acc = {}` each planning cycle.
struct BenchAccum {
    double ms_gall_gpu = 0, ms_abs_gpu = 0, ms_abs_cpu = 0, ms_gall_cpu = 0;
    double kernel_gall_gpu = 0, kernel_abs_gpu = 0;
    int    nodes = 0;
};

// Batched-pool marginal gain vs the layered reference; logs [bench_correctness].
void benchmarkGpuCorrectness(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                             bool optimize_yaw, bool marginal_split, const char* phase);

// Per-node CPU-vs-GPU gain values (abs / 1-parent / all) -> NBV_X1_CSV for R².
void benchmarkX1Accuracy(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                         const std::vector<uint8_t>& flat_map, bool optimize_yaw, int replan_count, const char* phase);

// Timing of abs + G_all (CPU and GPU) on one tree; logs [X2rep]/[X2repABS]/[X2cpu]/[X1cpu].
void benchmarkX2Timing(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                       const std::vector<uint8_t>& flat_map, BenchAccum& acc,
                       bool optimize_yaw, bool marginal_split, int replan_count, const char* phase);

// Run whichever suite(s) the comma-separated string names ("correctness"/"x1"/"x2").
void runBenchSuite(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                   const std::vector<uint8_t>& flat_map, BenchAccum& acc, const std::string& suite,
                   bool optimize_yaw, bool marginal_split, int replan_count, const char* phase);

// End-of-cycle console summary of the accumulated X2 timings.
void logBenchSummary(const BenchAccum& acc);

}  // namespace planner_helpers

#endif  // MOTION_PLANNING_PLANNER_HELPERS_H
