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

// --- Gain benchmark (shared by AEP + RH-NBVP; RH-NBVP is the canonical implementation). ---
// Per-cycle accumulators; reset with `acc = {}` at the start of a planning cycle.
struct BenchAccum {
    double ms_gall_gpu = 0, ms_gall_split = 0, ms_g1p_cpu = 0, ms_abs_gpu = 0, ms_abs_cpu = 0;
    double kernel_gall_gpu = 0, kernel_abs_gpu = 0, g1p_err_sum = 0, g1p_err_max = 0;
    int    nodes = 0;
};

// Time all gain variants (GPU vs CPU), log X2 lines, dump the X1 CSV (NBV_X1_CSV), check batched-vs-reference; accumulates into acc.
void benchmarkGains(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                    const std::vector<uint8_t>& flat_map, BenchAccum& acc,
                    bool optimize_yaw, bool marginal_split, int replan_count, const char* phase);

// End-of-cycle console summary of the accumulated benchmark timings.
void logBenchSummary(const BenchAccum& acc);

}  // namespace planner_helpers

#endif  // MOTION_PLANNING_PLANNER_HELPERS_H
