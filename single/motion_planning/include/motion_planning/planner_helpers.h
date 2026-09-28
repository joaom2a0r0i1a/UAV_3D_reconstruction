#ifndef PLANNER_HELPERS_H
#define PLANNER_HELPERS_H

// Helpers shared by AEP and RH_NBVP

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

// ESDF clearance at a position
double getMapDistance(const voxblox::EsdfServer& server, const Eigen::Vector3d& position);

// True iff every node on the path clears uav_radius.
bool isPathCollisionFree(const voxblox::EsdfServer& server, const std::vector<rrt_star::Node*>& path, double uav_radius);

// Edge clearance check
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

/*                RVIZ MARKERS               */
void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    const std::vector<rrt_star::Node*>& nodes);
void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id, const std::string& ns,
                    rrt_star::Node* node, int& path_id_counter);
void clear_all_voxels(ros::Publisher& pub_voxels);
void clearMarkers(ros::Publisher& pub_markers, int& node_id_counter, int& edge_id_counter, int& path_id_counter);

/*               GAIN BENCHMARK              */

// Per-cycle timing accumulators
struct BenchAccum {
    double ms_gall_gpu = 0, ms_abs_gpu = 0, ms_abs_cpu = 0, ms_gall_cpu = 0;
    double kernel_gall_gpu = 0, kernel_abs_gpu = 0;
    int nodes = 0;
};

// Batch check suite
void benchmarkBatchCheck(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                         bool optimize_yaw, bool marginal_split, const char* phase);

// Accuracy suite
void benchmarkAccuracy(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                       const std::vector<uint8_t>& flat_map, bool optimize_yaw, int replan_count, const char* phase);

// Timing suite
void benchmarkTiming(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                     const std::vector<uint8_t>& flat_map, BenchAccum& acc,
                     bool optimize_yaw, bool marginal_split, int replan_count, const char* phase);

// Run the named suites
void runBenchSuite(GainEvaluator& seg, const std::vector<rrt_star::Node*>& nodes,
                   const std::vector<uint8_t>& flat_map, BenchAccum& acc, const std::string& suite,
                   bool optimize_yaw, bool marginal_split, int replan_count, const char* phase);

// Timing summary
void logBenchSummary(const BenchAccum& acc);

}

#endif  // PLANNER_HELPERS_H
