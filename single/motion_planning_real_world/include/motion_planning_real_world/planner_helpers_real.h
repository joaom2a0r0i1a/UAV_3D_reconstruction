#ifndef MOTION_PLANNING_REAL_WORLD_PLANNER_HELPERS_REAL_H
#define MOTION_PLANNING_REAL_WORLD_PLANNER_HELPERS_REAL_H

// mrs-free copy of motion_planning/planner_helpers for the real-world planners.
// Deltas vs sim: no benchmark section (no benchmarking mid-air), distance() takes Eigen
// (mavros, no mrs Reference), markers use the BARE frame_id (real frames: map/base_link —
// no MRS uavX/ prefix). Same namespace + names so the planner bodies match the sim ones.

#include <vector>
#include <string>
#include <Eigen/Core>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>
#include <rrt_construction/rrt_star_kd.h>

namespace planner_helpers {

// ESDF clearance at a world position (0.0 if no ESDF or the point is unmapped).
double getMapDistance(const voxblox::EsdfServer& server, const Eigen::Vector3d& position);

// True iff every node on the path clears uav_radius.
bool isPathCollisionFree(const voxblox::EsdfServer& server, const std::vector<rrt_star::Node*>& path, double uav_radius);

// Sample from->to at `resolution` spacing requiring clearance >= uav_radius; optimistic_edges treats unobserved space as free (bootstrap) vs blocking it.
bool isEdgeCollisionFree(const voxblox::EsdfServer& server, const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                         double uav_radius, double resolution, bool optimistic_edges);

// 3D euclidean distance between two [x,y,z,yaw] points (yaw ignored).
double distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b);

// All non-root nodes of the tree.
std::vector<rrt_star::Node*> collectTreeNodes(rrt_star& tree);

// True iff p is inside the axis-aligned bounded box.
bool inBoundingBox(const Eigen::Vector4d& p, float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

// Log each non-root node's gain / score-contribution / score.
void logTreeNodes(rrt_star& tree, double lambda);

// --- RViz marker helpers. Publisher / frame passed explicitly; id counters mutated in place. ---
void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id,
                    const std::vector<rrt_star::Node*>& nodes);
void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id,
                    rrt_star::Node* node, int& path_id_counter);
void clear_all_voxels(ros::Publisher& pub_voxels);
void clearMarkers(ros::Publisher& pub_markers, int& node_id_counter, int& edge_id_counter, int& path_id_counter);

}  // namespace planner_helpers

#endif  // MOTION_PLANNING_REAL_WORLD_PLANNER_HELPERS_REAL_H
