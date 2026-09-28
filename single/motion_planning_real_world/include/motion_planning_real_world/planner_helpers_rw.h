#ifndef PLANNER_HELPERS_RW_H
#define PLANNER_HELPERS_RW_H

// Real world copy of planner_helpers, without MRS

#include <vector>
#include <string>
#include <Eigen/Core>
#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>
#include <voxblox_ros/esdf_server.h>
#include <rrt_construction/rrt_star_kd.h>

namespace planner_helpers {

// ESDF clearance at a position
double getMapDistance(const voxblox::EsdfServer& server, const Eigen::Vector3d& position);

// True iff every node on the path clears uav_radius.
bool isPathCollisionFree(const voxblox::EsdfServer& server, const std::vector<rrt_star::Node*>& path, double uav_radius);

// Edge clearance check
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

/*                RVIZ MARKERS               */
void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id,
                    const std::vector<rrt_star::Node*>& nodes);
void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id,
                    rrt_star::Node* node, int& path_id_counter);
void clear_all_voxels(ros::Publisher& pub_voxels);
void clearMarkers(ros::Publisher& pub_markers, int& node_id_counter, int& edge_id_counter, int& path_id_counter);

}  // namespace planner_helpers

#endif  // PLANNER_HELPERS_RW_H
