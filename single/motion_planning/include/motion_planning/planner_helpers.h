#ifndef MOTION_PLANNING_PLANNER_HELPERS_H
#define MOTION_PLANNING_PLANNER_HELPERS_H

// Stateless helpers shared by AEP and RH-NBVP (Plan 3, step 2). Free functions that take the
// needed state explicitly, so both planners' methods are thin wrappers over one implementation.

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <voxblox_ros/esdf_server.h>
#include <rrt_construction/rrt_star_kd.h>
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

// Log each non-root node's gain / score-contribution / score.
void logTreeNodes(rrt_star& tree, double lambda);

}  // namespace planner_helpers

#endif  // MOTION_PLANNING_PLANNER_HELPERS_H
