#include "motion_planning/planner_helpers.h"

#include <ros/ros.h>
#include <mrs_lib/geometry/misc.h>
#include <cmath>
#include <algorithm>

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

void logTreeNodes(rrt_star& tree, double lambda) {
    for (const auto& up : tree.getNodes())
        if (up->parent)
            ROS_INFO("[Node] gain=%.3f score_contribution=%.3f score=%.3f",
                     up->gain, up->gain * exp(-lambda * up->cost), up->score);
}

}  // namespace planner_helpers
