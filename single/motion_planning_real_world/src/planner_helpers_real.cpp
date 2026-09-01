#include "motion_planning_real_world/planner_helpers_real.h"

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <algorithm>

namespace planner_helpers {

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

double distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
    return (a.head<3>() - b.head<3>()).norm();
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

void visualize_tree(ros::Publisher& pub_markers, const std::string& frame_id,
                    const std::vector<rrt_star::Node*>& nodes) {
    visualization_msgs::Marker edges;
    edges.header.stamp = ros::Time::now();
    edges.header.frame_id = frame_id;
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

void visualize_path(ros::Publisher& pub_markers, const std::string& frame_id,
                    rrt_star::Node* node, int& path_id_counter) {
    rrt_star::Node* current = node;

    while (current->parent) {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = path_id_counter;
        p.header.frame_id = frame_id;
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

}  // namespace planner_helpers
