#ifndef RRT_STAR_2D_H
#define RRT_STAR_2D_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

namespace rrt_star_2d {

struct Node {
    Eigen::Vector4d point;
    std::shared_ptr<Node> parent;
    double cost;
    double gain;
    double score;

    Node(const Eigen::Vector4d& p);
};

Eigen::Vector3d sampleSpace(double dim_x, double dim_y);

void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

void findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode);

void steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result);

void steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node);

bool collides(const Eigen::Vector2d& point, const std::vector<std::pair<Eigen::Vector2d, double>>& obstacles);

void findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes);

void chooseParent(std::shared_ptr<Node>& point, const std::vector<std::shared_ptr<Node>>& nearbyNodes);

void rewire(std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

double calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2);

void backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode);

bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
             const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
             double dim_x, double dim_y, double dim_z, int max_iter,
             double step_size, double radius, double tolerance,
             std::vector<std::shared_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path);

}  // namespace rrt_star

#endif // RRT_STAR_2D_H