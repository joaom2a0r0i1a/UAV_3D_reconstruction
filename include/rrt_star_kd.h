#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <libs/nanoflann.hpp>

namespace rrt_star {

struct Node {
    Eigen::Vector3d point;
    std::shared_ptr<Node> parent;
    double cost;
    double gain;
    double score;

    Node(const Eigen::Vector3d& p);
};

struct KDTree_data {
    const std::vector<std::shared_ptr<Node>>& points;

    KDTree_data(const std::vector<std::shared_ptr<Node>>& points) : points(points) {}

    inline size_t kdtree_get_point_count() const {
        return points.size();
    }

    inline double kdtree_distance(const double* p1, const size_t idx_p2, size_t /*size*/) const {
        const double d0 = p1[0] - points[idx_p2]->point.x();
        const double d1 = p1[1] - points[idx_p2]->point.y();
        const double d2 = p1[2] - points[idx_p2]->point.z();
        return d0 * d0 + d1 * d1 + d2 * d2; // Euclidean squared distance
    }

    inline double kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0) return points[idx]->point.x();
        else if (dim == 1) return points[idx]->point.y();
        else return points[idx]->point.z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
};

KDTree_data tree_data;

// Define the type for the KD-tree
typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, KDTree_data>, KDTree_data, 3> Tree;

void buildKDTree(std::vector<std::shared_ptr<Node>>& points, std::shared_ptr<Tree>& tree);

void destroyKDTree(std::shared_ptr<Tree>& tree);

void initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& points, std::shared_ptr<Tree>& tree);

Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

//void findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode);

void findNearest(const Eigen::Vector3d& point, std::shared_ptr<Tree>& kdTree, std::shared_ptr<Node>& nearestNode);

void steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result);

void steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node);

bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

//void findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes);

void findNearby(const std::shared_ptr<Node>& point, double radius, std::shared_ptr<Tree>& kdTree, std::vector<std::shared_ptr<Node>>& nearbyNodes);

void chooseParent(std::shared_ptr<Node>& point, const std::vector<std::shared_ptr<Node>>& nearbyNodes);

void rewire(std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

//void rewire(std::shared_ptr<Tree>& kdTree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

double calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2);

void backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode);

bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
             const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
             double dim_x, double dim_y, double dim_z, int max_iter,
             double step_size, double radius, double tolerance,
             std::vector<std::shared_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path);

}  // namespace rrt_star

#endif // RRT_STAR_H

