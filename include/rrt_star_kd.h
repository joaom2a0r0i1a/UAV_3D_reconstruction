#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <libs/nanoflann.hpp>

namespace rrt_star {

struct Node {
    Eigen::Vector4d point; // Use standard containers instead of Eigen::Vector4d
    Node* parent;
    double cost;
    double gain;
    double score;

    Node(const Eigen::Vector4d& p);
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, KDTree>,
    KDTree, 4 /* Dimension */>;

Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

Eigen::Vector3d computeSamplingDimensions(double radius);

Node* findNearest(KDTree& tree, std::vector<Node>& nodes, const Eigen::Vector3d& point);

Node* steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize);

Node* steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize);

bool collides(const std::vector<double>& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

std::vector<Node*> findNearby(KDTree& tree, std::vector<Node>& nodes, Node* point, double radius);

Node* chooseParent(Node* point, std::vector<Node*>& nearbyNodes);

void rewire(KDTree& tree, std::vector<Node>& nodes, Node* new_node, std::vector<Node*>& nearby_nodes, double radius);

std::pair<std::vector<Eigen::Vector4d>, Node*> backtrackPathNode(Node* node);

double calculateYawAngle(const Node* node1, const Node* node2);

std::pair<std::vector<Node*>, std::vector<Eigen::Vector4d>> rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                                                                    const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                                                                    double dim_x, double dim_y, double dim_z, int max_iter,
                                                                    double step_size, double radius, double tolerance);

}  // namespace rrt_star

#endif // RRT_STAR_H
