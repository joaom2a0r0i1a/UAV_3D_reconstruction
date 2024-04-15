#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace rrt_star {

struct Node {
    Eigen::Vector4d point;
    Node* parent;
    double cost;
    double gain;
    double score;

    Node(const Eigen::Vector4d& p);
};

Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

Eigen::Vector3d computeSamplingDimensions(double radius);

Node* findNearest(std::vector<Node*>& tree, const Eigen::Vector3d& point);

Node* steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize);

bool collides(const std::vector<double>& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

std::vector<Node*> findNearby(std::vector<Node*>& tree, Node* point, double radius);

Node* chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& nearby_nodes, double radius);

std::pair<std::vector<Eigen::Vector4d>, Node*> backtrackPathNode(Node* node);

double calculateYawAngle(const Node* node1, const Node* node2);

std::pair<std::vector<Node*>, std::vector<Eigen::Vector4d>> rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                                                                    const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                                                                    double dim_x, double dim_y, double dim_z, int max_iter,
                                                                    double step_size, double radius, double tolerance);

}  // namespace rrt_star

#endif // RRT_STAR_H