#ifndef RRT_STAR_2D_YAW_H
#define RRT_STAR_2D_YAW_H

#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace rrt_star {

struct Node {
    Eigen::Vector3d point;
    Node* parent;
    double cost;
    double gain;

    Node(const Eigen::Vector3d& p);
};

Eigen::Vector2d sampleSpace(double dim_x, double dim_y);

Node* findNearest(std::vector<Node*>& tree, const Eigen::Vector2d& point);

Node* steer(Node* fromNode, const Eigen::Vector2d& toPoint, double stepSize);

bool collides(const std::vector<double>& point, const std::vector<std::pair<Eigen::Vector2d, double>>& obstacles);

std::vector<Node*> findNearby(std::vector<Node*>& tree, Node* point, double radius);

Node* chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& nearby_nodes, double radius);

std::pair<std::vector<Eigen::Vector3d>, Node*> backtrackPathNode(Node* node);

double calculateYawAngle(const Node* node1, const Node* node2);

std::pair<std::vector<Node*>, std::vector<Eigen::Vector3d>> rrtStar(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                                                                    const std::vector<std::pair<Eigen::Vector2d, double>>& obstacles,
                                                                    double dim_x, double dim_y, int max_iter,
                                                                    double step_size, double radius, double tolerance);

}  // namespace rrt_star

#endif // RRT_STAR_H