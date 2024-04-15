#include "rrt_star.h"
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>

namespace rrt_star {

Node::Node(const Eigen::Vector4d& p) : point(p), parent(nullptr), cost(0), gain(0), score(0) {}

Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis_x(0.0, dim_x);
    std::uniform_real_distribution<double> dis_y(0.0, dim_y);
    std::uniform_real_distribution<double> dis_z(0.0, dim_z);
    double rand_x = dis_x(gen);
    double rand_y = dis_y(gen);
    double rand_z = dis_z(gen);
    return Eigen::Vector3d(rand_x, rand_y, rand_z);
}

Eigen::Vector3d computeSamplingDimensions(double radius) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-radius, radius);

    bool solutionFound = false;
    double rand_x, rand_y, rand_z;
    while (!solutionFound) {
        rand_x = dis(gen);
        rand_y = dis(gen);
        rand_z = dis(gen);
        solutionFound = (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() <= radius);
    }

    return Eigen::Vector3d(rand_x, rand_y, rand_z);
}

Node* findNearest(std::vector<Node*>& tree, const Eigen::Vector3d& point) {
    double minDist = std::numeric_limits<double>::max();
    Node* nearestNode = nullptr;
    for (Node* node : tree) {
        double distance = (node->point.head<3>() - point).norm();
        if (distance < minDist) {
            minDist = distance;
            nearestNode = node;
        }
    }
    return nearestNode;
}

Node* steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        return new Node(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point.z()));
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        return new Node(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.z()));
    }
}

bool collides(const std::vector<double>& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point[0] - obstacle.first[0]) * (point[0] - obstacle.first[0]) +
                          (point[1] - obstacle.first[1]) * (point[1] - obstacle.first[1]) + 
                          (point[2] - obstacle.first[2]) * (point[2] - obstacle.first[2]);
        if (distance < obstacle.second * obstacle.second * obstacle.second + 0.5) {
            return true;
        }
    }
    return false;
}

std::vector<Node*> findNearby(std::vector<Node*>& tree, Node* point, double radius) {
    std::vector<Node*> nearbyNodes;
    for (Node* node : tree) {
        double distance = (node->point.head<3>() - point->point.head<3>()).norm();
        if (distance < radius) {
            nearbyNodes.push_back(node);
        }
    }
    return nearbyNodes;
}

Node* chooseParent(Node* point, const std::vector<Node*>& nearbyNodes) {
    double minCost = std::numeric_limits<double>::infinity();
    Node* parent = nullptr;
    for (Node* node : nearbyNodes) {
        double cost = node->cost + (node->point.head<3>() - point->point.head<3>()).norm();
        if (cost < minCost) {
            minCost = cost;
            parent = node;
        }
    }
    point->parent = parent;
    point->cost = minCost;
    return point;
}

void rewire(std::vector<Node*>& tree, Node* new_node, const std::vector<Node*>& nearby_nodes, double radius) {
    for (const auto& node : nearby_nodes) {
        double new_cost = new_node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
        }
    }
}

double calculateYawAngle(const Node* node1, const Node* node2) {
    double dx = node2->point.x() - node1->point.x();
    double dy = node2->point.y() - node1->point.y();
    return std::atan2(dy, dx);
}

std::pair<std::vector<Eigen::Vector4d>, Node*> backtrackPathNode(Node* node) {
    std::vector<Eigen::Vector4d> path;
    Node* nextBestNode = node;
    while (node) {
        path.push_back({node->point});
        node = node->parent;
        if (node && node->parent && node->cost < nextBestNode->cost) {
            nextBestNode = node;
        }
    }
    std::reverse(path.begin(), path.end());
    return {path, nextBestNode};
}

std::pair<std::vector<Node*>, std::vector<Eigen::Vector4d>> rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                                                                    const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                                                                    double dim_x, double dim_y, double dim_z, int max_iter,
                                                                    double step_size, double radius, double tolerance) {
    std::vector<Node*> tree = {new Node(start)};
    std::vector<Node*> goalReachedNodes;
    std::vector<Eigen::Vector4d> path;
    Node* minCostNode = nullptr; // Declaration of minCostNode variable

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d randPoint = sampleSpace(dim_x, dim_y, dim_z);
        Node* nearestNode = findNearest(tree, randPoint);
        Node* newNode = steer(nearestNode, randPoint, step_size);
        
        if (!collides({newNode->point.x(), newNode->point.y(), newNode->point.z()}, obstacles)) {
            std::vector<Node*> nearbyNodes = findNearby(tree, newNode, radius);
            newNode = chooseParent(newNode, nearbyNodes);
            tree.push_back(newNode);
            rewire(tree, newNode, nearbyNodes, radius);
            
            if ((newNode->point.head<3>() - goal.head<3>()).norm() <= tolerance) {
                std::cout << "Goal reached!" << std::endl;

                goalReachedNodes.push_back(newNode);
                minCostNode = *std::min_element(goalReachedNodes.begin(), goalReachedNodes.end(),
                                                      [](Node* node1, Node* node2) { return node1->cost < node2->cost; });
                auto [p, nextBestNode] = backtrackPathNode(minCostNode);
                path = p;

                for (size_t i = 0; i < path.size() - 1; ++i) {
                    path[i][3] = calculateYawAngle(new Node(path[i]), new Node(path[i + 1]));
                    if (i == path.size() - 2) {
                        path[i + 1][3] = goal[3];
                    }
                }
                return {tree, path};
            }
        }
    }
    return {tree, path};
}


/*int main() {
    Eigen::Vector3d start(1, 1, 1, 0);
    Eigen::Vector3d goal(9, 9, 9, 0);
    double dim_x = 10;
    double dim_y = 10;
    double dim_z = 10;
    std::vector<std::pair<Eigen::Vector3d, double>> obstacles = { {Eigen::Vector3d(5, 5, 5), 1} };
    auto [tree, path] = rrtStar(start, goal, obstacles, dim_x, dim_y, dim_z, 1000, 0.5, 2, 0.5);
    return 0;
}*/

}  // namespace rrt_star
