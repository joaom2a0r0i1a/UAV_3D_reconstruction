#include "rrt_star_kd.h"
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
        if (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() > radius) {
            continue;
        }
        solutionFound = true;
    }

    return Eigen::Vector3d(rand_x, rand_y, rand_z);
}

Node* findNearest(KDTree& tree, std::vector<Node>& nodes, const Eigen::Vector3d& point) {
    double query_pt[3] = { point.x(), point.y(), point.z() };
    size_t ret_index;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);
    tree.findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    return &nodes[ret_index];
}

Node* steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        return new Node(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point[3]));
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        return new Node(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point[3]));
    }
}

Node* steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    Node* new_node = nullptr;
    if (dist < stepSize) {
        new_node = new Node(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point[3]));
        new_node->parent = fromNode;
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        new_node = new Node(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point[3]));
        new_node->parent = fromNode;
    }
    return new_node;
}

bool collides(const Eigen::Vector4d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point.head<3>() - obstacle.first).norm();
        if (distance < obstacle.second) {
            return true;
        }
    }
    return false;
}

std::vector<Node*> findNearby(KDTree& tree, std::vector<Node>& nodes, Node* point, double radius) {
    std::vector<Node*> nearbyNodes;
    std::vector<nanoflann::ResultItem<uint32_t, double>> indices_dists;
    nanoflann::SearchParameters params;
    tree.radiusSearch(&point->point[0], radius, indices_dists, params);
    for (const auto& [idx, dist] : indices_dists) {
        nearbyNodes.push_back(&nodes[idx]);
    }
    return nearbyNodes;
}

Node* chooseParent(Node* point, std::vector<Node*>& nearbyNodes) {
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

void rewire(KDTree& tree, std::vector<Node>& nodes, Node* new_node, std::vector<Node*>& nearby_nodes, double radius) {
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
        path.push_back({ node->point });
        node = node->parent;
        if (node && node->parent && node->cost < nextBestNode->cost) {
            nextBestNode = node;
        }
    }
    std::reverse(path.begin(), path.end());
    return { path, nextBestNode };
}

std::pair<std::vector<Node*>, std::vector<Eigen::Vector4d>> rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                                                                    const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                                                                    double dim_x, double dim_y, double dim_z, int max_iter,
                                                                    double step_size, double radius, double tolerance) {
    std::vector<Node> nodes = { Node(start) };
    KDTree tree(4, nodes, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
    tree.buildIndex();
    std::vector<Node*> goalReachedNodes;
    std::vector<Eigen::Vector4d> path;
    Node* minCostNode = nullptr;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d randPoint = sampleSpace(dim_x, dim_y, dim_z);
        Node* nearestNode = findNearest(tree, nodes, randPoint);
        Node* newNode = steer(nearestNode, randPoint, step_size);

        if (!collides(newNode->point, obstacles)) {
            std::vector<Node*> nearbyNodes = findNearby(tree, nodes, newNode, radius);
            newNode = chooseParent(newNode, nearbyNodes);
            nodes.push_back(*newNode);
            tree.addPoints(nodes.size() - 1, nodes.size() - 1);
            rewire(tree, nodes, newNode, nearbyNodes, radius);

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
                return { goalReachedNodes, path };
            }
        }
    }
    return { goalReachedNodes, path };
}

}  // namespace rrt_star
