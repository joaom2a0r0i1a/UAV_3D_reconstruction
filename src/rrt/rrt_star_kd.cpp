#include "rrt_star_kd.h"

namespace rrt_star {

Node::Node(const Eigen::Vector3d& p) : point(p), parent(nullptr), cost(0), gain(0), score(0) {}

/*void buildKDTree(std::vector<std::shared_ptr<Node>>& points, std::shared_ptr<Tree>& tree) {
    // Create the KD-tree adaptor
    PointCloudAdaptor pcAdaptor(points);
    tree = std::make_shared<Tree>(3, pcAdaptor);
    tree->buildIndex();
}

void destroyKDTree(std::shared_ptr<Tree>& tree) {
    tree.reset();
}

void initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& points, std::shared_ptr<Tree>& tree) {
    buildKDTree(points, tree);
}*/

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

void computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
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

    result = Eigen::Vector3d(rand_x, rand_y, rand_z);
}

void findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double minDist = std::numeric_limits<double>::max();
    for (const auto& node : tree) {
        double distance = (node->point.head<3>() - point).norm();
        if (distance < minDist) {
            minDist = distance;
            nearestNode = node;
        }
    }
}

// CHANGE THIS ////////////////////
// ADD RRT AS CLASS ///////////////
// CHANGE FINDNEAREST /////////////
// CHANGE FINDNEARBY //////////////

void findNearest(const Eigen::Vector3d& point, std::unique_ptr<Tree>& kdTree, std::shared_ptr<Node>& nearestNode) {
    Eigen::Vector3d query_pt = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    resultSet.init(&index, &nearestNode->cost);
    kdTree->findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParameters(10));
    nearestNode->cost = std::sqrt(nearestNode->cost); // NEAREST NODE RETURNED SQUARED
}

///////////////////////////////////

void steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        result = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point.z()));
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        result = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.z()));
    }
}

void steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        new_node = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point.z()));
        new_node->parent = fromNode;
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        new_node = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.z()));
        new_node->parent = fromNode;
    }
}

bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point - obstacle.first).norm();
        if (distance < obstacle.second) {
            return true;
        }
    }
    return false;
}

void findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    for (const auto& node : tree) {
        double distance = (node->point.head<3>() - point->point.head<3>()).norm();
        if (distance < radius) {
            nearbyNodes.push_back(node);
        }
    }
}

// CHANGE HERE

/*void findNearby(const std::shared_ptr<Node>& point, double radius, std::shared_ptr<Tree>& kdTree, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    Eigen::Vector3d query_pt = {point->point.x(), point->point.y(), point->point.z()};
    nanoflann::SearchParameters params;
    const size_t num_results = kdTree->radiusSearch(query_pt.data(), radius * radius, std::back_inserter(nearbyNodes), params); // Square the radius for KD-tree search
}*/

void findNearby(const std::shared_ptr<Node>& point, double radius, std::shared_ptr<Tree>& kdTree, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    nearbyNodes.clear();
    Eigen::Vector3d query_pt = {point->point.x(), point->point.y(), point->point.z()};
    std::size_t ret_index[10];
    double out_dist[10];   
    nanoflann::KNNResultSet<double> resultSet(10);
    resultSet.init(ret_index, out_dist);
    kdTree->findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParameters(10));
    for (int i = 0; i < resultSet.size(); ++i) {
    if (out_dist[i] <= pow(radius, 2.0)) {
        nearbyNodes.push_back(tree_data.points[ret_index[i]]);
    }
    }
}

void chooseParent(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    double minCost = std::numeric_limits<double>::infinity();
    std::shared_ptr<Node> parent = nullptr;
    for (const auto& node : nearbyNodes) {
        double cost = node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (cost < minCost) {
            minCost = cost;
            parent = node;
        }
    }
    new_node->parent = parent;
    new_node->cost = minCost;
}

void rewire(std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius) {
    for (const auto& node : nearby_nodes) {
        double new_cost = new_node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
        }
    }
}

double calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) {
    double dx = node2->point.x() - node1->point.x();
    double dy = node2->point.y() - node1->point.y();
    return std::atan2(dy, dx);
}

void backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back({currentNode->point, 0}); // CHANGE HERE
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->cost < nextBestNode->cost) {
            nextBestNode = currentNode;
        }
    }
    std::reverse(path.begin(), path.end());
}

bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
             const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
             double dim_x, double dim_y, double dim_z, int max_iter,
             double step_size, double radius, double tolerance,
             std::vector<std::shared_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path) {
    tree = {std::make_shared<Node>(start)};
    std::vector<std::shared_ptr<Node>> goalReachedNodes;
    std::shared_ptr<Node> minCostNode = nullptr;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d randPoint;
        computeSamplingDimensions(radius, randPoint);
        std::shared_ptr<Node> nearestNode;
        findNearest(tree, randPoint, nearestNode);
        std::shared_ptr<Node> newNode;
        steer(nearestNode, randPoint, step_size, newNode);
        
        if (!collides(newNode->point.head<3>(), obstacles)) {
            std::vector<std::shared_ptr<Node>> nearbyNodes;
            findNearby(tree, newNode, radius, nearbyNodes);
            chooseParent(newNode, nearbyNodes);
            tree.push_back(newNode);
            rewire(tree, newNode, nearbyNodes, radius);
            
            if ((newNode->point.head<3>() - goal.head<3>()).norm() <= tolerance) {
                std::cout << "Goal reached!" << std::endl;

                goalReachedNodes.push_back(newNode);
                minCostNode = *std::min_element(goalReachedNodes.begin(), goalReachedNodes.end(),
                                                      [](const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) { return node1->cost < node2->cost; });
                backtrackPathNode(minCostNode, path, minCostNode);
                
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    path[i][3] = calculateYawAngle(std::make_shared<Node>(path[i]), std::make_shared<Node>(path[i + 1]));
                    if (i == path.size() - 2) {
                        path[i + 1][3] = goal[3];
                    }
                }
                return true;
            }
        }
    }
    return false;
}

}  // namespace rrt_star
