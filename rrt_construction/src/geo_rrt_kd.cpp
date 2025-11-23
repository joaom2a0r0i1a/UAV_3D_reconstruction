#include "rrt_construction/geo_rrt_kd.h"

// Constructors
geo_rrt::geo_rrt() {}

geo_rrt::Node::Node(const Eigen::Vector4d& p) : pose(p), parent(nullptr), path_gain(0), max_gain(0), score(0), cost(0) {}

// Add and clear Nodes
void geo_rrt::KDTree_data::clear() {
    points.clear();
    data.clear();
}

void geo_rrt::addKDTreeNode(std::shared_ptr<Node>& node) {
    tree_data_.addNode(node);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
}

void geo_rrt::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void geo_rrt::initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes) {
    tree_data_.addNodes(nodes);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
Eigen::Vector3d geo_rrt::sampleSpace(double dim_x, double dim_y, double dim_z) {
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

void geo_rrt::computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
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

void geo_rrt::computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result) {
    bool solutionFound = false;
    double rand_x, rand_y, rand_z, rand_yaw;
    while (!solutionFound) {
        rand_x = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        rand_y = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        rand_z = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);        
        if (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() > radius) {
            continue;
        }
        solutionFound = true;
    }
    rand_yaw = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);

    result = Eigen::Vector4d(rand_x, rand_y, rand_z, rand_yaw);
}

void geo_rrt::computeYaw(double radius, double& result) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> yaw_dis(-M_PI, M_PI);
    result = yaw_dis(gen);
}

void geo_rrt::findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double minDist = std::numeric_limits<double>::max();
    for (const auto& node : tree) {
        double distance = (node->pose.head<3>() - point).norm();
        if (distance < minDist) {
            minDist = distance;
            nearestNode = node;
        }
    }
}

void geo_rrt::findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestNode = tree_data_.data[index];
}

void geo_rrt::steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result) {
    double dist = (toPoint - fromNode->pose.head<3>()).norm();
    if (dist < stepSize) {
        result = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->pose.w()));
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->pose.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->pose.head<3>() + stepSize * direction;
        result = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->pose.w()));
    }
}

void geo_rrt::steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node) {
    double dist = (toPoint - fromNode->pose.head<3>()).norm();
    if (dist < stepSize) {
        new_node = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->pose.w()));
        new_node->parent = fromNode;
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->pose.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->pose.head<3>() + stepSize * direction;
        new_node = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->pose.w()));
        new_node->parent = fromNode;
    }
}

bool geo_rrt::collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point - obstacle.first).norm();
        if (distance < obstacle.second) {
            return true;
        }
    }
    return false;
}

void geo_rrt::findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    for (const auto& node : tree) {
        double distance = (node->pose.head<3>() - point->pose.head<3>()).norm();
        if (distance < radius) {
            nearbyNodes.push_back(node);
        }
    }
}

void geo_rrt::findNearbyKD(const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    nearbyNodes.clear();
    Eigen::Vector3d query_pt = {point->pose.x(), point->pose.y(), point->pose.z()};
    std::size_t ret_index[10];
    double out_dist[10];   
    nanoflann::KNNResultSet<double> resultSet(10);
    resultSet.init(ret_index, out_dist);
    kdtree_->findNeighbors(resultSet, query_pt.data(), nanoflann::SearchParameters(10));
    for (int i = 0; i < resultSet.size(); ++i) {
        if (out_dist[i] <= pow(radius, 2.0)) {
            nearbyNodes.push_back(tree_data_.data[ret_index[i]]);
        }
    }
}

void geo_rrt::chooseParent(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    double minCost = std::numeric_limits<double>::infinity();
    std::shared_ptr<Node> parent = nullptr;
    for (const auto& node : nearbyNodes) {
        double cost = node->cost + (node->pose.head<3>() - new_node->pose.head<3>()).norm();
        if (cost < minCost) {
            minCost = cost;
            parent = node;
        }
    }
    new_node->parent = parent;
    new_node->cost = minCost;
}

void geo_rrt::rewire(const std::shared_ptr<Node>& new_node, std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius) {
    for (const auto& node : nearby_nodes) {
        double new_cost = new_node->cost + (node->pose.head<3>() - new_node->pose.head<3>()).norm();
        if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
        }
    }
}

double geo_rrt::calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) {
    double dx = node2->pose.x() - node1->pose.x();
    double dy = node2->pose.y() - node1->pose.y();
    return std::atan2(dy, dx);
}

void geo_rrt::backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back(currentNode->pose);
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->cost < nextBestNode->cost) {
            nextBestNode = currentNode;
        }
    }
    std::reverse(path.begin(), path.end());
}

void geo_rrt::backtrackPathAEP(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& path) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end());
}
