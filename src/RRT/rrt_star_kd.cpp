#include "RRT/rrt_star_kd.h"

// Constructors
rrt_star::rrt_star() {}

rrt_star::Node::Node(const Eigen::Vector4d& p) : point(p), parent(nullptr), cost(0), gain(0), score(0) {}

// Add and clear Nodes
void rrt_star::KDTree_data::clear() {
    points.clear();
    data.clear();
}

void rrt_star::addKDTreeNode(std::shared_ptr<Node>& node) {
    tree_data_.addNode(node);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
}

void rrt_star::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void rrt_star::initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes) {
    tree_data_.addNodes(nodes);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
Eigen::Vector3d rrt_star::sampleSpace(double dim_x, double dim_y, double dim_z) {
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

void rrt_star::computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
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

void rrt_star::findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double minDist = std::numeric_limits<double>::max();
    for (const auto& node : tree) {
        double distance = (node->point.head<3>() - point).norm();
        if (distance < minDist) {
            minDist = distance;
            nearestNode = node;
        }
    }
}

void rrt_star::findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestNode = tree_data_.data[index];
}

void rrt_star::steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        result = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point.w()));
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        result = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.w()));
    }
}

void rrt_star::steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    if (dist < stepSize) {
        new_node = std::make_shared<Node>(Eigen::Vector4d(toPoint.x(), toPoint.y(), toPoint.z(), fromNode->point.w()));
        new_node->parent = fromNode;
    } else {
        Eigen::Vector3d direction = (toPoint - fromNode->point.head<3>()).normalized();
        Eigen::Vector3d newPoint = fromNode->point.head<3>() + stepSize * direction;
        new_node = std::make_shared<Node>(Eigen::Vector4d(newPoint.x(), newPoint.y(), newPoint.z(), fromNode->point.w()));
        new_node->parent = fromNode;
    }
    //addKDTreeNode(new_node);
}

bool rrt_star::collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point - obstacle.first).norm();
        if (distance < obstacle.second) {
            return true;
        }
    }
    return false;
}

void rrt_star::findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    for (const auto& node : tree) {
        double distance = (node->point.head<3>() - point->point.head<3>()).norm();
        if (distance < radius) {
            nearbyNodes.push_back(node);
        }
    }
}

void rrt_star::findNearbyKD(const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
    nearbyNodes.clear();
    Eigen::Vector3d query_pt = {point->point.x(), point->point.y(), point->point.z()};
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

void rrt_star::chooseParent(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearbyNodes) {
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

void rrt_star::rewire(std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius) {
    for (const auto& node : nearby_nodes) {
        double new_cost = new_node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
        }
    }
}

double rrt_star::calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) {
    double dx = node2->point.x() - node1->point.x();
    double dy = node2->point.y() - node1->point.y();
    return std::atan2(dy, dx);
}

void rrt_star::backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back(currentNode->point);
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->cost < nextBestNode->cost) {
            nextBestNode = currentNode;
        }
    }
    std::reverse(path.begin(), path.end());
}

bool rrt_star::rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
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
