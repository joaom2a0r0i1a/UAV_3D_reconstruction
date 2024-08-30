#include "motion_planning_python/RRT/kino_rrt_star_kd.h"

// Constructors
kino_rrt_star::kino_rrt_star() {}

kino_rrt_star::Node::Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v) : point(p), velocity(v), parent(nullptr), cost(0), gain(0), score(0) {}

kino_rrt_star::Trajectory::Trajectory() : total_cost(0) {}

// Add and clear Nodes
void kino_rrt_star::KDTree_data::clear() {
    points.clear();
    data.clear();
}

void kino_rrt_star::addKDTreeNode(std::shared_ptr<Node>& node) {
    tree_data_.addNode(node);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
}

void kino_rrt_star::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void kino_rrt_star::initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes) {
    tree_data_.addNodes(nodes);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
Eigen::Vector3d kino_rrt_star::sampleSpace(double dim_x, double dim_y, double dim_z) {
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

void kino_rrt_star::computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
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

void kino_rrt_star::computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result) {
    bool solutionFound = false;
    while (!solutionFound) {
        double rand_x = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double rand_y = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double rand_z = 2.0 * radius * (((double) rand()) / ((double) RAND_MAX) - 0.5);        
        if (Eigen::Vector3d(rand_x, rand_y, rand_z).norm() > radius) {
            continue;
        }
        solutionFound = true;
    }
    double rand_yaw = 2.0 * M_PI * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    result = Eigen::Vector4d(rand_x, rand_y, rand_z, rand_yaw);
}

void kino_rrt_star::computeYaw(double radius, double& result) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> yaw_dis(-M_PI, M_PI);
    result = yaw_dis(gen);
}

void kino_rrt_star::computeAccelerationSampling(double a_max, Eigen::Vector3d& result) {
    bool solutionFound = false;
    while (!solutionFound) {
        double a_x = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double a_y = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        double a_z = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);        
        if (Eigen::Vector3d(a_x, a_y, a_z).norm() > a_max) {
            continue;
        }
        solutionFound = true;
    }
    result = Eigen::Vector3d(a_x, a_y, a_z);
}

void kino_rrt_star::findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestNode = tree_data_.data[index];
}

void kino_rrt_star::steer_trajectory(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, const Eigen::Vector3d& accel, double stepSize, std::vector<std::shared_ptr<Node>>& new_node_trajectory) {
    double dist = (toPoint - fromNode->point.head<3>()).norm();
    double dt = 0.1;
    Eigen::Vector3d current_velocity = fromNode->velocity;

    std::shared_ptr<Node> currentNode = fromNode;
    double distance = 0.0;

    while (distance < stepSize) {
        current_velocity += accel * dt;
        Eigen::Vector3d new_position = currentNode->point.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;
        double dx = new_position.x() - currentNode->point.x();
        double dy = new_position.y() - currentNode->point.y();
        double heading = std::atan2(dy, dx);

        std::shared_ptr<Node> newNode = std::make_shared<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), heading), current_velocity);
        newNode->parent = currentNode;
        newNode->cost = currentNode->cost + (new_position - currentNode->point.head<3>()).norm();
        new_node_trajectory.push_back(newNode);

        currentNode = newNode;
        distance += (new_position - fromNode->point.head<3>()).norm();
    }
}

bool kino_rrt_star::collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles) {
    for (const auto& obstacle : obstacles) {
        double distance = (point - obstacle.first).norm();
        if (distance < obstacle.second) {
            return true;
        }
    }
    return false;
}

void kino_rrt_star::findNearbyKD(const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes) {
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

void kino_rrt_star::chooseParent(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearbyNodes) {
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

void kino_rrt_star::rewire(const std::shared_ptr<Node>& new_node, std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius) {
    for (const auto& node : nearby_nodes) {
        double new_cost = new_node->cost + (node->point.head<3>() - new_node->point.head<3>()).norm();
        if (new_cost < node->cost) {
            node->parent = new_node;
            node->cost = new_cost;
        }
    }
}

double kino_rrt_star::calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2) {
    double dx = node2->point.x() - node1->point.x();
    double dy = node2->point.y() - node1->point.y();
    return std::atan2(dy, dx);
}

void kino_rrt_star::backtrackTrajectory(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& path, std::shared_ptr<Node>& nextBestNode) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->cost < nextBestNode->cost) {
            nextBestNode = currentNode;
        }
    }
    std::reverse(path.begin(), path.end());
}

void kino_rrt_star::backtrackPathAEP(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& path) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        path.push_back(currentNode);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end());
}

bool kino_rrt_star::rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
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
        steer_trajectory(nearestNode, randPoint, step_size, new_node_trajectory);
        
        if (!collides(newNode->point.head<3>(), obstacles)) {
            std::vector<std::shared_ptr<Node>> nearbyNodes;
            findNearby(tree, newNode, radius, nearbyNodes);
            chooseParent(newNode, nearbyNodes);
            tree.push_back(newNode);
            rewire(newNode, nearbyNodes, radius);
            
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
