#include "rrt_construction/improved_krrt_kd.h"

// Constructors
improved_krrt::improved_krrt() {}

improved_krrt::Node::Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a, const double yrate) : time(0), gain(0), cost1(0), cost2(0), score(0), yaw_rate(yrate), pose(p), velocity(v), acceleration(a), parent(nullptr) {}

// Add and clear Nodes
void improved_krrt::KDTree_data::clear() {
    points.clear();
    data.clear();
}

void improved_krrt::addKDTreeNode(std::shared_ptr<Node>& node) {
    tree_data_.addNode(node);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
}

void improved_krrt::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void improved_krrt::initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes) {
    tree_data_.addNodes(nodes);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
void improved_krrt::computeSamplingDimensions(double radius, Eigen::Vector3d& result) {
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

void improved_krrt::computeSamplingDimensionsYaw(double radius, Eigen::Vector4d& result) {
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

void improved_krrt::computeAccelerationSampling(double a_max, Eigen::Vector3d& result) {
    bool solutionFound = false;
    double a_x, a_y, a_z;
    while (!solutionFound) {
        a_x = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        a_y = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        if (Eigen::Vector2d(a_x, a_y).norm() > a_max) {
            continue;
        }
        solutionFound = true;
    }
    a_z = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    result = Eigen::Vector3d(a_x, a_y, a_z);
}

void improved_krrt::findNearestKD(const Eigen::Vector3d& position, std::shared_ptr<Node>& nearestNode) {
    double query_pt[3] = {position.x(), position.y(), position.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestNode = tree_data_.data[index];
}

void improved_krrt::steer_trajectory(const std::shared_ptr<Node>& fromNode, double max_velocity, double target_heading, Eigen::Vector3d& accel, double max_heading_velocity, double max_heading_acceleration, double stepSizeTime, std::vector<std::shared_ptr<Node>>& newNodes) {
    double dt = 0.1;
    std::shared_ptr<Node> currentNode = fromNode;
    Eigen::Vector3d current_velocity = currentNode->velocity; 

    double max_velocity_modulus = sqrt(pow(max_velocity, 2) + pow(max_velocity, 2));
    
    double current_heading = currentNode->pose[3];
    double current_heading_velocity = currentNode->yaw_rate;
    bool heading_tolerance = max_heading_velocity*dt;

    while (currentNode->time < stepSizeTime) {
        // Translation
        if (current_velocity.head(2).norm() > max_velocity) {
            current_velocity.head(2) = current_velocity.head(2).normalized() * max_velocity;
            accel[0] = 0;
            accel[1] = 0;
        }

        if (current_velocity[2] > max_velocity) {
            current_velocity[2] = max_velocity;
            accel[2] = 0;
        } else if (current_velocity[2] < -max_velocity) {
            current_velocity[2] = -max_velocity;
            accel[2] = 0;
        }

        Eigen::Vector3d new_position = currentNode->pose.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;

        // Rotation
        current_heading = fmod(current_heading + M_PI, 2.0 * M_PI) - M_PI;
        double heading_difference = target_heading - current_heading;
        heading_difference = fmod(heading_difference + M_PI, 2.0 * M_PI) - M_PI;

        double accel_heading = 0.0;
        if (heading_difference > 0) {
            accel_heading = max_heading_acceleration;
        } else if (heading_difference < 0) {
            accel_heading = -max_heading_acceleration;
        }

        if (current_heading_velocity > max_heading_velocity) {
            current_heading_velocity = max_heading_velocity;
            accel_heading = 0;
        } else if (current_heading_velocity < -max_heading_velocity) {
            current_heading_velocity = -max_heading_velocity;
            accel_heading = 0;
        }

        double new_heading;
        if (std::abs(heading_difference) > heading_tolerance) {
            new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
        } else {
            new_heading = current_heading;
        }
        
        // Updated Velocities
        current_velocity += accel * dt;
        current_heading_velocity += accel_heading * dt;

        // New Node
        std::shared_ptr<Node> newNode = std::make_shared<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), new_heading), current_velocity, accel, 0.0);
        newNode->parent = currentNode;
        newNode->cost1 = newNode->parent->cost1 + std::abs((newNode->velocity).norm() - max_velocity_modulus)*dt;
        newNode->cost2 = newNode->parent->cost2 + dt;
        newNode->time = newNode->parent->time + dt;
        newNodes.push_back(newNode);

        currentNode = newNode;
        current_heading = new_heading;
    }
}

void improved_krrt::steer_trajectory_linear(const std::shared_ptr<Node>& fromNode, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSizeTime, std::vector<std::shared_ptr<Node>>& newNodes) {
    double dt = 0.1;
    std::shared_ptr<Node> currentNode = fromNode;
    Eigen::Vector3d current_velocity = currentNode->velocity; 

    double distance = 0.0;

    double max_velocity_modulus = sqrt(pow(max_velocity, 2) + pow(max_velocity, 2));

    while (distance < stepSizeTime) {
    //while (currentNode->time < stepSizeTime) {
        if (current_velocity.head(2).norm() > max_velocity) {
            current_velocity.head(2) = current_velocity.head(2).normalized() * max_velocity;
            accel[0] = 0;
            accel[1] = 0;
        }

        if (current_velocity[2] > max_velocity) {
            current_velocity[2] = max_velocity;
            accel[2] = 0;
        } else if (current_velocity[2] < -max_velocity) {
            current_velocity[2] = -max_velocity;
            accel[2] = 0;
        }

        Eigen::Vector3d new_position = currentNode->pose.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;
        double dx = new_position.x() - currentNode->pose.x();
        double dy = new_position.y() - currentNode->pose.y();
        double heading = std::atan2(dy, dx);

        current_velocity += accel * dt;
        distance += (new_position - currentNode->pose.head<3>()).norm();

        std::shared_ptr<Node> newNode = std::make_shared<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), heading), current_velocity, accel, 0.0);
        newNode->parent = currentNode;
        newNode->cost1 = newNode->parent->cost1 + std::abs((newNode->velocity).norm() - max_velocity_modulus)*dt;
        newNode->cost2 = newNode->parent->cost2 + dt;
        newNode->time = newNode->parent->time + dt;
        newNodes.push_back(newNode);

        currentNode = newNode;
    }
}

void improved_krrt::steer_trajectory_angular(const std::shared_ptr<Node>& fromNode, double target_heading, double max_heading_velocity, double max_heading_acceleration, std::vector<std::shared_ptr<Node>>& ChangedNodes) {
    double dt = 0.1;
    double current_heading = fromNode->pose[3];
    double current_heading_velocity = fromNode->yaw_rate;
    bool heading_tolerance = max_heading_velocity*dt;

    size_t trajectory_size = ChangedNodes.size();
    for (size_t i = 0; i < trajectory_size; i++) {
        current_heading = fmod(current_heading + M_PI, 2.0 * M_PI) - M_PI;

        double heading_difference = target_heading - current_heading;
        heading_difference = fmod(heading_difference + M_PI, 2.0 * M_PI) - M_PI;

        double accel_heading = 0.0;
        if (heading_difference > 0) {
            accel_heading = max_heading_acceleration;
        } else if (heading_difference < 0) {
            accel_heading = -max_heading_acceleration;
        }

        if (current_heading_velocity > max_heading_velocity) {
            current_heading_velocity = max_heading_velocity;
            accel_heading = 0;
        } else if (current_heading_velocity < -max_heading_velocity) {
            current_heading_velocity = -max_heading_velocity;
            accel_heading = 0;
        }

        double new_heading;
        if (std::abs(heading_difference) > heading_tolerance) {
            new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
        } else {
            new_heading = current_heading;
        }
        current_heading_velocity += accel_heading * dt;

        ChangedNodes[i]->pose[3] = new_heading;
        current_heading = new_heading;
    }
}

void improved_krrt::backtrackTrajectory(const std::shared_ptr<Node>& node, const double nextBestTime, std::vector<std::shared_ptr<Node>>& fullTrajectory, std::shared_ptr<Node>& nextBestTrajectory) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        fullTrajectory.push_back(currentNode);
        currentNode = currentNode->parent;
        if (currentNode && currentNode->parent && currentNode->time >= nextBestTime && currentNode->cost2 < nextBestTrajectory->cost2) {
            nextBestTrajectory = currentNode;
        }
    }
    std::reverse(fullTrajectory.begin(), fullTrajectory.end());
}

void improved_krrt::backtrackTrajectoryKAEP(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& fullTrajectory) {
    std::shared_ptr<Node> currentNode = node;
    while (currentNode) {
        fullTrajectory.push_back(currentNode);
        currentNode = currentNode->parent;
    }
    std::reverse(fullTrajectory.begin(), fullTrajectory.end());
}
