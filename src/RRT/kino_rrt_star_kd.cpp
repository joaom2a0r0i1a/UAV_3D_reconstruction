#include "motion_planning_python/RRT/kino_rrt_star_kd.h"

// Constructors
kino_rrt_star::kino_rrt_star() {}

kino_rrt_star::Node::Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v) : point(p), velocity(v) {}

kino_rrt_star::Trajectory::Trajectory() : parent(nullptr), cost(0), gain(0), score(0) {}

kino_rrt_star::Trajectory::Trajectory(const std::shared_ptr<Node>& Node) : parent(nullptr), cost(0), gain(0), score(0) {TrajectoryPoints.push_back(Node);}

// Add and clear Nodes
void kino_rrt_star::KDTree_data::clear() {
    points.clear();
    data.clear();
}

void kino_rrt_star::addKDTreeTrajectory(std::shared_ptr<Trajectory>& Trajectory) {
    tree_data_.addTrajectory(Trajectory);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
}

void kino_rrt_star::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void kino_rrt_star::initializeKDTreeWithTrajectories(std::vector<std::shared_ptr<Trajectory>>& Trajectories) {
    tree_data_.addTrajectories(Trajectories);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
void kino_rrt_star::computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result) {
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

void kino_rrt_star::computeAccelerationSampling(double a_max, Eigen::Vector3d& result) {
    bool solutionFound = false;
    double a_x, a_y, a_z;
    while (!solutionFound) {
        a_x = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        a_y = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
        a_z = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);        
        if (Eigen::Vector3d(a_x, a_y, a_z).norm() > a_max) {
            continue;
        }
        solutionFound = true;
    }
    result = Eigen::Vector3d(a_x, a_y, a_z);
}

void kino_rrt_star::findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Trajectory>& nearestTrajectory) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestTrajectory = tree_data_.data[index];
    //nearestNode = nearestTrajectory->TrajectoryPoints.back();
}

void kino_rrt_star::steer_trajectory(const std::shared_ptr<Trajectory>& fromTrajectory, double max_velocity, bool reset_velocity, const Eigen::Vector3d& accel, double stepSize, std::shared_ptr<Trajectory>& newTrajectory) {
    double dt = 0.1;
    size_t trajectory_size = fromTrajectory->TrajectoryPoints.size();
    Eigen::Vector3d current_velocity; 
    if (!reset_velocity) {
        current_velocity = fromTrajectory->TrajectoryPoints.back()->velocity;
    } else {
        current_velocity = Eigen::Vector3d::Zero();
    }

    std::shared_ptr<Node> currentNode = fromTrajectory->TrajectoryPoints.back();
    double distance = 0.0;
    //double time = 0.0;
    //double max_time = 1.0;

    //while (time < max_time) {
    while (distance < stepSize) {
        current_velocity += accel * dt;

        if (current_velocity.norm() > max_velocity) {
            current_velocity = current_velocity.normalized() * max_velocity;
        }

        Eigen::Vector3d new_position = currentNode->point.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;
        double dx = new_position.x() - currentNode->point.x();
        double dy = new_position.y() - currentNode->point.y();
        double heading = std::atan2(dy, dx);

        std::shared_ptr<Node> newNode = std::make_shared<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), heading), current_velocity);
        newTrajectory->TrajectoryPoints.push_back(newNode);
        newTrajectory->cost += (new_position - currentNode->point.head<3>()).norm();
        
        distance += (new_position - currentNode->point.head<3>()).norm();
        //time += dt;
        currentNode = newNode;
    }
    newTrajectory->parent = fromTrajectory;
}

void kino_rrt_star::backtrackTrajectory(const std::shared_ptr<Trajectory>& trajectory, std::vector<std::shared_ptr<Trajectory>>& fullTrajectory, std::shared_ptr<Trajectory>& nextBestTrajectory) {
    std::shared_ptr<Trajectory> currentTrajectory = trajectory;
    while (currentTrajectory) {
        fullTrajectory.push_back(currentTrajectory);
        currentTrajectory = currentTrajectory->parent;
        if (currentTrajectory && currentTrajectory->parent && currentTrajectory->cost < nextBestTrajectory->cost) {
            nextBestTrajectory = currentTrajectory;
        }
    }
    std::reverse(fullTrajectory.begin(), fullTrajectory.end());
}

void kino_rrt_star::backtrackTrajectoryAEP(const std::shared_ptr<Trajectory>& trajectory, std::vector<std::shared_ptr<Trajectory>>& fullTrajectory) {
    std::shared_ptr<Trajectory> currentTrajectory = trajectory;
    while (currentTrajectory) {
        fullTrajectory.push_back(currentTrajectory);
        currentTrajectory = currentTrajectory->parent;
    }
    std::reverse(fullTrajectory.begin(), fullTrajectory.end());
}
