#include "rrt_construction/kino_rrt_star_kd.h"

// Constructors
kino_rrt_star::kino_rrt_star() {}

kino_rrt_star::Node::Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a) : point(p), velocity(v), acceleration(a) {}

kino_rrt_star::Trajectory::Trajectory() : parent(nullptr), cost(0.0), gain(0.0), score(0.0), cost1(0.0), cost2(0.0) {}

kino_rrt_star::Trajectory::Trajectory(std::unique_ptr<Node> Node) : parent(nullptr), cost(0.0), gain(0.0), score(0.0), cost1(0.0), cost2(0.0) {TrajectoryPoints.push_back(std::move(Node));}

std::unique_ptr<kino_rrt_star::Trajectory> kino_rrt_star::Trajectory::clone() const {
    auto copy = std::make_unique<Trajectory>();
    copy->parent = nullptr;
    copy->cost = cost;
    copy->gain = gain;
    copy->score = score;
    copy->cost1 = cost1;
    copy->cost2 = cost2;
    for (const auto& node : TrajectoryPoints) {
        copy->TrajectoryPoints.push_back(std::make_unique<Node>(*node));
    }
    return copy;
}

// Add and clear Nodes
void kino_rrt_star::KDTree_data::clear() {
    // Clear the raw coordinate mirror FIRST, then release trajectory ownership. Every
    // structural link (Trajectory* parent and std::vector<Trajectory*> children) is
    // non-owning, so destroying `data` frees all trajectory memory instantly with zero
    // risk of cyclic retention or double-free.
    points.clear();
    data.clear();
}

kino_rrt_star::Trajectory* kino_rrt_star::addKDTreeTrajectory(std::unique_ptr<Trajectory> Trajectory) {
    // The planners set Trajectory->parent before insertion (steer_trajectory*), so
    // forward that parent to addTrajectory to keep the parent <-> children raw links
    // aligned as ownership is transferred into the flat `data` vector.
    kino_rrt_star::Trajectory* parentTrajectory = Trajectory->parent;
    kino_rrt_star::Trajectory* observer = tree_data_.addTrajectory(std::move(Trajectory), parentTrajectory);
    kdtree_->addPoints(tree_data_.points.size() - 1, tree_data_.points.size() - 1);
    return observer;
}

void kino_rrt_star::clearKDTree() {
    tree_data_.clear();
    kdtree_ = std::unique_ptr<Tree>(new Tree(3, tree_data_));
}

void kino_rrt_star::initializeKDTreeWithTrajectories(std::vector<std::unique_ptr<Trajectory>>& Trajectories) {
    tree_data_.addTrajectories(Trajectories);
    kdtree_->addPoints(0, tree_data_.points.size() - 1);
}

// RRT implementation
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
        if (Eigen::Vector2d(a_x, a_y).norm() > a_max) {
            continue;
        }
        solutionFound = true;
    }
    a_z = 2.0 * a_max * (((double) rand()) / ((double) RAND_MAX) - 0.5);
    result = Eigen::Vector3d(a_x, a_y, a_z);
}

void kino_rrt_star::findNearestKD(const Eigen::Vector3d& point, Trajectory*& nearestTrajectory) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    kdtree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearestTrajectory = tree_data_.data[index].get();
}

void kino_rrt_star::steer_trajectory(Trajectory* fromTrajectory, double max_velocity, bool reset_velocity, double target_heading, Eigen::Vector3d& accel, double max_heading_velocity, double max_heading_acceleration, double stepSize, std::unique_ptr<Trajectory>& newTrajectory) {
    double dt = 0.1;
    size_t trajectory_size = fromTrajectory->TrajectoryPoints.size();
    Eigen::Vector3d current_velocity;
    current_velocity = fromTrajectory->TrajectoryPoints.back()->velocity;
    /*if (!reset_velocity) {
        current_velocity = fromTrajectory->TrajectoryPoints.back()->velocity;
    } else {
        current_velocity = Eigen::Vector3d::Zero();
    }*/

    Node* currentNode = fromTrajectory->TrajectoryPoints.back().get();
    double distance = 0.0;

    double current_heading = fromTrajectory->TrajectoryPoints.back()->point[3];
    double current_heading_velocity = 0.0;
    double accel_heading;

    double heading_difference = fmod((target_heading - current_heading + M_PI), (2*M_PI)) - M_PI;
    if (heading_difference < -M_PI) {
        heading_difference = heading_difference + 2*M_PI;
    }

    if (heading_difference > 0) {
        accel_heading = max_heading_acceleration;
    } else if (heading_difference < 0) {
        accel_heading = -max_heading_acceleration;
    } else if (heading_difference == 0) {
        accel_heading = 0;
    }

    int num_points = 0;
    double max_velocity_modulus = sqrt(pow(max_velocity, 2) + pow(max_velocity, 2));
    double velocity_sum = 0.0;
    double time = 0.0;
    double max_time = 2.0;

    bool in_heading_tolerance = false;
    bool heading_tolerance = max_heading_velocity*dt;

    newTrajectory->parent = fromTrajectory;
    while (time < max_time) {
    //while (distance < stepSize) {
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

        Eigen::Vector3d new_position = currentNode->point.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;

        if (current_heading_velocity > max_heading_velocity) {
            current_heading_velocity = max_heading_velocity;
            accel_heading = 0;
        } else if (current_heading_velocity < -max_heading_velocity) {
            current_heading_velocity = -max_heading_velocity;
            accel_heading = 0;
        }

        double new_heading;
        if(heading_difference > 0) {
            if (target_heading - current_heading > heading_tolerance) {
                new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
            } else {
                new_heading = current_heading;
            }
        } else {
            if (target_heading - current_heading < -heading_tolerance) {
                new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
            } else {
                new_heading = current_heading;
            }
        }

        current_velocity += accel * dt;
        current_heading_velocity += accel_heading * dt;

        std::unique_ptr<Node> newNode = std::make_unique<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), new_heading), current_velocity, accel);
        Node* newNodePtr = newNode.get();
        newTrajectory->TrajectoryPoints.push_back(std::move(newNode));
        //newTrajectory->cost += dt;
        //newTrajectory->cost += (new_position - currentNode->point.head<3>()).norm();
        //newTrajectory->cost += (max_velocity_modulus - current_velocity.norm());
        newTrajectory->cost2 += (new_position - currentNode->point.head<3>()).norm();
        velocity_sum += current_velocity.norm();

        distance += (new_position - currentNode->point.head<3>()).norm();
        time += dt;
        currentNode = newNodePtr;
        current_heading = new_heading;
        num_points += 1;
    }
    //newTrajectory->cost = newTrajectory->cost / num_points;
    double average_velocity = velocity_sum / num_points;
    newTrajectory->cost1 = std::abs(max_velocity_modulus - average_velocity);
}

void kino_rrt_star::steer_trajectory_linear(Trajectory* fromTrajectory, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSize, std::unique_ptr<Trajectory>& newTrajectory) {
    double dt = 0.1;
    size_t trajectory_size = fromTrajectory->TrajectoryPoints.size();
    Eigen::Vector3d current_velocity;
    current_velocity = fromTrajectory->TrajectoryPoints.back()->velocity;
    /*if (!reset_velocity) {
        current_velocity = fromTrajectory->TrajectoryPoints.back()->velocity;
    } else {
        current_velocity = Eigen::Vector3d::Zero();
    }*/

    Node* currentNode = fromTrajectory->TrajectoryPoints.back().get();
    double distance = 0.0;
    int num_points = 0;
    double velocity_sum = 0.0;
    double max_velocity_modulus = sqrt(pow(max_velocity, 2) + pow(max_velocity, 2));
    double time = 0.0;
    double max_time = 2.0;

    newTrajectory->parent = fromTrajectory;
    while (time < max_time) {
    //while (distance < stepSize) {
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

        Eigen::Vector3d new_position = currentNode->point.head<3>() + current_velocity*dt + 0.5*accel*dt*dt;
        double dx = new_position.x() - currentNode->point.x();
        double dy = new_position.y() - currentNode->point.y();
        double heading = std::atan2(dy, dx);

        current_velocity += accel * dt;

        std::unique_ptr<Node> newNode = std::make_unique<Node>(Eigen::Vector4d(new_position.x(), new_position.y(), new_position.z(), heading), current_velocity, accel);
        Node* newNodePtr = newNode.get();
        newTrajectory->TrajectoryPoints.push_back(std::move(newNode));
        //newTrajectory->cost += dt;
        //newTrajectory->cost += (new_position - currentNode->point.head<3>()).norm();
        //newTrajectory->cost += (max_velocity_modulus - current_velocity.norm());
        newTrajectory->cost2 += (new_position - currentNode->point.head<3>()).norm();
        velocity_sum += current_velocity.norm();

        distance += (new_position - currentNode->point.head<3>()).norm();
        time += dt;
        currentNode = newNodePtr;
        num_points += 1;
    }
    //newTrajectory->cost = newTrajectory->cost / num_points;
    double average_velocity = velocity_sum / num_points;
    newTrajectory->cost1 += std::abs(max_velocity_modulus - average_velocity);
}

void kino_rrt_star::steer_trajectory_angular(Trajectory* fromTrajectory, double target_heading, double max_heading_velocity, double max_heading_acceleration, Trajectory* toChangeTrajectory) {
    double dt = 0.1;
    double current_heading = fromTrajectory->TrajectoryPoints.back()->point[3];
    double current_heading_velocity = 0.0;
    double accel_heading;

    double heading_difference = fmod((target_heading - current_heading + M_PI), (2*M_PI)) - M_PI;
    if (heading_difference < -M_PI) {
        heading_difference = heading_difference + 2*M_PI;
    }

    if (heading_difference > 0) {
        accel_heading = max_heading_acceleration;
    } else if (heading_difference < 0) {
        accel_heading = -max_heading_acceleration;
    } else if (heading_difference == 0) {
        accel_heading = 0;
    }

    //toChangeTrajectory->cost2 = abs(heading_difference);

    bool in_heading_tolerance = false;
    bool heading_tolerance = max_heading_velocity*dt;
    size_t trajectory_size = toChangeTrajectory->TrajectoryPoints.size();
    for (int i = 0; i < trajectory_size; i++) {
        if (current_heading_velocity > max_heading_velocity) {
            current_heading_velocity = max_heading_velocity;
            accel_heading = 0;
        } else if (current_heading_velocity < -max_heading_velocity) {
            current_heading_velocity = -max_heading_velocity;
            accel_heading = 0;
        }

        double new_heading;
        if(heading_difference > 0) {
            if (target_heading - current_heading > heading_tolerance) {
                new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
            } else {
                new_heading = current_heading;
            }
        } else {
            if (target_heading - current_heading < -heading_tolerance) {
                new_heading = current_heading + current_heading_velocity*dt + 0.5*accel_heading*dt*dt;
            } else {
                new_heading = current_heading;
            }
        }
        current_heading_velocity += accel_heading * dt;

        toChangeTrajectory->TrajectoryPoints[i]->point[3] = new_heading;
        current_heading = new_heading;
    }
}

void kino_rrt_star::backtrackTrajectory(Trajectory* trajectory, std::vector<std::unique_ptr<Trajectory>>& fullTrajectory, Trajectory*& nextBestTrajectory) {
    // Collect the branch (trajectory -> root) as observers, tracking the lowest
    // cost2 interior trajectory for nextBestTrajectory.
    std::vector<Trajectory*> chain;
    Trajectory* bestInterior = nullptr;
    for (Trajectory* currentTrajectory = trajectory; currentTrajectory; currentTrajectory = currentTrajectory->parent) {
        chain.push_back(currentTrajectory);
        Trajectory* parent = currentTrajectory->parent;
        if (parent && parent->parent) {
            if (!bestInterior || parent->cost2 < bestInterior->cost2) {
                bestInterior = parent;
            }
        }
    }
    std::reverse(chain.begin(), chain.end());   // root first

    // Emit owning deep copies, relinking parents amongst the clones so the branch
    // is self-contained and survives clearKDTree(). nextBestTrajectory points at
    // the clone matching the chosen interior trajectory.
    fullTrajectory.clear();
    Trajectory* prev_clone = nullptr;
    for (Trajectory* original : chain) {
        std::unique_ptr<Trajectory> copy = original->clone();
        copy->parent = prev_clone;
        if (original == bestInterior) {
            nextBestTrajectory = copy.get();
        }
        prev_clone = copy.get();
        fullTrajectory.push_back(std::move(copy));
    }
}

void kino_rrt_star::backtrackTrajectoryAEP(Trajectory* trajectory, std::vector<std::unique_ptr<Trajectory>>& fullTrajectory) {
    // Collect the branch (trajectory -> root) as observers first.
    std::vector<Trajectory*> chain;
    for (Trajectory* currentTrajectory = trajectory; currentTrajectory; currentTrajectory = currentTrajectory->parent) {
        chain.push_back(currentTrajectory);
    }
    std::reverse(chain.begin(), chain.end());   // root first

    // Emit owning deep copies, relinking parents amongst the clones so the branch
    // is self-contained and survives clearKDTree().
    fullTrajectory.clear();
    Trajectory* prev_clone = nullptr;
    for (Trajectory* original : chain) {
        std::unique_ptr<Trajectory> copy = original->clone();
        copy->parent = prev_clone;
        prev_clone = copy.get();
        fullTrajectory.push_back(std::move(copy));
    }
}
