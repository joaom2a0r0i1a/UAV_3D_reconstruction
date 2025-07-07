#ifndef KINO_RRT_STAR_H
#define KINO_RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <rrt_construction/libs/nanoflann.hpp>

class kino_rrt_star {
public:

    struct Node {
        Eigen::Vector4d point;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a);
    };

    struct Trajectory {
        std::vector<std::shared_ptr<Node>> TrajectoryPoints;
        std::shared_ptr<Trajectory> parent;
        double cost;
        double gain;
        double score;

        double cost1;
        double cost2;

        // Constructors
        Trajectory();
        Trajectory(const std::shared_ptr<Node>& Node);

        // Method to add a node to the trajectory
        void addNode(const std::shared_ptr<Node>& node) {
            TrajectoryPoints.push_back(node);
        }

        void clear() {
            TrajectoryPoints.clear();
            cost = 0.0;
            gain = 0.0;
            score = 0.0;

            cost1 = 0.0;
            cost2 = 0.0;
        }
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::shared_ptr<Trajectory>> data;

        void clear();

        inline void addTrajectory(const std::shared_ptr<Trajectory>& newTrajectory) {
            data.push_back(newTrajectory);
            points.push_back(newTrajectory->TrajectoryPoints.back()->point.head(3));
        }

        inline void addTrajectories(const std::vector<std::shared_ptr<Trajectory>>& newTrajectories) {
            for (int i = 0; i < newTrajectories.size(); ++i) {
                data.push_back(newTrajectories[i]);
                points.push_back(newTrajectories[i]->TrajectoryPoints.back()->point.head(3));
            }
        }

        inline size_t kdtree_get_point_count() const {
            return points.size();
        }

        inline double kdtree_get_pt(const size_t idx, int dim) const {
            if (dim == 0) return points[idx].x();
            else if (dim == 1) return points[idx].y();
            else return points[idx].z();
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }
    };

    // Define the type for the KD-tree
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, KDTree_data>, KDTree_data, 3> Tree;
    
    kino_rrt_star();

    void addKDTreeTrajectory(std::shared_ptr<Trajectory>& newTrajectory);

    void clearKDTree();

    void initializeKDTreeWithTrajectories(std::vector<std::shared_ptr<Trajectory>>& Trajectories);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);
    
    void computeAccelerationSampling(double a_max, Eigen::Vector3d& result);

    void findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Trajectory>& nearestTrajectory);

    void steer_trajectory(const std::shared_ptr<Trajectory>& fromTrajectory, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSize, std::shared_ptr<Trajectory>& newTrajectory);

    void steer_trajectory(const std::shared_ptr<Trajectory>& fromTrajectory, double max_velocity, bool reset_velocity, double target_heading, Eigen::Vector3d& accel, double max_heading_velocity, double max_heading_acceleration, double stepSize, std::shared_ptr<Trajectory>& newTrajectory);

    void steer_trajectory_linear(const std::shared_ptr<Trajectory>& fromTrajectory, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSize, std::shared_ptr<Trajectory>& newTrajectory);

    void steer_trajectory_angular(const std::shared_ptr<Trajectory>& fromTrajectory, double target_heading, double max_heading_velocity, double max_heading_acceleration, std::shared_ptr<Trajectory>& toChangeTrajectory);

    void backtrackTrajectory(const std::shared_ptr<Trajectory>& trajectory, std::vector<std::shared_ptr<Trajectory>>& fullTrajectory, std::shared_ptr<Trajectory>& nextBestTrajectory);

    void backtrackTrajectoryAEP(const std::shared_ptr<Trajectory>& trajectory, std::vector<std::shared_ptr<Trajectory>>& fullTrajectory);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
};

#endif // KINO_RRT_STAR_H

