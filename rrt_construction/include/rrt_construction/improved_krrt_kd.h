#ifndef IMPROVED_KINO_RRT_STAR_H
#define IMPROVED_KINO_RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <rrt_construction/libs/nanoflann.hpp>

class improved_krrt {
public:

    struct Node {
        double time;
        double gain;
        double cost1;
        double cost2;
        double score;
        double yaw_rate;
        Eigen::Vector4d pose;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        std::shared_ptr<Node> parent;
        Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v, const Eigen::Vector3d& a, const double yrate);
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::shared_ptr<Node>> data;

        void clear();

        inline void addNode(const std::shared_ptr<Node>& newNode) {
            data.push_back(newNode);
            points.push_back(newNode->pose.head(3));
        }

        inline void addNodes(const std::vector<std::shared_ptr<Node>>& newNodes) {
            for (int i = 0; i < newNodes.size(); ++i) {
                data.push_back(newNodes[i]);
                points.push_back(newNodes[i]->pose.head(3));
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
    
    improved_krrt();

    void addKDTreeNode(std::shared_ptr<Node>& node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsYaw(double radius, Eigen::Vector4d& result);
    
    void computeAccelerationSampling(double a_max, Eigen::Vector3d& result);

    void findNearestKD(const Eigen::Vector3d& position, std::shared_ptr<Node>& nearestNode);

    void steer_trajectory(const std::shared_ptr<Node>& fromNode, double max_velocity, double target_heading, Eigen::Vector3d& accel, double max_heading_velocity, double max_heading_acceleration, double stepSizeTime, std::vector<std::shared_ptr<Node>>& newNodes);

    void steer_trajectory_linear(const std::shared_ptr<Node>& fromNode, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSizeTime, std::vector<std::shared_ptr<Node>>& newNodes);

    void steer_trajectory_angular(const std::shared_ptr<Node>& fromNode, double target_heading, double max_heading_velocity, double max_heading_acceleration, std::vector<std::shared_ptr<Node>>& ChangedNodes);

    void backtrackTrajectory(const std::shared_ptr<Node>& node, const double nextBestTime, std::vector<std::shared_ptr<Node>>& fullTrajectory, std::shared_ptr<Node>& nextBestTrajectory);

    void backtrackTrajectoryKAEP(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& fullTrajectory);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
};

#endif // IMPROVED_KINO_RRT_STAR_H

