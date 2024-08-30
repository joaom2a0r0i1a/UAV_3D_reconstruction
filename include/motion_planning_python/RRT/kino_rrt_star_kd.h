#ifndef KINO_RRT_STAR_H
#define KINO_RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <motion_planning_python/libs/nanoflann.hpp>

class kino_rrt_star {
public:

    struct Node {
        Eigen::Vector4d point;
        Eigen::Vector3d velocity;
        std::shared_ptr<Node> parent;
        double cost;
        double gain;
        double score;

        Node(const Eigen::Vector4d& p, const Eigen::Vector3d& v);
    };

    struct Trajectory {
        std::vector<std::shared_ptr<Node>> TrajectoryPoints;
        double total_cost; 

        // Constructor
        Trajectory();

        // Method to add a node to the trajectory
        void addNode(const std::shared_ptr<Node>& node) {
            TrajectoryPoints.push_back(node);
            total_cost += node->cost;
        }

        void clear() {
            TrajectoryPoints.clear();
            total_cost = 0.0;
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

    void addKDTreeNode(std::shared_ptr<Node>& node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes);

    Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);

    void computeYaw(double radius, double& result);
    
    void computeAccelerationSampling(double a_max, Eigen::Vector3d& result);

    void findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode);

    void steer_trajectory(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, const Eigen::Vector3d& accel, double stepSize, std::vector<std::shared_ptr<Node>>& new_node_trajectory);

    bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles)

    void findNearbyKD(const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes);

    void chooseParent(std::shared_ptr<Node>& point, const std::vector<std::shared_ptr<Node>>& nearbyNodes);

    void rewire(const std::shared_ptr<Node>& new_node, std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

    double calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2);

    void backtrackTrajectory(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& path, std::shared_ptr<Node>& nextBestNode);

    void backtrackPathAEP(const std::shared_ptr<Node>& node, std::vector<std::shared_ptr<Node>>& path);

    bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                double dim_x, double dim_y, double dim_z, int max_iter,
                double step_size, double radius, double tolerance,
                std::vector<std::shared_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
};

#endif // KINO_RRT_STAR_H

