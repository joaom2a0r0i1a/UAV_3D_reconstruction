#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <rrt_construction/libs/nanoflann.hpp>

class rrt_star {
public:

    struct Node {
        Eigen::Vector4d point;
        std::shared_ptr<Node> parent;
        double cost;
        double gain;
        double score;

        Node(const Eigen::Vector4d& p);
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::shared_ptr<Node>> data;

        void clear();

        inline void addNode(const std::shared_ptr<Node>& newNode) {
            data.push_back(newNode);
            points.push_back(newNode->point.head(3));
        }

        inline void addNodes(const std::vector<std::shared_ptr<Node>>& newNodes) {
            for (int i = 0; i < newNodes.size(); ++i) {
                data.push_back(newNodes[i]);
                points.push_back(newNodes[i]->point.head(3));
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
    
    rrt_star();

    void addKDTreeNode(std::shared_ptr<Node>& node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::shared_ptr<Node>>& nodes);

    Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);

    void computeYaw(double radius, double& result);

    void findNearest(const std::vector<std::shared_ptr<Node>>& tree, const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode);

    void findNearestKD(const Eigen::Vector3d& point, std::shared_ptr<Node>& nearestNode);

    void steer(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& result);

    void steer_parent(const std::shared_ptr<Node>& fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::shared_ptr<Node>& new_node);

    bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

    void findNearby(const std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes);

    void findNearbyKD(const std::shared_ptr<Node>& point, double radius, std::vector<std::shared_ptr<Node>>& nearbyNodes);

    void chooseParent(std::shared_ptr<Node>& point, const std::vector<std::shared_ptr<Node>>& nearbyNodes);

    //void rewire(std::vector<std::shared_ptr<Node>>& tree, const std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

    void rewire(const std::shared_ptr<Node>& new_node, std::vector<std::shared_ptr<Node>>& nearby_nodes, double radius);

    double calculateYawAngle(const std::shared_ptr<Node>& node1, const std::shared_ptr<Node>& node2);

    void backtrackPathNode(const std::shared_ptr<Node>& node, std::vector<Eigen::Vector4d>& path, std::shared_ptr<Node>& nextBestNode);

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

#endif // RRT_STAR_H

