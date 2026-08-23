#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>
#include <functional>

#include <rrt_construction/libs/nanoflann.hpp>

class rrt_star {
public:

    struct Node {
        Eigen::Vector4d point;
        Node* parent;
        std::vector<Node*> children;
        double cost;
        double gain;            // configured gain: marginal (de-overlapped) when marginal_gain, else absolute
        double absolute_gain;   // own-view gain, invariant under RRT* rewire; -1.0 = not yet computed
        double absolute_yaw;    // heading for absolute_gain; command this (not point[3]) when using absolute_gain
        double score;
        double cum_gain;

        // Voxels that were unknown but are now seen (packed index -> flag); excludes global-map voxels.
        std::unordered_map<uint64_t, uint8_t> observed_unknown_voxels;

        std::vector<float> depth_buffer;

        Node(const Eigen::Vector4d& p);
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::unique_ptr<Node>> data;   // exclusive owner of all node memory

        void clear();

        // Takes ownership of newNode, wires parent<->children links, returns a non-owning observer.
        inline Node* addNode(std::unique_ptr<Node> newNode, Node* parentNode) {
            if (parentNode) {
                newNode->parent = parentNode;
                parentNode->children.push_back(newNode.get());
            }
            points.push_back(newNode->point.head(3));
            data.push_back(std::move(newNode));
            return data.back().get();
        }

        inline void addNodes(std::vector<std::unique_ptr<Node>>& newNodes) {
            for (size_t i = 0; i < newNodes.size(); ++i) {
                Node* parentNode = newNodes[i]->parent;   // links already set by the caller
                if (parentNode) {
                    parentNode->children.push_back(newNodes[i].get());
                }
                points.push_back(newNodes[i]->point.head(3));
                data.push_back(std::move(newNodes[i]));
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

    // Optional straight-edge collision check (set by the planner). Empty => skip edge checks (open envs).
    // chooseParent()/rewire() will refuse any edge for which this returns false.
    using EdgeChecker = std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>;
    void setEdgeCollisionChecker(EdgeChecker fn) { edge_free_ = std::move(fn); }

    // Takes ownership of node and returns a non-owning observer to it.
    Node* addKDTreeNode(std::unique_ptr<Node> node);

    void clearKDTree();

    // Read-only view of every node the tree owns (index 0 is the root). Non-owning use only.
    inline const std::vector<std::unique_ptr<Node>>& getNodes() const { return tree_data_.data; }

    void initializeKDTreeWithNodes(std::vector<std::unique_ptr<Node>>& nodes);

    Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);

    void computeYaw(double radius, double& result);

    void findNearest(const std::vector<std::unique_ptr<Node>>& tree, const Eigen::Vector3d& point, Node*& nearestNode);

    void findNearestKD(const Eigen::Vector3d& point, Node*& nearestNode);

    void steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& result);

    void steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& new_node, bool fixed_step = false, double minEdge = 0.0);

    bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

    void findNearby(const std::vector<std::unique_ptr<Node>>& tree, Node* point, double radius, std::vector<Node*>& nearbyNodes);

    void findNearbyKD(Node* point, double radius, std::vector<Node*>& nearbyNodes);

    void findNearbyKDRadius(const Node* point, double radius, std::vector<Node*>& nearbyNodes);

    void chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

    void rewire(Node* new_node, std::vector<Node*>& nearby_nodes, double radius);

    // After a re-parent, recompute descendant costs by walking the children links.
    void propagateCost(Node* node);

    double calculateYawAngle(Node* node1, Node* node2);

    void backtrackPathNode(Node* node, std::vector<Eigen::Vector4d>& path, Node*& nextBestNode);

    // Fills path with owning deep copies of the branch (root -> node), so it outlives clearKDTree().
    void backtrackPathAEP(Node* node, std::vector<std::unique_ptr<Node>>& path);

    bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                double dim_x, double dim_y, double dim_z, int max_iter,
                double step_size, double radius, double tolerance,
                std::vector<std::unique_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
    EdgeChecker edge_free_;   // straight-edge collision test; empty => edges are not checked
};

#endif // RRT_STAR_H

