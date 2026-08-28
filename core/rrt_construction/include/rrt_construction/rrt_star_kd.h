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

    /* ----------------------- Data types ----------------------- */

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

        // Depth-pool slot = append index in KDTree_data::data (unique per live node, resets on tree clear).
        int  depth_slot   = -1;
        bool depth_in_pool = false;

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
            newNode->depth_slot = (int)data.size();
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
                newNodes[i]->depth_slot = (int)data.size();
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


    /* ----------------------- Construction & tree mutation ----------------------- */

    // Takes ownership of node and returns a non-owning observer to it.
    Node* addKDTreeNode(std::unique_ptr<Node> node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::unique_ptr<Node>>& nodes);

    // Read-only view of every node the tree owns (index 0 is the root). Non-owning use only.
    inline const std::vector<std::unique_ptr<Node>>& getNodes() const { return tree_data_.data; }

    // Optional straight-edge collision check (set by the planner). Empty => skip edge checks (open envs).
    // chooseParent()/rewire() will refuse any edge for which this returns false.
    using EdgeChecker = std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>;
    void setEdgeCollisionChecker(EdgeChecker fn) { edge_free_ = std::move(fn); }


    /* ----------------------- Sampling ----------------------- */

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);


    /* ----------------------- Nearest / nearby queries ----------------------- */

    void findNearestKD(const Eigen::Vector3d& point, Node*& nearestNode);

    void findNearbyKD(Node* point, double radius, std::vector<Node*>& nearbyNodes);


    /* ----------------------- Steering ----------------------- */

    void steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& new_node, bool fixed_step = false, double minEdge = 0.0);


    /* ----------------------- RRT* wiring ----------------------- */

    void chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

    void rewire(Node* new_node, std::vector<Node*>& nearby_nodes, double radius);

    // After a re-parent, recompute descendant costs by walking the children links.
    void propagateCost(Node* node);


    /* ----------------------- Path extraction ----------------------- */

    void backtrackPathNode(Node* node, std::vector<Eigen::Vector4d>& path, Node*& nextBestNode);

    // Fills path with owning deep copies of the branch (root -> node), so it outlives clearKDTree().
    void backtrackPathAEP(Node* node, std::vector<std::unique_ptr<Node>>& path);


    /* ----------------------- Static utilities ----------------------- */

    // Stable shallow-first order (parents before children); walks parent pointers.
    static void sortByDepth(std::vector<Node*>& nodes);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
    EdgeChecker edge_free_;   // straight-edge collision test; empty => edges are not checked
};

#endif // RRT_STAR_H
