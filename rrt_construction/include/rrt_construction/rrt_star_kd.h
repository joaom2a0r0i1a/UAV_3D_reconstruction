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
        Node* parent;                  // non-owning observer of the parent node
        std::vector<Node*> children;   // non-owning observers of the child nodes
        double cost;
        double gain;
        double score;

        // THE MAP: Stores indices of voxels that were UNKNOWN but are now SEEN.
        // We only add to this if the voxel is NOT in the global map (flat_map).
        // Key: Packed Index, Value: 1 (Just a flag)
        std::unordered_map<uint64_t, uint8_t> observed_unknown_voxels;

        std::vector<float> depth_buffer;

        Node(const Eigen::Vector4d& p);
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::unique_ptr<Node>> data;   // exclusive owner of all node memory

        void clear();

        // Takes exclusive ownership of newNode and returns a non-owning observer.
        // The raw structural links (parent <-> children) are wired BEFORE ownership
        // is transferred into the flat `data` vector, so they are valid the instant
        // the node lands in the tree.
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

    // Takes ownership of node and returns a non-owning observer to it.
    Node* addKDTreeNode(std::unique_ptr<Node> node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::unique_ptr<Node>>& nodes);

    Eigen::Vector3d sampleSpace(double dim_x, double dim_y, double dim_z);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);

    void computeYaw(double radius, double& result);

    void findNearest(const std::vector<std::unique_ptr<Node>>& tree, const Eigen::Vector3d& point, Node*& nearestNode);

    void findNearestKD(const Eigen::Vector3d& point, Node*& nearestNode);

    void steer(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& result);

    void steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& new_node);

    bool collides(const Eigen::Vector3d& point, const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles);

    void findNearby(const std::vector<std::unique_ptr<Node>>& tree, Node* point, double radius, std::vector<Node*>& nearbyNodes);

    void findNearbyKD(Node* point, double radius, std::vector<Node*>& nearbyNodes);

    void chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

    //void rewire(std::vector<std::unique_ptr<Node>>& tree, Node* new_node, const std::vector<Node*>& nearby_nodes, double radius);

    void rewire(Node* new_node, std::vector<Node*>& nearby_nodes, double radius);

    // Downward traversal helper: after a re-parent, recompute descendant costs by
    // walking the raw `children` links. Never searches the owning `data` vector.
    void propagateCost(Node* node);

    double calculateYawAngle(Node* node1, Node* node2);

    void backtrackPathNode(Node* node, std::vector<Eigen::Vector4d>& path, Node*& nextBestNode);

    // Fills path with owning deep copies of the branch (root -> node). The clones
    // keep their parent links amongst themselves so the branch is self-contained and
    // outlives clearKDTree(). Callers store this in their owning best_branch cache.
    void backtrackPathAEP(Node* node, std::vector<std::unique_ptr<Node>>& path);

    bool rrtStar(const Eigen::Vector4d& start, const Eigen::Vector4d& goal,
                const std::vector<std::pair<Eigen::Vector3d, double>>& obstacles,
                double dim_x, double dim_y, double dim_z, int max_iter,
                double step_size, double radius, double tolerance,
                std::vector<std::unique_ptr<Node>>& tree, std::vector<Eigen::Vector4d>& path);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
};

#endif // RRT_STAR_H

