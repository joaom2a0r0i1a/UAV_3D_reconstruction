#ifndef RRT_STAR_H
#define RRT_STAR_H

#include <Eigen/Dense>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>
#include <functional>

#include <rrt_construction/libs/nanoflann.hpp>

class rrt_star {
  public:
    /*                 DATA TYPES                */

    struct Node {
        Eigen::Vector4d point;
        Node* parent;
        std::vector<Node*> children;
        double cost;
        double gain;
        double absolute_gain;
        double absolute_yaw;
        double score;
        double cum_gain;

        // Newly Observed Voxels
        std::unordered_map<uint64_t, uint8_t> observed_unknown_voxels;

        std::vector<float> depth_buffer;

        // Depth Pool Slot
        int depth_slot = -1;
        bool depth_in_pool = false;

        Node(const Eigen::Vector4d& p);
    };

    struct KDTree_data {
        std::vector<Eigen::Vector3d> points;
        std::vector<std::unique_ptr<Node>> data;

        void clear();

        // Add Node to Tree
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
                Node* parentNode = newNodes[i]->parent;
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
            if (dim == 0) {
                return points[idx].x();
            } else if (dim == 1) {
                return points[idx].y();
            } else {
                return points[idx].z();
            }
        }

        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /*bb*/) const {
            return false;
        }
    };

    // Define the type for the KD-tree
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, KDTree_data>, KDTree_data, 3> Tree;

    /*                CONSTRUCTION               */

    Node* addKDTreeNode(std::unique_ptr<Node> node);

    void clearKDTree();

    void initializeKDTreeWithNodes(std::vector<std::unique_ptr<Node>>& nodes);

    // Tree Nodes
    inline const std::vector<std::unique_ptr<Node>>& getNodes() const {
        return tree_data_.data;
    }

    // Edge Collision Checker
    using EdgeChecker = std::function<bool(const Eigen::Vector3d&, const Eigen::Vector3d&)>;
    void setEdgeCollisionChecker(EdgeChecker fn) {
        edge_free_ = std::move(fn);
    }

    /*                  SAMPLING                 */

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsYaw(double radius, Eigen::Vector4d& result);

    /*              NEAREST QUERIES              */

    void findNearestKD(const Eigen::Vector3d& point, Node*& nearestNode);

    void findNearbyKD(Node* point, double radius, std::vector<Node*>& nearbyNodes);

    /*                  STEERING                 */

    void steer_parent(Node* fromNode, const Eigen::Vector3d& toPoint, double stepSize, std::unique_ptr<Node>& new_node, bool fixed_step = false, double minEdge = 0.0);

    /*                RRT* WIRING                */

    void chooseParent(Node* point, const std::vector<Node*>& nearbyNodes);

    void rewire(Node* new_node, std::vector<Node*>& nearby_nodes, double radius);

    // Update Descendant Costs
    void propagateCost(Node* node);

    /*              PATH EXTRACTION              */

    void backtrackPathNode(Node* node, std::vector<Eigen::Vector4d>& path, Node*& nextBestNode);

    // Copy Best Branch
    void backtrackPathAEP(Node* node, std::vector<std::unique_ptr<Node>>& path);

    /*              STATIC UTILITIES             */

    // Sort by Tree Depth
    static void sortByDepth(std::vector<Node*>& nodes);

  private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
    EdgeChecker edge_free_;
};

#endif  // RRT_STAR_H
