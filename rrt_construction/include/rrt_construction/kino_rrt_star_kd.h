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
        std::vector<std::unique_ptr<Node>> TrajectoryPoints;   // owns the trajectory nodes
        Trajectory* parent;                                    // non-owning observer of the parent trajectory
        std::vector<Trajectory*> children;                     // non-owning observers of downstream branches
        double cost;
        double gain;
        double score;

        double cost1;
        double cost2;

        // Constructors
        Trajectory();
        Trajectory(std::unique_ptr<Node> Node);

        // Method to add a node to the trajectory (takes ownership)
        void addNode(std::unique_ptr<Node> node) {
            TrajectoryPoints.push_back(std::move(node));
        }

        // Deep copy of this trajectory (owns fresh node copies); parent/children are left empty.
        std::unique_ptr<Trajectory> clone() const;

        void clear() {
            // Zero the raw structural links first, then drop the owned nodes.
            parent = nullptr;
            children.clear();
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
        std::vector<std::unique_ptr<Trajectory>> data;   // exclusive owner of all trajectory memory

        void clear();

        // Takes exclusive ownership of newTrajectory and returns a non-owning observer.
        // The raw parent <-> children links are aligned BEFORE ownership is transferred
        // into the flat `data` vector, so they are valid the instant it lands in the tree.
        inline Trajectory* addTrajectory(std::unique_ptr<Trajectory> newTrajectory, Trajectory* parentTrajectory) {
            if (parentTrajectory) {
                newTrajectory->parent = parentTrajectory;
                parentTrajectory->children.push_back(newTrajectory.get());
            }
            points.push_back(newTrajectory->TrajectoryPoints.back()->point.head(3));
            data.push_back(std::move(newTrajectory));
            return data.back().get();
        }

        inline void addTrajectories(std::vector<std::unique_ptr<Trajectory>>& newTrajectories) {
            for (size_t i = 0; i < newTrajectories.size(); ++i) {
                Trajectory* parentTrajectory = newTrajectories[i]->parent;   // links already set by the caller
                if (parentTrajectory) {
                    parentTrajectory->children.push_back(newTrajectories[i].get());
                }
                points.push_back(newTrajectories[i]->TrajectoryPoints.back()->point.head(3));
                data.push_back(std::move(newTrajectories[i]));
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

    // Takes ownership of newTrajectory and returns a non-owning observer to it.
    Trajectory* addKDTreeTrajectory(std::unique_ptr<Trajectory> newTrajectory);

    void clearKDTree();

    void initializeKDTreeWithTrajectories(std::vector<std::unique_ptr<Trajectory>>& Trajectories);

    void computeSamplingDimensions(double radius, Eigen::Vector3d& result);

    void computeSamplingDimensionsNBV(double radius, Eigen::Vector4d& result);

    void computeAccelerationSampling(double a_max, Eigen::Vector3d& result);

    void findNearestKD(const Eigen::Vector3d& point, Trajectory*& nearestTrajectory);

    void steer_trajectory(Trajectory* fromTrajectory, double max_velocity, bool reset_velocity, double target_heading, Eigen::Vector3d& accel, double max_heading_velocity, double max_heading_acceleration, double stepSize, std::unique_ptr<Trajectory>& newTrajectory);

    void steer_trajectory_linear(Trajectory* fromTrajectory, double max_velocity, bool reset_velocity, Eigen::Vector3d& accel, double stepSize, std::unique_ptr<Trajectory>& newTrajectory);

    void steer_trajectory_angular(Trajectory* fromTrajectory, double target_heading, double max_heading_velocity, double max_heading_acceleration, Trajectory* toChangeTrajectory);

    // Fills fullTrajectory with owning deep copies of the branch (root -> trajectory).
    // The clones keep their parent links amongst themselves so the branch is
    // self-contained and outlives clearKDTree(). Callers store this in their owning
    // best_branch cache.
    void backtrackTrajectory(Trajectory* trajectory, std::vector<std::unique_ptr<Trajectory>>& fullTrajectory, Trajectory*& nextBestTrajectory);

    void backtrackTrajectoryAEP(Trajectory* trajectory, std::vector<std::unique_ptr<Trajectory>>& fullTrajectory);

private:
    std::unique_ptr<Tree> kdtree_;
    KDTree_data tree_data_;
};

#endif // KINO_RRT_STAR_H

