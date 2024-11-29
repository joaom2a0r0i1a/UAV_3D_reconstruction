#ifndef KD_TREE_GAIN_H
#define KD_TREE_GAIN_H

#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <algorithm>
#include <limits>
#include <vector>
#include <memory>

#include <libs/nanoflann.hpp>

class kd_tree_gain {
public:
    struct KDTree_point_gain {
        std::vector<Eigen::Vector3d> points;
        std::vector<double> gains;

        void clear();

        inline void add_Point(const Eigen::Vector3d add_point) {
            points.push_back(add_point);
        }

        inline void add_Points(const std::vector<Eigen::Vector3d> add_points) {
            for (int i = 0; i < add_points.size(); ++i) {
                points.push_back(add_points[i]);
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
    typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, KDTree_point_gain>, KDTree_point_gain, 3> Tree_points;

    kd_tree_gain();

    void addKDTreePoint(Eigen::Vector3d point);

    void clearKDTreePoints();

    void initializeKDTreeWithPoints(std::vector<Eigen::Vector3d> points);

    void findNearestKDPoint(const Eigen::Vector3d& point, Eigen::Vector3d& nearest);

private:
    std::unique_ptr<Tree_points> goal_tree_;
    KDTree_point_gain tree_points_;
};

#endif // KD_TREE_GAIN_H