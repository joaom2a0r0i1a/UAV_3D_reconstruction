#include "kd_tree_gain.h"

// Constructors
kd_tree_gain::kd_tree_gain() {}

// Add and clear Nodes
void kd_tree_gain::KDTree_point_gain::clear() {
    points.clear();
}

void kd_tree_gain::addKDTreePoint(Eigen::Vector3d point) {
    tree_points_.add_Point(point);
    goal_tree_->addPoints(tree_points_.points.size() - 1, tree_points_.points.size() - 1);
}

void kd_tree_gain::clearKDTreePoints() {
    tree_points_.clear();
    goal_tree_ = std::unique_ptr<Tree_points>(new Tree_points(3, tree_points_));
}

void kd_tree_gain::initializeKDTreeWithPoints(std::vector<Eigen::Vector3d> points) {
    tree_points_.add_Points(points);
    goal_tree_->addPoints(0, tree_points_.points.size() - 1);
}

void kd_tree_gain::findNearestKDPoint(const Eigen::Vector3d& point, Eigen::Vector3d& nearest) {
    double query_pt[3] = {point.x(), point.y(), point.z()};
    nanoflann::KNNResultSet<double> resultSet(1);
    size_t index;
    double out_dist_sqr;
    resultSet.init(&index, &out_dist_sqr);
    goal_tree_->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    nearest = tree_points_.points[index];
}