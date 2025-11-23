#ifndef GEO_CACHED_H
#define GEO_CACHED_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/UavState.h>

#include <cache_nodes/Node.h>
#include <cache_nodes/BestNode.h>
#include <cache_nodes/BestNodeResponse.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <minkindr_conversions/kindr_msg.h>
#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/gain_evaluator.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <functional>
#include <geometry_msgs/Point.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, cache_nodes::Node> RTreeValue;

struct GridKey {
    int ix, iy, iz;
    bool operator==(const GridKey& other) const noexcept {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

struct GridKeyHasher {
    std::size_t operator()(const GridKey& k) const noexcept {
        return ((std::hash<int>()(k.ix) ^ (std::hash<int>()(k.iy) << 1)) >> 1)
             ^ (std::hash<int>()(k.iz) << 1);
    }
};

class GEOCached {
public:
    GEOCached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    void GetTransformation();
    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isCollisionFree(const Eigen::Vector3d& pos) const;

    GridKey computeGridKey(const Eigen::Vector3d& pos) const;
    void sampleDynamicCandidates(double x_min, double x_max,
                              double y_min, double y_max,
                              double z_min, double z_max,
                              double spacing);

    void timerSampleCandidates(const ros::TimerEvent&);
    void timerReevaluate(const ros::TimerEvent&);

    void callbackUavState(const mrs_msgs::UavState::ConstPtr& msg);
    void callbackGain(const cache_nodes::Node::ConstPtr& msg);
    bool callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    GainEvaluator evaluator;

    voxblox::EsdfServer voxblox_server_;
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;

    std::string frame_id;
    std::string body_frame_id;
    std::string camera_frame_id;
    std::string ns;

    // Transformer
    std::unique_ptr<mrs_lib::Transformer> transformer_;

    geometry_msgs::TransformStamped T_C_B_message;
    voxblox::Transformation T_C_B;

    ros::ServiceServer ss_best_node;
    ros::Subscriber sub_gain;
    ros::Subscriber sub_uav_state;

    ros::Timer sample_timer;
    ros::Timer reevaluate_timer;

    double g_zero;
    double range;
    int num_yaw_samples;
    double uav_radius;

    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;

    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
    double spacing;

    std::unordered_map<GridKey, cache_nodes::Node, GridKeyHasher> candidate_map;

    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;
    double x, y, z;
};

#endif // GEO_CACHED_H
