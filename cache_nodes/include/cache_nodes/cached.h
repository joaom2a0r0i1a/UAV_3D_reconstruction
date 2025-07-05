#ifndef CACHED_H
#define CACHED_H

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

class Cached {
public:
    Cached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

private:
    void GetTransformation();
    void callbackUavState(const mrs_msgs::UavState::ConstPtr& msg);
    void timerReevaluate(const ros::TimerEvent&);
    void callbackGain(const cache_nodes::Node::ConstPtr& msg);
    bool callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res);

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
    ros::Timer reevaluate_timer;

    double g_zero;
    double range;
    int num_yaw_samples;

    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;

    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;
    double x, y, z;
};

#endif // CACHED_H
