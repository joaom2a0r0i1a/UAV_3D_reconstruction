#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/UavState.h>

#include <cache_nodes/Node.h>
#include <cache_nodes/Query.h>
#include <cache_nodes/QueryResponse.h>
#include <cache_nodes/BestNode.h>
#include <cache_nodes/BestNodeResponse.h>
#include <cache_nodes/Reevaluate.h>

#include <mrs_lib/param_loader.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <minkindr_conversions/kindr_msg.h>

#include <rrt_kd.h>
#include <evaluator.h>
#include <RTree.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <functional>
#include <geometry_msgs/Point.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <unordered_map>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, cache_nodes::Node> RTreeValue;

class MultiCached {
public:
    MultiCached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_) {
        ss_best_node = nh_private_.advertiseService("best_node_in", &MultiCached::callbackBestNode, this);

        /* Parameter loading */
        mrs_lib::ParamLoader param_loader(nh_private_, "multicached");

        // Frames, Coordinates and Dimensions
        param_loader.loadParam("frame_id", frame_id);
        param_loader.loadParam("body/frame_id", body_frame_id);

        param_loader.loadParam("visualize/mean", visualize_mean, false);
        param_loader.loadParam("visualize/sigma", visualize_sigma, false);
        param_loader.loadParam("visualize/pts", visualize_pts, false);
        param_loader.loadParam("visualize/resolution", resolution, 1);
        param_loader.loadParam("local_planning/g_zero", g_zero, 2.0);
        param_loader.loadParam("local_planning/yaw_samples", num_yaw_samples, 10);

        // Camera
        param_loader.loadParam("camera/h_fov", horizontal_fov);
        param_loader.loadParam("camera/width", resolution_x);
        param_loader.loadParam("camera/height", resolution_y);
        param_loader.loadParam("camera/min_distance", min_distance);
        param_loader.loadParam("camera/max_distance", max_distance);
        param_loader.loadParam("camera/frame_id", camera_frame_id);

        sub_gain = nh_private_.subscribe("tree_node_in", 10, &Cached::callbackGain, this);

        std::vector<std::string> uav_ids;
        param_loader.loadParam("uav_ids", uav_ids);
        
        for (const auto& uav_id : uav_ids) {
            std::string topic = "uav" + uav_id + "_state_in";
            ros::Subscriber sub = nh_private_.subscribe<mrs_msgs::UavState>(topic, 10, 
                              boost::bind(&MultiCached::callbackUavState, this, _1, uav_id));
            sub_uav_states.push_back(sub);
        }

        tsdf_map_ = voxblox_server_.getTsdfMapPtr();
        esdf_map_ = voxblox_server_.getEsdfMapPtr();
        evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());

        // Get vertical FoV and setup camera
        vertical_fov = evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
        evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
        GetTransformation();

        ROS_WARN("Range max parameter not specified");
        ROS_WARN("Defaulting to 10 m...");
        range = 10.0;

        reevaluate_timer = nh.createTimer(ros::Duration(3), &MultiCached::timerReevaluate, this);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Voxblox Map Server
    voxblox::EsdfServer voxblox_server_;

    // Shortcut to Maps
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;

    std::string frame_id;
    std::string body_frame_id;
    std::string camera_frame_id;

    // Transformations
    geometry_msgs::TransformStamped T_C_B_message;
    voxblox::Transformation T_C_B;

    ros::ServiceServer ss_best_node;
    ros::Subscriber sub_gain;
    std::vector<ros::Subscriber> sub_uav_states;
    ros::Timer reevaluate_timer;

    bool visualize_mean;
    bool visualize_sigma;
    bool visualize_pts;
    int resolution;
    double g_zero;
    double range;
    int num_yaw_samples;
    Evaluator evaluator;

    // Camera Parameters
    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;

    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;

    // Map to store UAV positions by ID
    std::unordered_map<std::string, Eigen::Vector3d> uav_positions_;

    void GetTransformation() {
        T_C_B_message.transform.translation.x = -0.173024;
        T_C_B_message.transform.translation.y = 0.011500;
        T_C_B_message.transform.translation.z = 0.059864;

        T_C_B_message.transform.rotation.x = 0.000000;
        T_C_B_message.transform.rotation.y = -0.087156;
        T_C_B_message.transform.rotation.z = 0.000000;
        T_C_B_message.transform.rotation.w = 0.996195;

        // Transform into matrix
        tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
        evaluator.setCameraExtrinsics(T_C_B);
    }

    void callbackUavState(const mrs_msgs::UavState::ConstPtr& msg, const std::string& uav_id) {
        uav_positions_[uav_id] = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    }

    void timerReevaluate(const ros::TimerEvent&) {
        ROS_INFO("Reevaluate Start");

        std::vector<RTreeValue> result_s;

        rtree.query(bgi::satisfies([this](RTreeValue const& v) {
            for (const auto& uav_pos : uav_positions_) {
                Point uav_point(uav_pos.second.x(), uav_pos.second.y(), uav_pos.second.z());
                Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
                if (bg::distance(uav_point, node_point) <= range) {
                    return true;
                }
            }
            return false;
        }), std::back_inserter(result_s));

        size_t result_before = result_s.size();

        for (const auto& node : result_s) {
            Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
            eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
            trajectory_point_gain.position_W = pos;
            trajectory_point_gain.setFromYaw(node.second.yaw);
            std::pair<double, double> result = evaluator.computeGainAEP(trajectory_point_gain);

            ROS_INFO("[MultiCached]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
            ROS_INFO("[MultiCached]: Old Point gain: %f", node.second.gain);
            ROS_INFO("[MultiCached]: New Point gain: %f", result.first);

            cache_nodes::Node updated_node = node.second;
            updated_node.gain = result.first;
            updated_node.yaw = result.second;

            rtree.remove(node);
            if (updated_node.gain > g_zero) {
                rtree.insert(std::make_pair(node.first, updated_node));
            }
        }

        size_t numNodes = rtree.size();
        ROS_INFO("[MultiCached]: List Size Before: %lu", result_before);
        ROS_INFO("[MultiCached]: Nodes in the RTree: %lu", numNodes);
        ROS_INFO("[MultiCached]: List Size After: %lu", result_s.size());

        ROS_INFO("Reevaluate Done");
    }

    void callbackGain(const cache_nodes::Node::ConstPtr& msg) {
        Point point(msg->position.x, msg->position.y, msg->position.z);
        rtree.insert(std::make_pair(point, *msg));
    }

    bool callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
        std::vector<RTreeValue> result_n;

        // Retrieve all nodes in the RTree
        rtree.query(bgi::satisfies([](RTreeValue const& v) { return true; }), std::back_inserter(result_n));

        for (const auto& value : result_n) {
            if (value.second.gain > req.threshold) {
                res.best_node.push_back(value.second.position);
                res.gain.push_back(value.second.gain);
            }
        }

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    MultiCached multicached(nh, nh_private);
    ros::spin();
    return 0;
}
