#include "cache_nodes/cached.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, cache_nodes::Node> RTreeValue;

Cached::Cached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    
    ss_best_node = nh_private_.advertiseService("best_node_in", &Cached::callbackBestNode, this);
    
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "cached");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
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
    sub_uav_state = nh_private_.subscribe("uav_state_in", 10, &Cached::callbackUavState, this);

    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("cached");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    // Get vertical FoV and setup camera
    vertical_fov = evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
    GetTransformation();

    reevaluate_timer = nh.createTimer(ros::Duration(3), &Cached::timerReevaluate, this);
}

void Cached::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.3).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[Cached]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    evaluator.setCameraExtrinsics(T_C_B);

    ROS_INFO("[KinoAEPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
    ROS_INFO("[KinoAEPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
}

void Cached::callbackUavState(const mrs_msgs::UavState::ConstPtr& msg) {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
}

void Cached::timerReevaluate(const ros::TimerEvent&) {
    ROS_INFO("Reevaluate Start");

    std::vector<RTreeValue> result_s;
    rtree.query(bgi::satisfies([this](RTreeValue const& v) {
        Point current_point(x, y, z);
        Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
        return(bg::distance(current_point, node_point) <= 2*max_distance);
    }), std::back_inserter(result_s));

    size_t numNodesBefore = rtree.size();

    for (const auto& node : result_s) {
        Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
        trajectory_point_gain.position_W = pos;
        trajectory_point_gain.setFromYaw(node.second.yaw);
        std::pair<double, double> result = evaluator.computeGainOptimizedAEP(trajectory_point_gain);

        ROS_INFO("[Cached]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
        ROS_INFO("[Cached]: Old Point gain: %f", node.second.gain);
        ROS_INFO("[Cached]: New Point gain: %f", result.first);

        cache_nodes::Node updated_node = node.second;
        updated_node.gain = result.first;
        updated_node.yaw = result.second;

        rtree.remove(node);
        if (updated_node.gain > g_zero) {
            rtree.insert(std::make_pair(node.first, updated_node));
        }
    }

    size_t numNodesAfter = rtree.size();
    
    ROS_INFO("[Cached]: Nodes in the RTree Before: %lu", numNodesBefore);
    ROS_INFO("[Cached]: Search List Size: %lu", result_s.size());
    ROS_INFO("[Cached]: Nodes in the RTree After: %lu", numNodesAfter);

    ROS_INFO("Reevaluate Done");
}

void Cached::callbackGain(const cache_nodes::Node::ConstPtr& msg) {
    Point point(msg->position.x, msg->position.y, msg->position.z);
    rtree.insert(std::make_pair(point, *msg));
}

bool Cached::callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
    std::vector<RTreeValue> result_n;

    rtree.query(bgi::satisfies([this](RTreeValue const& v) {
        bool above_g_zero = v.second.gain > g_zero;
        return(above_g_zero);
    }), std::back_inserter(result_n));

    for (const auto& value : result_n) {
        if (value.second.gain > req.threshold) {
            res.best_node.push_back(value.second.position);
            res.gain.push_back(value.second.gain);
        }
    }

    return true;
}
