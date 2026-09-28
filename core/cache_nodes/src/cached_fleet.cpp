#include "cache_nodes/cached_fleet.h"

Cached_fleet::Cached_fleet(ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    ss_best_node = nh_private_.advertiseService("best_node_in", &Cached_fleet::callbackBestNode, this);

    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "multicached");

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

    param_loader.loadParam("uav_ids", uav_ids);

    sub_gain = nh_private_.subscribe("tree_node_in", 10, &Cached_fleet::callbackGain, this);

    for (const auto& uav_id : uav_ids) {
        std::string topic = uav_id + "_state_in";
        ros::Subscriber sub = nh_private_.subscribe<mrs_msgs::UavState>(topic, 10,
                                                                        boost::bind(&Cached_fleet::callbackUavState, this, _1, uav_id));
        sub_uav_states.push_back(sub);
    }

    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("multicached");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    // Get vertical FoV and setup camera
    vertical_fov = evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
    GetTransformation();

    reevaluate_timer = nh.createTimer(ros::Duration(3), &Cached_fleet::timerReevaluate, this);
}

void Cached_fleet::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.3).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[Cached_fleet]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    evaluator.setCameraExtrinsics(T_C_B);

    ROS_INFO("[KAEP]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
    ROS_INFO("[KAEP]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
}

void Cached_fleet::callbackUavState(const mrs_msgs::UavState::ConstPtr& msg, const std::string& uav_id) {
    uav_positions_[uav_id] = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void Cached_fleet::timerReevaluate(const ros::TimerEvent&) {
    ROS_INFO("Reevaluate Start");

    std::vector<RTreeValue> result_s;

    rtree.query(bgi::satisfies([this](RTreeValue const& v) {
                    for (const auto& uav_pos : uav_positions_) {
                        Point uav_point(uav_pos.second.x(), uav_pos.second.y(), uav_pos.second.z());
                        Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
                        if (bg::distance(uav_point, node_point) <= 2 * max_distance) {
                            return true;
                        }
                    }
                    return false;
                }),
                std::back_inserter(result_s));

    size_t numNodesBefore = rtree.size();

    for (const auto& node : result_s) {
        Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
        Eigen::Vector4d trajectory_point_gain;
        trajectory_point_gain.head<3>() = pos;
        trajectory_point_gain[3] = node.second.yaw;
        std::pair<double, double> result = evaluator.computeGainRaycasting(trajectory_point_gain, true);

        ROS_INFO("[Cached_fleet]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
        ROS_INFO("[Cached_fleet]: Old Point gain: %f", node.second.gain);
        ROS_INFO("[Cached_fleet]: New Point gain: %f", result.first);

        cache_nodes::Node updated_node = node.second;
        updated_node.gain = result.first;
        updated_node.yaw = result.second;

        rtree.remove(node);
        if (updated_node.gain > g_zero) {
            rtree.insert(std::make_pair(node.first, updated_node));
        }
    }

    size_t numNodesAfter = rtree.size();

    ROS_INFO("[Cached_fleet]: Nodes in the RTree Before: %lu", numNodesBefore);
    ROS_INFO("[Cached_fleet]: Search List Size: %lu", result_s.size());
    ROS_INFO("[Cached_fleet]: Nodes in the RTree After: %lu", numNodesAfter);

    ROS_INFO("Reevaluate Done");
}

void Cached_fleet::callbackGain(const cache_nodes::Node::ConstPtr& msg) {
    Point point(msg->position.x, msg->position.y, msg->position.z);
    rtree.insert(std::make_pair(point, *msg));
}

bool Cached_fleet::callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
    std::vector<RTreeValue> result_n;

    rtree.query(bgi::satisfies([this](RTreeValue const& v) {
                    bool above_g_zero = v.second.gain > g_zero;
                    return (above_g_zero);
                }),
                std::back_inserter(result_n));

    for (const auto& value : result_n) {
        if (value.second.gain > req.threshold) {
            res.best_node.push_back(value.second.position);
            res.gain.push_back(value.second.gain);
        }
    }

    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Cached_fleet multicached(nh, nh_private);
    ros::spin();
    return 0;
}
