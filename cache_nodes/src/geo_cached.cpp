#include "cache_nodes/geo_cached.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, cache_nodes::Node> RTreeValue;

GEOCached::GEOCached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    
    ss_best_node = nh_private_.advertiseService("best_node_in", &GEOCached::callbackBestNode, this);
    
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "cached");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
    param_loader.loadParam("local_planning/g_zero", g_zero, 2.0);
    param_loader.loadParam("local_planning/yaw_samples", num_yaw_samples, 10);
    param_loader.loadParam("local_planning/uav_radius", uav_radius, 1.5);

    // Camera
    param_loader.loadParam("camera/h_fov", horizontal_fov);
    param_loader.loadParam("camera/width", resolution_x);
    param_loader.loadParam("camera/height", resolution_y);
    param_loader.loadParam("camera/min_distance", min_distance);
    param_loader.loadParam("camera/max_distance", max_distance);
    param_loader.loadParam("camera/frame_id", camera_frame_id);

    // Bounded Box
    param_loader.loadParam("bounded_box/min_x", min_x);
    param_loader.loadParam("bounded_box/max_x", max_x);
    param_loader.loadParam("bounded_box/min_y", min_y);
    param_loader.loadParam("bounded_box/max_y", max_y);
    param_loader.loadParam("bounded_box/min_z", min_z);
    param_loader.loadParam("bounded_box/max_z", max_z);
    param_loader.loadParam("bounded_box/spacing", spacing);

    sub_gain = nh_private_.subscribe("tree_node_in", 10, &GEOCached::callbackGain, this);
    sub_uav_state = nh_private_.subscribe("uav_state_in", 10, &GEOCached::callbackUavState, this);

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

    sample_timer = nh.createTimer(ros::Duration(1.0), &GEOCached::timerSampleCandidates, this);
    reevaluate_timer = nh.createTimer(ros::Duration(3), &GEOCached::timerReevaluate, this);
}

void GEOCached::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.3).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[GEOCached]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    evaluator.setCameraExtrinsics(T_C_B);

    ROS_INFO("[KinoAEPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
    ROS_INFO("[KinoAEPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
}

double GEOCached::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool GEOCached::isCollisionFree(const Eigen::Vector3d& pos) const {
    double dist = getMapDistance(pos);
    return dist >= uav_radius;
}

GridKey GEOCached::computeGridKey(const Eigen::Vector3d& pos) const {
    return GridKey{
        static_cast<int>(std::round(pos.x() / spacing)),
        static_cast<int>(std::round(pos.y() / spacing)),
        static_cast<int>(std::round(pos.z() / spacing))
    };
}

void GEOCached::sampleDynamicCandidates(double x_min, double x_max,
                                       double y_min, double y_max,
                                       double z_min, double z_max,
                                       double spacing) {
    ROS_INFO("[GEOCached] Building uniform candidate field...");
    int initial_size = rtree.size();
    // Define local cube around UAV, clipped to global bounds
    double lx_min = std::max(x_min, x - (2 * max_distance));
    double lx_max = std::min(x_max, x + (2 * max_distance));
    double ly_min = std::max(y_min, y - (2 * max_distance));
    double ly_max = std::min(y_max, y + (2 * max_distance));
    double lz_min = std::max(z_min, z - (2 * max_distance));
    double lz_max = std::min(z_max, z + (2 * max_distance));

    // Convert bounds to lattice indices
    int ix_min = std::ceil(lx_min / spacing);
    int ix_max = std::floor(lx_max / spacing);
    int iy_min = std::ceil(ly_min / spacing);
    int iy_max = std::floor(ly_max / spacing);
    int iz_min = std::ceil(lz_min / spacing);
    int iz_max = std::floor(lz_max / spacing);

    for (int ix = ix_min; ix <= ix_max; ++ix) {
        for (int iy = iy_min; iy <= iy_max; ++iy) {
            for (int iz = iz_min; iz <= iz_max; ++iz) {
                GridKey key{ix, iy, iz};

                if (candidate_map.count(key)) continue;

                Eigen::Vector3d pos(ix * spacing,
                                    iy * spacing,
                                    iz * spacing);

                if (!isCollisionFree(pos)) continue;
                
                cache_nodes::Node node;
                node.position.x = pos.x();
                node.position.y = pos.y();
                node.position.z = pos.z();
                node.yaw = 0.0;
                node.gain = 0.0;
                node.active = true;

                candidate_map[key] = node;
                Point p(pos.x(), pos.y(), pos.z());
                rtree.insert(std::make_pair(p, node));
            }
        }
    }
    int final_size = rtree.size();
    int num_inserted = final_size - initial_size;
    ROS_INFO("[GEOCached] Inserted %u uniform candidates", num_inserted);
}

void GEOCached::timerSampleCandidates(const ros::TimerEvent&) {
    sampleDynamicCandidates(min_x, max_x, min_y, max_y, min_z, max_z, spacing);
}

void GEOCached::timerReevaluate(const ros::TimerEvent&) {
    ROS_INFO("Reevaluate Start");

    std::vector<RTreeValue> result_s;
    rtree.query(bgi::satisfies([this](RTreeValue const& v) {
        Point current_point(x, y, z);
        Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
        return(bg::distance(current_point, node_point) <= 2*max_distance);
    }), std::back_inserter(result_s));

    size_t numNodesBefore = rtree.size();
    std::vector<RTreeValue> updated_nodes;

    for (const auto& node : result_s) {
        Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
        
        bool update = true;
        if (!node.second.active) {
            if (!isCollisionFree(pos)) {
                update = false;
            }
        }

        if (!update) continue;
        
        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
        trajectory_point_gain.position_W = pos;
        trajectory_point_gain.setFromYaw(node.second.yaw);

        auto [path_gain, max_gain, pose_angle] = evaluator.computeGainOptimizedGEO(trajectory_point_gain);
        double new_gain = 0.0;
        if (max_gain > 0.0) {
            new_gain = path_gain / max_gain * 100.0;
        }

        ROS_INFO("[GEOCached]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
        ROS_INFO("[GEOCached]: Old Point gain: %f", node.second.gain);
        ROS_INFO("[GEOCached]: New Point gain: %f", new_gain);

        cache_nodes::Node updated_node = node.second;
        updated_node.active = true;
        updated_node.gain = new_gain;
        updated_node.yaw = pose_angle;

        rtree.remove(node);
        GridKey key = computeGridKey(pos);
        if (updated_node.gain > g_zero) {
            candidate_map[key] = updated_node;
            rtree.insert(std::make_pair(node.first, updated_node));
        } /*else {
            candidate_map.erase(key);
        }*/
    }

    size_t numNodesAfter = rtree.size();
    
    ROS_INFO("[GEOCached]: Nodes in the RTree Before: %lu", numNodesBefore);
    ROS_INFO("[GEOCached]: Search List Size: %lu", result_s.size());
    ROS_INFO("[GEOCached]: Nodes in the RTree After: %lu", numNodesAfter);

    ROS_INFO("Reevaluate Done");
}

void GEOCached::callbackUavState(const mrs_msgs::UavState::ConstPtr& msg) {
    x = msg->pose.position.x;
    y = msg->pose.position.y;
    z = msg->pose.position.z;
}

void GEOCached::callbackGain(const cache_nodes::Node::ConstPtr& msg) {
    Point point(msg->position.x, msg->position.y, msg->position.z);
    rtree.insert(std::make_pair(point, *msg));
}

bool GEOCached::callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
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
