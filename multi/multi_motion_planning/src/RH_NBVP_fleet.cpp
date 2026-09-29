#include "multi_motion_planning/RH_NBVP_fleet.h"

RH_NBVP_fleet::RH_NBVP_fleet(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "RH_NBVP_fleet");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);
    param_loader.loadParam("uav_id", uav_id);

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
    param_loader.loadParam("camera/frame_id", camera_frame_id);

    // Bounded Box for Sampling
    if (!loadEnvironmentRegion(nh_private_, "planning_box", min_x, max_x, min_y, max_y, min_z, max_z)) {
        ros::shutdown();
        return;
    }

    // RRT Tree
    param_loader.loadParam("rrt/N_max", N_max);
    param_loader.loadParam("rrt/N_termination", N_termination);
    param_loader.loadParam("rrt/N_yaw_samples", num_yaw_samples);
    param_loader.loadParam("rrt/radius", radius);
    param_loader.loadParam("rrt/step_size", step_size);
    param_loader.loadParam("rrt/tolerance", tolerance);

    // Camera
    param_loader.loadParam("camera/h_fov", horizontal_fov);
    param_loader.loadParam("camera/width", resolution_x);
    param_loader.loadParam("camera/height", resolution_y);
    param_loader.loadParam("camera/min_distance", min_distance);
    param_loader.loadParam("camera/max_distance", max_distance);

    // Planner
    param_loader.loadParam("path/uav_radius", uav_radius);
    param_loader.loadParam("path/collision_check_resolution", collision_check_resolution_, 0.1);
    param_loader.loadParam("path/recovery_enabled", recovery_enabled_, true);
    param_loader.loadParam("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    param_loader.loadParam("path/recovery_min_tree", recovery_min_tree_, 10);
    param_loader.loadParam("path/recovery_timeout", recovery_timeout_, 12.0);
    param_loader.loadParam("path/lambda", lambda);

    // Timer
    param_loader.loadParam("timer_main/rate", timer_main_rate);

    // Initialize UAV as state IDLE
    state_ = STATE_IDLE;
    iteration_ = 0;

    // Get vertical FoV and setup camera
    vertical_fov = segment_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    segment_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    segment_evaluator.setEsdfMap(esdf_map_);

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("RH_NBVP_fleet");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    set_variables = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();

    // Get Sampling Radius
    bounded_radius = sqrt(pow(min_x - max_x, 2.0) + pow(min_y - max_y, 2.0) + pow(min_z - max_z, 2.0));

    /* Publishers */
    pub_markers = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);
    pub_reference = nh_private_.advertise<mrs_msgs::Reference>("reference_out", 1);
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_initial_reference = nh_private_.advertise<mrs_msgs::ReferenceStamped>("initial_reference_out", 5);
    pub_evade = nh_private_.advertise<multiagent_collision_check::Segment>("evasion_segment_out", 100);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "RH_NBVP_fleet";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &RH_NBVP_fleet::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &RH_NBVP_fleet::callbackControlManagerDiag, this);
    // Paths of the other UAVs on their own queue, read right after each plan
    nh_evade_ = nh_private_;
    nh_evade_.setCallbackQueue(&evade_queue_);
    mrs_lib::SubscribeHandlerOptions shopts_evade = shopts;
    shopts_evade.nh = nh_evade_;
    sub_evade = mrs_lib::SubscribeHandler<multiagent_collision_check::Segment>(shopts_evade, "evasion_segment_in", &RH_NBVP_fleet::callbackEvade, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &RH_NBVP_fleet::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &RH_NBVP_fleet::callbackStop, this);

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &RH_NBVP_fleet::timerMain, this);

    is_initialized = true;
}

double RH_NBVP_fleet::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool RH_NBVP_fleet::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const {
    for (rrt_star::Node* node : path) {
        if (getMapDistance(node->point.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

bool RH_NBVP_fleet::isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
    const Eigen::Vector3d d = to - from;
    const int n = std::max(1, static_cast<int>(std::ceil(d.norm() / collision_check_resolution_)));
    for (int i = 0; i <= n; ++i) {
        if (getMapDistance(from + d * (static_cast<double>(i) / n)) < uav_radius) {
            return false;
        }
    }
    return true;
}

void RH_NBVP_fleet::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[RH_NBVP_fleet]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void RH_NBVP_fleet::planStep() {
    best_score_ = 0;
    rrt_star::Node* best_node = nullptr;
    next_best_node = nullptr;

    // Multi-UAV remove previous planned agent path
    int k;
    for (k = 0; k < agentsId_.size(); k++) {
        if (agentsId_[k] == uav_id) {
            break;
        }
    }
    if (k < agentsId_.size()) {
        segments_[k]->clear();
        segments_[k]->push_back(Eigen::Vector3d(pose[0], pose[1], pose[2]));
    }

    // Finds root node
    std::unique_ptr<rrt_star::Node> root;
    if (prev_best_branch.size() > 1) {
        root = std::make_unique<rrt_star::Node>(prev_best_branch[1]);
    } else {
        root = std::make_unique<rrt_star::Node>(pose);
    }

    // Evaluates root node
    root->cost = 0;
    root->score = root->gain;

    RRTStar.clearKDTree();
    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));

    if (root_ptr->score > best_score_) {
        best_score_ = root_ptr->score;
        best_node = root_ptr;
    }

    clearMarkers();

    bool isFirstIteration = true;
    int j = 1;
    collision_id_counter_ = 0;
    ros::WallTime plan_start_ = ros::WallTime::now();
    while (j < N_max || best_score_ == 0.0) {
        // Backtrack When Stuck
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in  = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) {
                executed_path_.pop_back();
            }
            if (!executed_path_.empty()) {
                retreating_ = true;
                ROS_WARN("[RH_NBVP_fleet]: Backtracking (%s, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", j, executed_path_.size());
                best_branch.clear();
                prev_best_branch.clear();
                return;
            }
            rotate();
            plan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        for (size_t i = 1; i < prev_best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue;
            }

            const Eigen::Vector4d& node_position = prev_best_branch[i];

            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);

            std::unique_ptr<rrt_star::Node> new_node_best = std::make_unique<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;
            visualize_node(new_node_best->point, ns);

            trajectory_point = new_node_best->point;

            double result_best = segment_evaluator.computeFixedGainRaycasting(trajectory_point);
            new_node_best->gain = result_best;

            segment_evaluator.computeCost(new_node_best.get());
            segment_evaluator.computeScore(new_node_best.get(), lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best.get();
            }

            ROS_INFO("[RH_NBVP_fleet]: Best Score BB: %f", new_node_best->score);

            rrt_star::Node* added_node_best = RRTStar.addKDTreeNode(std::move(new_node_best));
            visualize_edge(added_node_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > 0) {
            break;
        }

        prev_best_branch.clear();

        Eigen::Vector4d rand_point_yaw;
        Eigen::Vector3d rand_point;
        RRTStar.computeSamplingDimensionsYaw(bounded_radius, rand_point_yaw);
        rand_point = rand_point_yaw.head(3);
        rand_point += root_ptr->point.head(3);

        rrt_star::Node* nearest_node = nullptr;
        RRTStar.findNearestKD(rand_point, nearest_node);

        std::unique_ptr<rrt_star::Node> new_node;
        RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

        if (new_node->point[0] > max_x || new_node->point[0] < min_x || new_node->point[1] < min_y || new_node->point[1] > max_y || new_node->point[2] < min_z || new_node->point[2] > max_z) {
            continue;
        }

        // Collision Check
        std::vector<rrt_star::Node*> trajectory_segment;
        //trajectory_segment.push_back(new_node->parent);
        trajectory_segment.push_back(new_node.get());

        bool success_collision = false;
        success_collision = isPathCollisionFree(trajectory_segment);

        if (!success_collision || !isEdgeCollisionFree(new_node->parent->point.head<3>(), new_node->point.head<3>()) ||
            multiagent::isInCollision(new_node->parent->point, new_node->point, uav_radius, segments_)) {
            //clear_node();
            /*if (multiagent::isInCollision(new_node->parent->point, new_node->point, uav_radius, segments_)) {
                ROS_INFO("[RH_NBVP_fleet]: In Drone Collision");
            }*/
            trajectory_segment.clear();
            collision_id_counter_++;
            continue;
        }

        trajectory_segment.clear();
        visualize_node(new_node->point, ns);

        new_node->point[3] = rand_point_yaw[3];
        Eigen::Vector4d trajectory_point_gain = new_node->point;
        //ROS_INFO("[RH_NBVP_fleet]: Best gain RayCast: %f", new_node->gain);
        double result = segment_evaluator.computeFixedGainRaycasting(trajectory_point_gain);
        new_node->gain = result;

        segment_evaluator.computeCost(new_node.get());
        segment_evaluator.computeScore(new_node.get(), lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node.get();
        }

        ROS_INFO("[RH_NBVP_fleet]: Best Score: %f", new_node->score);

        rrt_star::Node* added_node = RRTStar.addKDTreeNode(std::move(new_node));
        visualize_edge(added_node, ns);

        if (j > N_termination) {
            ROS_INFO("[RH_NBVP_fleet]: RH_NBVP Terminated");
            RRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            changeState(STATE_STOPPED);
            break;
        }

        ++j;
    }
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathNode(best_node, best_branch, next_best_node);
        visualize_path(best_node, ns);
        prev_best_branch = best_branch;
    }
}

double RH_NBVP_fleet::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {
    return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                   vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

void RH_NBVP_fleet::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[RH_NBVP_fleet]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();

    ROS_INFO("[RH_NBVP_fleet]: Rotating 360 degrees");

    for (double i = 0.0; i <= 2.0; i = i + 0.4) {
        initial_reference.reference.position.x = pose[0];
        initial_reference.reference.position.y = pose[1];
        initial_reference.reference.position.z = pose[2] + 3;
        initial_reference.reference.heading = pose[3] + M_PI * i;
        pub_initial_reference.publish(initial_reference);
        // Max yaw rate is 0.5 rad/s so we wait 0.4*M_PI seconds between points
        ros::Duration(0.4 * M_PI).sleep();
    }

    ros::Duration(0.5).sleep();

    ROS_INFO("[RH_NBVP_fleet]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();
}

void RH_NBVP_fleet::rotate() {
    mrs_msgs::ReferenceStamped initial_reference;
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    // Rotate 360 degrees
    for (double i = 0.0; i <= 2.0; i = i + 0.4) {
        initial_reference.reference.position.x = pose[0];
        initial_reference.reference.position.y = pose[1];
        initial_reference.reference.position.z = pose[2];
        initial_reference.reference.heading = pose[3] + M_PI * i;
        pub_initial_reference.publish(initial_reference);
        // Max yaw rate is 0.5 rad/s so we wait 0.4*M_PI seconds between points
        ros::Duration(0.4 * M_PI).sleep();
    }
}

bool RH_NBVP_fleet::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[RH_NBVP_fleet]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;
}

bool RH_NBVP_fleet::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[RH_NBVP_fleet]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }
    changeState(STATE_STOPPED);

    std::stringstream ss;
    ss << "Stopping by request";

    res.success = true;
    res.message = ss.str();
    return true;
}

void RH_NBVP_fleet::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[RH_NBVP_fleet]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void RH_NBVP_fleet::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[RH_NBVP_fleet]: getting UavState diagnostics");
    geometry_msgs::Pose uav_state = msg->pose;
    double yaw = mrs_lib::getYaw(uav_state);
    pose = {uav_state.position.x, uav_state.position.y, uav_state.position.z, yaw};
}

void RH_NBVP_fleet::callbackEvade(const multiagent_collision_check::Segment::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[RH_NBVP_fleet]: getting CollisionCheck diagnostics");

    int i;
    for (i = 0; i < agentsId_.size(); i++) {
        if (agentsId_[i] == msg->uav_id) {
            break;
        }
    }

    // If no match was found, add the uav_id to the list of UAV IDs
    if (i == agentsId_.size()) {
        agentsId_.push_back(msg->uav_id);
        segments_.push_back(new std::vector<Eigen::Vector3d>);
    }

    // Update the segment list with the poses from msg
    segments_[i]->clear();
    for (std::vector<geometry_msgs::Point>::const_iterator it = msg->uav_path.begin(); it != msg->uav_path.end(); ++it) {
        segments_[i]->push_back(Eigen::Vector3d(it->x, it->y, it->z));
    }
}

// Paths of the other UAVs only
std::vector<std::vector<Eigen::Vector3d>*> RH_NBVP_fleet::otherSegments() const {
    std::vector<std::vector<Eigen::Vector3d>*> others;
    for (size_t i = 0; i < agentsId_.size(); ++i) {
        if (agentsId_[i] != uav_id) {
            others.push_back(segments_[i]);
        }
    }
    return others;
}

bool RH_NBVP_fleet::isPathClearOfOthers(const std::vector<Eigen::Vector3d>& path) const {
    const std::vector<std::vector<Eigen::Vector3d>*> others = otherSegments();
    for (size_t i = 1; i < path.size(); ++i) {
        const Eigen::Vector4d a(path[i - 1].x(), path[i - 1].y(), path[i - 1].z(), 0.0);
        const Eigen::Vector4d b(path[i].x(), path[i].y(), path[i].z(), 0.0);
        if (multiagent::isInCollision(a, b, uav_radius, others)) {
            return false;
        }
    }
    return true;
}

void RH_NBVP_fleet::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[RH_NBVP_fleet]: waiting for data: ControlManagerDiag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[RH_NBVP_fleet]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[RH_NBVP_fleet]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[RH_NBVP_fleet]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[RH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[RH_NBVP_fleet]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[RH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[RH_NBVP_fleet]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_INITIALIZE: {
            mrs_msgs::ReferenceStamped initial_reference;
            initialize(initial_reference);
            changeState(STATE_WAITING_INITIALIZE);
            break;
        }
        case STATE_PLANNING: {
            retreating_ = false;
            planStep();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            // Latest Paths of the Other UAVs
            evade_queue_.callAvailable(ros::WallDuration(0.0));

            // Retreat to Previous Node, if no other UAV is in the way
            if (retreating_ && !executed_path_.empty()) {
                if (!isPathClearOfOthers({pose.head<3>(), executed_path_.back().head<3>()})) {
                    ROS_WARN("[RH_NBVP_fleet]: Retreat blocked by another UAV, rotating instead");
                    rotate();
                    break;
                }
                retreat_node_ = std::make_unique<rrt_star::Node>(executed_path_.back());
                retreat_node_->parent = nullptr;
                next_best_node = retreat_node_.get();
            }

            if (!next_best_node) {
                ROS_WARN("[RH_NBVP_fleet]: No node chosen, planning again");
                break;
            }

            // Recheck Against the Latest Paths of the Other UAVs
            if (!retreating_) {
                const Eigen::Vector3d step_from = next_best_node->parent ? next_best_node->parent->point.head<3>() : pose.head<3>();
                if (!isPathClearOfOthers({step_from, next_best_node->point.head<3>()})) {
                    ROS_WARN("[RH_NBVP_fleet]: Path crosses the new path of another UAV, planning again");
                    best_branch.clear();
                    prev_best_branch.clear();
                    break;
                }
            }

            // Store Flown Path
            if (!retreating_) {
                if (executed_path_.empty() && next_best_node->parent) {
                    executed_path_.push_back(next_best_node->parent->point);
                }
                executed_path_.push_back(next_best_node->point);
            }

            iteration_ += 1;

            current_waypoint_.position.x = next_best_node->point[0];
            current_waypoint_.position.y = next_best_node->point[1];
            current_waypoint_.position.z = next_best_node->point[2];
            current_waypoint_.heading = next_best_node->point[3];

            visualize_frustum(next_best_node);
            visualize_unknown_voxels(next_best_node);

            mrs_msgs::Reference reference;

            mrs_msgs::ReferenceStamped initial_reference;
            initial_reference.header.frame_id = ns + "/" + frame_id;
            initial_reference.header.stamp = ros::Time::now();

            initial_reference.reference.position.x = next_best_node->point[0];
            initial_reference.reference.position.y = next_best_node->point[1];
            initial_reference.reference.position.z = next_best_node->point[2];
            initial_reference.reference.heading = next_best_node->point[3];
            pub_reference.publish(initial_reference.reference);
            pub_initial_reference.publish(initial_reference);

            multiagent_collision_check::Segment segment;
            segment.uav_id = uav_id;

            if (retreating_) {
                geometry_msgs::Point from;
                from.x = pose[0];
                from.y = pose[1];
                from.z = pose[2];
                segment.uav_path.push_back(from);
            } else if (next_best_node && next_best_node->parent) {
                mrs_msgs::Reference prev_ref;
                prev_ref.position.x = next_best_node->parent->point[0];
                prev_ref.position.y = next_best_node->parent->point[1];
                prev_ref.position.z = next_best_node->parent->point[2];
                prev_ref.heading = next_best_node->parent->point[3];
                segment.uav_path.push_back(prev_ref.position);
            }

            segment.uav_path.push_back(initial_reference.reference.position);

            ROS_INFO_STREAM("Publishing to pub_evade with segment: uav_id=" << segment.uav_id
                                                                            << " with path points=" << segment.uav_path.size());
            pub_evade.publish(segment);

            best_branch.clear();
            ros::Duration(1).sleep();

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[RH_NBVP_fleet]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_.heading - current_yaw), cos(current_waypoint_.heading - current_yaw)));
                ROS_INFO("[RH_NBVP_fleet]: Distance to waypoint: %.2f", dist);
            } else {
                ROS_INFO("[RH_NBVP_fleet]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[RH_NBVP_fleet]: Total Iterations: %d", iteration_);
            ROS_INFO("[RH_NBVP_fleet]: Shutting down.");
            // Multi-UAV remove final pose so drones don't collide when algorithm is finished
            int k;
            for (k = 0; k < agentsId_.size(); k++) {
                if (agentsId_[k] == uav_id) {
                    break;
                }
            }
            if (k < agentsId_.size()) {
                segments_[k]->clear();
                segments_[k]->push_back(Eigen::Vector3d(pose[0], pose[1], pose[2]));
            }
            ros::shutdown();
            return;
        }
        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[RH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[RH_NBVP_fleet]: waiting for command");
            }
            break;
        }
    }
}

void RH_NBVP_fleet::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[RH_NBVP_fleet]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[RH_NBVP_fleet]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

// Rotates the colors by 120 degrees of hue per UAV, UAV1 keeps the original colors
void RH_NBVP_fleet::colorForUav(std_msgs::ColorRGBA& color) const {
    const float r = color.r, g = color.g, b = color.b;
    const int palette = (uav_id - 1) % 3;
    if (palette == 1) {
        color.r = b;
        color.g = r;
        color.b = g;
    } else if (palette == 2) {
        color.r = g;
        color.g = b;
        color.b = r;
    }
}

void RH_NBVP_fleet::visualize_node(const Eigen::Vector4d& pos, const std::string& ns) {
    visualization_msgs::Marker n;
    n.header.stamp = ros::Time::now();
    n.header.seq = node_id_counter_;
    n.header.frame_id = ns + "/" + frame_id;
    n.id = node_id_counter_;
    n.ns = "nodes";
    n.type = visualization_msgs::Marker::SPHERE;
    n.action = visualization_msgs::Marker::ADD;
    n.pose.position.x = pos[0];
    n.pose.position.y = pos[1];
    n.pose.position.z = pos[2];

    n.pose.orientation.x = 1;
    n.pose.orientation.y = 0;
    n.pose.orientation.z = 0;
    n.pose.orientation.w = 0;

    n.scale.x = 0.2;
    n.scale.y = 0.2;
    n.scale.z = 0.2;

    n.color.r = 0.4;
    n.color.g = 0.7;
    n.color.b = 0.2;
    n.color.a = 1;
    colorForUav(n.color);

    node_id_counter_++;

    n.lifetime = ros::Duration(30.0);
    n.frame_locked = false;
    pub_markers.publish(n);
}

void RH_NBVP_fleet::visualize_edge(rrt_star::Node* node, const std::string& ns) {
    visualization_msgs::Marker e;
    e.header.stamp = ros::Time::now();
    e.header.seq = edge_id_counter_;
    e.header.frame_id = ns + "/" + frame_id;
    e.id = edge_id_counter_;
    e.ns = "tree_branches";
    e.type = visualization_msgs::Marker::ARROW;
    e.action = visualization_msgs::Marker::ADD;
    e.pose.position.x = node->parent->point[0];
    e.pose.position.y = node->parent->point[1];
    e.pose.position.z = node->parent->point[2];

    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->point[0] - node->parent->point[0],
                        node->point[1] - node->parent->point[1],
                        node->point[2] - node->parent->point[2]);
    q.setFromTwoVectors(init, dir);
    q.normalize();

    e.pose.orientation.x = q.x();
    e.pose.orientation.y = q.y();
    e.pose.orientation.z = q.z();
    e.pose.orientation.w = q.w();

    e.scale.x = dir.norm();
    e.scale.y = 0.05;
    e.scale.z = 0.05;

    e.color.r = 1.0;
    e.color.g = 0.3;
    e.color.b = 0.7;
    e.color.a = 1.0;
    colorForUav(e.color);

    edge_id_counter_++;

    e.lifetime = ros::Duration(30.0);
    e.frame_locked = false;
    pub_markers.publish(e);
}

void RH_NBVP_fleet::visualize_path(rrt_star::Node* node, const std::string& ns) {
    rrt_star::Node* current = node;

    while (current->parent) {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = path_id_counter_;
        p.header.frame_id = ns + "/" + frame_id;
        p.id = path_id_counter_;
        p.ns = "path";
        p.type = visualization_msgs::Marker::ARROW;
        p.action = visualization_msgs::Marker::ADD;
        p.pose.position.x = current->parent->point[0];
        p.pose.position.y = current->parent->point[1];
        p.pose.position.z = current->parent->point[2];

        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        Eigen::Vector3d dir(current->point[0] - current->parent->point[0],
                            current->point[1] - current->parent->point[1],
                            current->point[2] - current->parent->point[2]);
        q.setFromTwoVectors(init, dir);
        q.normalize();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();
        p.pose.orientation.w = q.w();

        p.scale.x = dir.norm();
        p.scale.y = 0.07;
        p.scale.z = 0.07;

        p.color.r = 0.7;
        p.color.g = 0.7;
        p.color.b = 0.3;
        p.color.a = 1.0;
        colorForUav(p.color);

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter_++;
    }
}

void RH_NBVP_fleet::visualize_frustum(rrt_star::Node* position) {
    Eigen::Vector4d trajectory_point_visualize = position->point;

    visualization_msgs::Marker frustum;
    frustum.header.frame_id = ns + "/" + frame_id;
    frustum.header.stamp = ros::Time::now();
    frustum.ns = "camera_frustum";
    frustum.id = 0;
    frustum.type = visualization_msgs::Marker::LINE_LIST;
    frustum.action = visualization_msgs::Marker::ADD;

    // Line width
    frustum.scale.x = 0.02;

    frustum.color.a = 1.0;
    frustum.color.r = 1.0;
    frustum.color.g = 0.0;
    frustum.color.b = 0.0;

    std::vector<geometry_msgs::Point> points;
    segment_evaluator.visualize_frustum(trajectory_point_visualize, points);

    frustum.points = points;
    frustum.lifetime = ros::Duration(10.0);
    pub_frustum.publish(frustum);
}

void RH_NBVP_fleet::visualize_unknown_voxels(rrt_star::Node* position) {
    Eigen::Vector4d trajectory_point_visualize = position->point;

    voxblox::Pointcloud voxel_points;
    segment_evaluator.visualizeGain(trajectory_point_visualize, voxel_points);

    visualization_msgs::MarkerArray voxels_marker;
    for (size_t i = 0; i < voxel_points.size(); ++i) {
        visualization_msgs::Marker unknown_voxel;
        unknown_voxel.header.frame_id = ns + "/" + frame_id;
        unknown_voxel.header.stamp = ros::Time::now();
        unknown_voxel.ns = "unknown_voxels";
        unknown_voxel.id = i;
        unknown_voxel.type = visualization_msgs::Marker::CUBE;
        unknown_voxel.action = visualization_msgs::Marker::ADD;

        // Voxel size
        unknown_voxel.scale.x = 0.2;
        unknown_voxel.scale.y = 0.2;
        unknown_voxel.scale.z = 0.2;

        unknown_voxel.color.a = 0.5;
        unknown_voxel.color.r = 0.0;
        unknown_voxel.color.g = 1.0;
        unknown_voxel.color.b = 0.0;

        unknown_voxel.pose.position.x = voxel_points[i].x();
        unknown_voxel.pose.position.y = voxel_points[i].y();
        unknown_voxel.pose.position.z = voxel_points[i].z();
        unknown_voxel.lifetime = ros::Duration(3.0);
        voxels_marker.markers.push_back(unknown_voxel);
    }
    pub_voxels.publish(voxels_marker);
}

void RH_NBVP_fleet::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void RH_NBVP_fleet::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void RH_NBVP_fleet::clearMarkers() {
    // Clear nodes
    visualization_msgs::Marker clear_nodes;
    clear_nodes.header.stamp = ros::Time::now();
    clear_nodes.ns = "nodes";
    clear_nodes.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_nodes);

    // Clear edges
    visualization_msgs::Marker clear_edges;
    clear_edges.header.stamp = ros::Time::now();
    clear_edges.ns = "tree_branches";
    clear_edges.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_edges);

    // Clear path
    visualization_msgs::Marker clear_path;
    clear_path.header.stamp = ros::Time::now();
    clear_path.ns = "path";
    clear_path.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_path);

    // Reset marker ID counters
    node_id_counter_ = 0;
    edge_id_counter_ = 0;
    path_id_counter_ = 0;
}
