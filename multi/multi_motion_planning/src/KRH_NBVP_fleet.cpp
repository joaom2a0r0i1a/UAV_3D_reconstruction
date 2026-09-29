#include "multi_motion_planning/KRH_NBVP_fleet.h"

KRH_NBVP_fleet::KRH_NBVP_fleet(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "KRH_NBVP_fleet");

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

    // UAV Parameters
    param_loader.loadParam("uav_parameters/max_vel", max_velocity);
    param_loader.loadParam("uav_parameters/max_accel", max_accel);
    param_loader.loadParam("uav_parameters/max_heading_vel", max_heading_velocity);
    param_loader.loadParam("uav_parameters/max_heading_accel", max_heading_accel);

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
    param_loader.loadParam("path/lambda", lambda);
    param_loader.loadParam("path/lambda2", lambda2);
    param_loader.loadParam("path/max_acceleration_iterations", max_accel_iterations);
    param_loader.loadParam("path/recovery_enabled", recovery_enabled_, true);
    param_loader.loadParam("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    param_loader.loadParam("path/recovery_min_tree", recovery_min_tree_, 10);
    param_loader.loadParam("path/recovery_timeout", recovery_timeout_, 12.0);

    // Timer
    param_loader.loadParam("timer_main/rate", timer_main_rate);

    // Initialize UAV as state IDLE
    state_ = STATE_IDLE;
    iteration_ = 0;
    reset_velocity = false;

    // Get vertical FoV and setup camera
    vertical_fov = segment_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    segment_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    segment_evaluator.setEsdfMap(esdf_map_);

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("KRH_NBVP_fleet");
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
    shopts.node_name          = "KRH_NBVP_fleet";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &KRH_NBVP_fleet::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &KRH_NBVP_fleet::callbackControlManagerDiag, this);
    // Paths of the other UAVs on their own queue, read right after each plan
    nh_evade_ = nh_private_;
    nh_evade_.setCallbackQueue(&evade_queue_);
    mrs_lib::SubscribeHandlerOptions shopts_evade = shopts;
    shopts_evade.nh = nh_evade_;
    sub_evade = mrs_lib::SubscribeHandler<multiagent_collision_check::Segment>(shopts_evade, "evasion_segment_in", &KRH_NBVP_fleet::callbackEvade, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &KRH_NBVP_fleet::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &KRH_NBVP_fleet::callbackStop, this);

    /* Service Clients */
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &KRH_NBVP_fleet::timerMain, this);

    is_initialized = true;
}

double KRH_NBVP_fleet::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool KRH_NBVP_fleet::isTrajectoryCollisionFree(kino_rrt_star::Trajectory* trajectory) const {
    kino_rrt_star::Node* node = trajectory->TrajectoryPoints.back().get();
    if (getMapDistance(node->point.head(3)) < uav_radius) {
        return false;
    }
    return true;
}

void KRH_NBVP_fleet::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[KRH_NBVP_fleet]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void KRH_NBVP_fleet::planStep() {
    best_score_ = 0.0;
    kino_rrt_star::Trajectory* best_trajectory = nullptr;
    next_best_trajectory = nullptr;

    double node_size = 0.2;

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
    std::unique_ptr<kino_rrt_star::Node> root_node_owned;
    if (best_branch.size() > 1) {
        root_node_owned = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, best_branch[1]->TrajectoryPoints.back()->velocity, best_branch[1]->TrajectoryPoints.back()->acceleration);
    } else {
        root_node_owned = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d(0, 0, 0));
    }
    std::unique_ptr<kino_rrt_star::Trajectory> Root = std::make_unique<kino_rrt_star::Trajectory>(std::move(root_node_owned));

    // Evaluates root node
    Root->cost = 0.0;
    Root->score = 0.0;

    KinoRRTStar.clearKDTree();
    kino_rrt_star::Trajectory* root_ptr = KinoRRTStar.addKDTreeTrajectory(std::move(Root));
    clearMarkers();
    visualize_node(root_ptr->TrajectoryPoints.back()->point, 2 * node_size, ns);

    bool isFirstIteration = true;
    int j = 1;
    collision_id_counter_ = 0;
    int expanded_num_nodes = 0;
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
                reset_velocity = true;
                ROS_WARN("[KRH_NBVP_fleet]: Backtracking (%s, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", j, executed_path_.size());
                best_branch.clear();
                return;
            }
            rotate();
            plan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue;
            }

            const Eigen::Vector4d& node_position = best_branch[i]->TrajectoryPoints.back()->point;

            kino_rrt_star::Trajectory* nearest_trajectory_best = nullptr;
            KinoRRTStar.findNearestKD(node_position.head(3), nearest_trajectory_best);

            std::unique_ptr<kino_rrt_star::Trajectory> raw_best_owned = best_branch[i]->clone();
            kino_rrt_star::Trajectory* raw_best = raw_best_owned.get();
            raw_best->parent = nearest_trajectory_best;

            visualize_node(raw_best->TrajectoryPoints.back()->point, node_size, ns);

            trajectory_point = raw_best->TrajectoryPoints.back()->point;

            double result_best = segment_evaluator.computeFixedGainRaycasting(trajectory_point);
            raw_best->gain = result_best;

            segment_evaluator.computeCostTwo(raw_best);
            segment_evaluator.computeScore(raw_best, lambda, lambda2);

            if (raw_best->score > best_score_) {
                best_score_ = raw_best->score;
                best_trajectory = raw_best;
            }

            ROS_INFO("[KRH_NBVP_fleet]: Best Score BB: %f", raw_best->score);

            KinoRRTStar.addKDTreeTrajectory(std::move(raw_best_owned));
            visualize_trajectory(raw_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > 0) {
            break;
        }

        best_branch.clear();

        Eigen::Vector4d rand_point_yaw;
        Eigen::Vector3d rand_point;
        KinoRRTStar.computeSamplingDimensionsYaw(bounded_radius, rand_point_yaw);
        rand_point = rand_point_yaw.head(3);
        rand_point += root_ptr->TrajectoryPoints.back()->point.head(3);

        kino_rrt_star::Trajectory* nearest_trajectory = nullptr;
        KinoRRTStar.findNearestKD(rand_point, nearest_trajectory);

        int accel_iteration = 0;
        int accel_tries = 0;
        while (accel_iteration < max_accel_iterations && accel_tries < 100 * max_accel_iterations) {
            accel_tries++;
            Eigen::Vector3d accel;
            KinoRRTStar.computeAccelerationSampling(max_accel, accel);

            std::unique_ptr<kino_rrt_star::Trajectory> new_trajectory = std::make_unique<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory(nearest_trajectory, max_velocity, reset_velocity, rand_point_yaw[3], accel, max_heading_velocity, max_heading_accel, step_size, new_trajectory);
            new_trajectory->TrajectoryPoints.back()->point[3] = rand_point_yaw[3];

            if (new_trajectory->TrajectoryPoints.back()->point[0] > max_x || new_trajectory->TrajectoryPoints.back()->point[0] < min_x || new_trajectory->TrajectoryPoints.back()->point[1] < min_y || new_trajectory->TrajectoryPoints.back()->point[1] > max_y || new_trajectory->TrajectoryPoints.back()->point[2] < min_z || new_trajectory->TrajectoryPoints.back()->point[2] > max_z) {
                break;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(new_trajectory.get())) {
                collision_id_counter_++;
                continue;
            }

            bool in_collision = false;
            for (int m = 1; m < (int)new_trajectory->TrajectoryPoints.size(); m++) {
                if (multiagent::isInCollision(new_trajectory->TrajectoryPoints[m - 1]->point, new_trajectory->TrajectoryPoints[m]->point, uav_radius, segments_)) {
                    collision_id_counter_++;
                    in_collision = true;
                    break;
                }
            }

            if (in_collision) {
                continue;
            }

            visualize_node(new_trajectory->TrajectoryPoints.back()->point, node_size, ns);
            ++accel_iteration;

            Eigen::Vector4d trajectory_point_gain = new_trajectory->TrajectoryPoints.back()->point;
            double result = segment_evaluator.computeFixedGainRaycasting(trajectory_point_gain);
            new_trajectory->gain = result;

            segment_evaluator.computeCostTwo(new_trajectory.get());
            segment_evaluator.computeScore(new_trajectory.get(), lambda, lambda2);

            if (new_trajectory->score > best_score_) {
                best_score_ = new_trajectory->score;
                best_trajectory = new_trajectory.get();
            }

            ROS_INFO("[KRH_NBVP_fleet]: Best Score: %f", new_trajectory->score);

            kino_rrt_star::Trajectory* added = KinoRRTStar.addKDTreeTrajectory(std::move(new_trajectory));
            visualize_trajectory(added, ns);
        }

        if (accel_iteration == 0) {
            continue;
        }

        expanded_num_nodes += accel_iteration;

        if (j > N_termination) {
            ROS_INFO("[KRH_NBVP_fleet]: KRH_NBVP Terminated");
            KinoRRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            changeState(STATE_STOPPED);
            break;
        }

        ++j;
    }

    ROS_INFO("[KRH_NBVP_fleet]: Final Best Score: %f", best_score_);
    ROS_INFO("[KRH_NBVP_fleet]: Node Iterations: %d", j);
    ROS_INFO("[KRH_NBVP_fleet]: Full Node Iterations: %d", expanded_num_nodes);

    if (best_trajectory) {
        reset_velocity = false;
        next_best_trajectory = best_trajectory;
        KinoRRTStar.backtrackTrajectory(best_trajectory, best_branch, next_best_trajectory);
        visualize_best_trajectory(best_trajectory, ns);
    }
}

double KRH_NBVP_fleet::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {
    return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                   vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

void KRH_NBVP_fleet::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[KRH_NBVP_fleet]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();

    ROS_INFO("[KRH_NBVP_fleet]: Rotating 360 degrees");

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

    ROS_INFO("[KRH_NBVP_fleet]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();
}

void KRH_NBVP_fleet::rotate() {
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

bool KRH_NBVP_fleet::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KRH_NBVP_fleet]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;
}

bool KRH_NBVP_fleet::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KRH_NBVP_fleet]: " << ss.str());

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

void KRH_NBVP_fleet::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KRH_NBVP_fleet]: getting ControlManager diagnostics");
    control_manager_diag = *msg;

    // If planner stops, set velocity to zero
    if (!control_manager_diag.tracker_status.have_goal && !reset_velocity) {
        reset_velocity = true;
    }
}

void KRH_NBVP_fleet::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KRH_NBVP_fleet]: getting UavState diagnostics");
    geometry_msgs::Pose uav_pose = msg->pose;
    geometry_msgs::Twist uav_velocity = msg->velocity;
    double yaw = mrs_lib::getYaw(uav_pose);
    pose = {uav_pose.position.x, uav_pose.position.y, uav_pose.position.z, yaw};
    velocity = {uav_velocity.linear.x, uav_velocity.linear.y, uav_velocity.linear.z};
}

void KRH_NBVP_fleet::callbackEvade(const multiagent_collision_check::Segment::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KRH_NBVP_fleet]: getting CollisionCheck diagnostics");

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
std::vector<std::vector<Eigen::Vector3d>*> KRH_NBVP_fleet::otherSegments() const {
    std::vector<std::vector<Eigen::Vector3d>*> others;
    for (size_t i = 0; i < agentsId_.size(); ++i) {
        if (agentsId_[i] != uav_id) {
            others.push_back(segments_[i]);
        }
    }
    return others;
}

bool KRH_NBVP_fleet::isPathClearOfOthers(const std::vector<Eigen::Vector3d>& path) const {
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

void KRH_NBVP_fleet::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[KRH_NBVP_fleet]: waiting for data: ControlManagerDiag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[KRH_NBVP_fleet]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[KRH_NBVP_fleet]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[KRH_NBVP_fleet]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KRH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KRH_NBVP_fleet]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KRH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KRH_NBVP_fleet]: waiting for command");
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
                const Eigen::Vector4d retreat_point = executed_path_.back();
                if (!isPathClearOfOthers({pose.head<3>(), retreat_point.head<3>()})) {
                    ROS_WARN("[KRH_NBVP_fleet]: Retreat blocked by another UAV, rotating instead");
                    rotate();
                    break;
                }

                iteration_ += 1;
                current_waypoint_.position.x = retreat_point[0];
                current_waypoint_.position.y = retreat_point[1];
                current_waypoint_.position.z = retreat_point[2];
                current_waypoint_.heading = retreat_point[3];

                multiagent_collision_check::Segment retreat_segment;
                retreat_segment.uav_id = uav_id;
                geometry_msgs::Point from;
                from.x = pose[0];
                from.y = pose[1];
                from.z = pose[2];
                retreat_segment.uav_path.push_back(from);
                retreat_segment.uav_path.push_back(current_waypoint_.position);
                pub_evade.publish(retreat_segment);

                mrs_msgs::ReferenceStamped retreat_reference;
                retreat_reference.header.frame_id = ns + "/" + frame_id;
                retreat_reference.header.stamp = ros::Time::now();
                retreat_reference.reference = current_waypoint_;
                pub_reference.publish(retreat_reference.reference);
                pub_initial_reference.publish(retreat_reference);

                ros::Duration(1).sleep();

                changeState(STATE_MOVING);
                break;
            }

            if (!next_best_trajectory) {
                ROS_WARN("[KRH_NBVP_fleet]: No trajectory chosen, planning again");
                break;
            }

            iteration_ += 1;

            current_waypoint_.position.x = next_best_trajectory->TrajectoryPoints.back()->point[0];
            current_waypoint_.position.y = next_best_trajectory->TrajectoryPoints.back()->point[1];
            current_waypoint_.position.z = next_best_trajectory->TrajectoryPoints.back()->point[2];
            current_waypoint_.heading = next_best_trajectory->TrajectoryPoints.back()->point[3];

            visualize_frustum(next_best_trajectory->TrajectoryPoints.back().get());
            visualize_unknown_voxels(next_best_trajectory->TrajectoryPoints.back().get());

            mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;

            srv_trajectory_reference.request.trajectory.header.frame_id = ns + "/" + frame_id;
            srv_trajectory_reference.request.trajectory.header.stamp = ros::Time::now();
            srv_trajectory_reference.request.trajectory.input_id = iteration_;
            srv_trajectory_reference.request.trajectory.fly_now = true;
            srv_trajectory_reference.request.trajectory.use_heading = true;

            srv_trajectory_reference.request.trajectory.dt = 0.1;

            mrs_msgs::Reference reference;

            // Recheck Against the Latest Paths of the Other UAVs
            std::vector<Eigen::Vector3d> planned_path;
            if (next_best_trajectory->parent) {
                for (const auto& point : next_best_trajectory->parent->TrajectoryPoints) {
                    planned_path.push_back(point->point.head<3>());
                }
            }
            for (const auto& point : next_best_trajectory->TrajectoryPoints) {
                planned_path.push_back(point->point.head<3>());
            }
            if (!isPathClearOfOthers(planned_path)) {
                ROS_WARN("[KRH_NBVP_fleet]: Trajectory crosses the new path of another UAV, planning again");
                best_branch.clear();
                break;
            }

            // Store Flown Path
            const kino_rrt_star::Trajectory* flown_parent = next_best_trajectory->parent;
            if (executed_path_.empty()) {
                executed_path_.push_back(flown_parent ? flown_parent->TrajectoryPoints.front()->point : pose);
            }
            if (flown_parent && flown_parent->parent) {
                executed_path_.push_back(flown_parent->TrajectoryPoints.back()->point);
            }
            executed_path_.push_back(next_best_trajectory->TrajectoryPoints.back()->point);

            if (next_best_trajectory->parent) {
                for (size_t i = 0; i < next_best_trajectory->parent->TrajectoryPoints.size(); i++) {
                    reference.position.x = next_best_trajectory->parent->TrajectoryPoints[i]->point[0];
                    reference.position.y = next_best_trajectory->parent->TrajectoryPoints[i]->point[1];
                    reference.position.z = next_best_trajectory->parent->TrajectoryPoints[i]->point[2];
                    reference.heading = next_best_trajectory->parent->TrajectoryPoints[i]->point[3];
                    srv_trajectory_reference.request.trajectory.points.push_back(reference);
                }
            }

            for (size_t j = 0; j < next_best_trajectory->TrajectoryPoints.size(); j++) {
                reference.position.x = next_best_trajectory->TrajectoryPoints[j]->point[0];
                reference.position.y = next_best_trajectory->TrajectoryPoints[j]->point[1];
                reference.position.z = next_best_trajectory->TrajectoryPoints[j]->point[2];
                reference.heading = next_best_trajectory->TrajectoryPoints[j]->point[3];
                pub_reference.publish(reference);
                srv_trajectory_reference.request.trajectory.points.push_back(reference);
            }

            multiagent_collision_check::Segment segment;
            segment.uav_id = uav_id;
            for (const auto& point : srv_trajectory_reference.request.trajectory.points) {
                segment.uav_path.push_back(point.position);
            }
            ROS_INFO_STREAM("Publishing to pub_evade with segment: uav_id=" << segment.uav_id
                                                                            << " with trajectory points=" << segment.uav_path.size());
            pub_evade.publish(segment);

            bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

            if (!success_trajectory) {
                ROS_ERROR("[KRH_NBVP_fleet]: service call for trajectory reference failed");
                //changeState(STATE_STOPPED);
                changeState(STATE_MOVING);
                return;
            } else {
                if (!srv_trajectory_reference.response.success) {
                    ROS_ERROR("[KRH_NBVP_fleet]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                    //changeState(STATE_STOPPED);
                    changeState(STATE_MOVING);
                    return;
                }
            }

            best_branch.clear();
            ros::Duration(1).sleep();

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KRH_NBVP_fleet]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_.heading - current_yaw), cos(current_waypoint_.heading - current_yaw)));
                ROS_INFO("[KRH_NBVP_fleet]: Distance to waypoint: %.2f", dist);
                if (dist <= 0.4 * step_size && yaw_difference <= 0.2 * M_PI) {
                    changeState(STATE_PLANNING);
                }
            } else {
                ROS_INFO("[KRH_NBVP_fleet]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[KRH_NBVP_fleet]: Total Iterations: %d", iteration_);
            ROS_INFO("[KRH_NBVP_fleet]: Shutting down.");
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
                ROS_INFO("[KRH_NBVP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KRH_NBVP_fleet]: waiting for command");
            }
            break;
        }
    }
}

void KRH_NBVP_fleet::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[KRH_NBVP_fleet]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[KRH_NBVP_fleet]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

// Rotates the colors by 120 degrees of hue per UAV, UAV1 keeps the original colors
void KRH_NBVP_fleet::colorForUav(std_msgs::ColorRGBA& color) const {
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

void KRH_NBVP_fleet::visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns) {
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

    n.scale.x = size;
    n.scale.y = size;
    n.scale.z = size;

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

void KRH_NBVP_fleet::visualize_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.stamp = ros::Time::now();
    trajectory_marker.header.frame_id = ns + "/" + frame_id;
    trajectory_marker.id = trajectory_id_counter_;
    trajectory_marker.ns = "trajectory";
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;

    trajectory_marker.color.r = 1.0;
    trajectory_marker.color.g = 0.3;
    trajectory_marker.color.b = 0.7;
    trajectory_marker.color.a = 1.0;
    colorForUav(trajectory_marker.color);

    trajectory_marker.scale.x = 0.1;
    trajectory_marker.scale.y = 0.1;
    trajectory_marker.scale.z = 0.1;

    for (const auto& node : trajectory->TrajectoryPoints) {
        geometry_msgs::Point p;
        p.x = node->point[0];
        p.y = node->point[1];
        p.z = node->point[2];
        trajectory_marker.points.push_back(p);
    }

    trajectory_marker.lifetime = ros::Duration(30.0);
    trajectory_marker.frame_locked = false;
    pub_markers.publish(trajectory_marker);

    trajectory_id_counter_++;
}

void KRH_NBVP_fleet::visualize_best_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
    kino_rrt_star::Trajectory* currentTrajectory = trajectory;

    while (currentTrajectory->parent) {
        visualization_msgs::Marker best_trajectory_marker;
        best_trajectory_marker.header.stamp = ros::Time::now();
        best_trajectory_marker.header.seq = best_trajectory_id_counter_;
        best_trajectory_marker.header.frame_id = ns + "/" + frame_id;
        best_trajectory_marker.id = best_trajectory_id_counter_;
        best_trajectory_marker.ns = "best_trajectory";
        best_trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
        best_trajectory_marker.action = visualization_msgs::Marker::ADD;

        best_trajectory_marker.color.r = 0.7;
        best_trajectory_marker.color.g = 0.7;
        best_trajectory_marker.color.b = 0.3;
        best_trajectory_marker.color.a = 1.0;
        colorForUav(best_trajectory_marker.color);

        best_trajectory_marker.scale.x = 0.2;
        best_trajectory_marker.scale.y = 0.2;
        best_trajectory_marker.scale.z = 0.2;

        for (const auto& node : currentTrajectory->TrajectoryPoints) {
            geometry_msgs::Point p;
            p.x = node->point[0];
            p.y = node->point[1];
            p.z = node->point[2];
            best_trajectory_marker.points.push_back(p);
        }

        best_trajectory_marker.lifetime = ros::Duration(100.0);
        best_trajectory_marker.frame_locked = false;
        pub_markers.publish(best_trajectory_marker);

        currentTrajectory = currentTrajectory->parent;
        best_trajectory_id_counter_++;
    }
}

void KRH_NBVP_fleet::visualize_frustum(kino_rrt_star::Node* position) {
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

void KRH_NBVP_fleet::visualize_unknown_voxels(kino_rrt_star::Node* position) {
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

void KRH_NBVP_fleet::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void KRH_NBVP_fleet::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void KRH_NBVP_fleet::clearMarkers() {
    // Clear nodes
    visualization_msgs::Marker clear_nodes;
    clear_nodes.header.stamp = ros::Time::now();
    clear_nodes.ns = "nodes";
    clear_nodes.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_nodes);

    // Clear trajectory
    visualization_msgs::Marker clear_trajectory;
    clear_trajectory.header.stamp = ros::Time::now();
    clear_trajectory.ns = "trajectory";
    clear_trajectory.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_trajectory);

    // Clear best trajectory
    visualization_msgs::Marker clear_best_trajectory;
    clear_best_trajectory.header.stamp = ros::Time::now();
    clear_best_trajectory.ns = "best_trajectory";
    clear_best_trajectory.action = visualization_msgs::Marker::DELETEALL;
    pub_markers.publish(clear_best_trajectory);

    // Reset marker ID counters
    node_id_counter_ = 0;
    trajectory_id_counter_ = 0;
    best_trajectory_id_counter_ = 0;
}
