#include "motion_planning_real_world/KRH_NBVP/KRH_NBVP_rw.h"

KRH_NBVP_rw::KRH_NBVP_rw(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "KRH_NBVP_rw");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
    param_loader.loadParam("camera/frame_id", camera_frame_id);

    // Bounded Box
    param_loader.loadParam("bounded_box/min_x", min_x);
    param_loader.loadParam("bounded_box/max_x", max_x);
    param_loader.loadParam("bounded_box/min_y", min_y);
    param_loader.loadParam("bounded_box/max_y", max_y);
    param_loader.loadParam("bounded_box/min_z", min_z);
    param_loader.loadParam("bounded_box/max_z", max_z);

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
    nh_private_.param("pose_sanity/max_distance", pose_max_distance_, 500.0);
    nh_private_.param("pose_sanity/max_speed", pose_max_speed_, 20.0);
    nh_private_.param("path/recovery_enabled", recovery_enabled_, true);
    nh_private_.param("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    nh_private_.param("path/recovery_min_tree", recovery_min_tree_, 10);
    nh_private_.param("path/recovery_timeout", recovery_timeout_, 12.0);
    nh_private_.param("rotation/step_deg", rotation_step_deg_, 45.0);
    nh_private_.param("rotation/settle", rotation_settle_, 1.0);
    nh_private_.param("exploration/initial", exploration_initial_, true);
    nh_private_.param("exploration/climb", exploration_climb_, 1.5);
    nh_private_.param("exploration/settle", exploration_settle_, 5.0);
    nh_private_.param("exploration/return_to_start", exploration_return_, true);

    // Timer
    param_loader.loadParam("timer_main/rate", timer_main_rate);

    // Initialize UAV as state IDLE
    state_ = STATE_IDLE;
    iteration_ = 0;
    reset_velocity = true;
    initial_offset = {0, 0, 0};

    // Get vertical FoV and setup camera
    vertical_fov = segment_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    segment_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    segment_evaluator.setEsdfMap(esdf_map_);

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("KRH_NBVP_rw");
    transformer_->setDefaultFrame(frame_id);
    transformer_->retryLookupNewest(true);

    set_variables = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();

    // Get Sampling Radius
    bounded_radius = sqrt(pow(min_x - max_x, 2.0) + pow(min_y - max_y, 2.0) + pow(min_z - max_z, 2.0));

    /* Publishers */
    pub_markers = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_setpoint = nh_private_.advertise<mavros_msgs::PositionTarget>("setpoint_out", 10);
    pub_offset = nh_private_.advertise<geometry_msgs::Point>("offset_out", 10);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "KRH_NBVP_rw";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_local_pose_diag = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "local_pose_in", &KRH_NBVP_rw::callbackLocalPose, this);
    sub_state = nh_private_.subscribe("state_in", 10, &KRH_NBVP_rw::callbackState, this);
    sub_local_velocity_diag = mrs_lib::SubscribeHandler<geometry_msgs::TwistStamped>(shopts, "local_velocity_in", &KRH_NBVP_rw::callbackLocalVelocity, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &KRH_NBVP_rw::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &KRH_NBVP_rw::callbackStop, this);
    ss_offset = nh_private_.advertiseService("offset_in", &KRH_NBVP_rw::callbackOffset, this);

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &KRH_NBVP_rw::timerMain, this);

    is_initialized = true;
}

double KRH_NBVP_rw::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool KRH_NBVP_rw::isTrajectoryCollisionFree(kino_rrt_star::Trajectory* trajectory) const {
    kino_rrt_star::Node* node = trajectory->TrajectoryPoints.back().get();
    if (getMapDistance(node->point.head(3)) < uav_radius) {
        return false;
    }
    return true;
}

void KRH_NBVP_rw::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.2).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[KRH_NBVP_rw]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void KRH_NBVP_rw::planStep() {
    best_score_ = 0.0;
    kino_rrt_star::Trajectory* best_trajectory = nullptr;

    double node_size = 0.2;

    std::unique_ptr<kino_rrt_star::Node> root_node;
    std::unique_ptr<kino_rrt_star::Trajectory> Root;
    if (best_branch.size() > 1) {
        root_node = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, best_branch[1]->TrajectoryPoints.back()->velocity, best_branch[1]->TrajectoryPoints.back()->acceleration);
        Root = std::make_unique<kino_rrt_star::Trajectory>(std::move(root_node));
    } else {
        root_node = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d(0, 0, 0));
        Root = std::make_unique<kino_rrt_star::Trajectory>(std::move(root_node));
    }

    Root->cost = 0.0;
    Root->score = 0.0;

    if (Root->score > best_score_) {
        best_score_ = Root->score;
        best_trajectory = Root.get();
    }

    KinoRRTStar.clearKDTree();
    kino_rrt_star::Trajectory* root_ptr = KinoRRTStar.addKDTreeTrajectory(std::move(Root));
    clearMarkers();
    visualize_node(root_ptr->TrajectoryPoints.back()->point, 2 * node_size, ns);

    bool isFirstIteration = true;
    int j = 1;
    collision_id_counter_ = 0;
    int expanded_num_nodes = 0;
    if (best_branch.size() > 0) {
        previous_trajectory = best_branch[0]->clone();
    }
    ros::WallTime plan_start_ = ros::WallTime::now();
    while (j < N_max || best_score_ <= 0.0) {
        // Backtrack When Stuck
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) {
                executed_path_.pop_back();
            }
            if (!executed_path_.empty()) {
                retreating_ = true;
                reset_velocity = true;
                ROS_WARN("[KRH_NBVP_rw]: Backtracking (%s after %.1fs, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", plan_elapsed, j, executed_path_.size());
                best_branch.clear();
                return;
            }
            ROS_INFO("[KRH_NBVP_rw]: Backtrack Rotation");
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

            double result_best = segment_evaluator.computeFixedGainRaycasting(trajectory_point, initial_offset);
            raw_best->gain = result_best;

            segment_evaluator.computeCostTwo(raw_best);
            segment_evaluator.computeScore(raw_best, lambda, lambda2);

            if (raw_best->score > best_score_) {
                best_score_ = raw_best->score;
                best_trajectory = raw_best;
            }

            ROS_INFO("[KRH_NBVP_rw]: Best Score BB: %f", raw_best->score);

            kino_rrt_star::Trajectory* added_bb = KinoRRTStar.addKDTreeTrajectory(std::move(raw_best_owned));
            visualize_trajectory(added_bb, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > 0.0) {
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

            std::unique_ptr<kino_rrt_star::Trajectory> new_trajectory;
            new_trajectory = std::make_unique<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory(nearest_trajectory, max_velocity, reset_velocity, rand_point_yaw[3], accel, max_heading_velocity, max_heading_accel, step_size, new_trajectory);
            new_trajectory->TrajectoryPoints.back()->point[3] = rand_point_yaw[3];

            bool OutOfBounds = false;

            if (new_trajectory->TrajectoryPoints.back()->point[0] > initial_offset[0] + max_x || new_trajectory->TrajectoryPoints.back()->point[0] < initial_offset[0] + min_x || new_trajectory->TrajectoryPoints.back()->point[1] < initial_offset[1] + min_y || new_trajectory->TrajectoryPoints.back()->point[1] > initial_offset[1] + max_y || new_trajectory->TrajectoryPoints.back()->point[2] < initial_offset[2] + min_z || new_trajectory->TrajectoryPoints.back()->point[2] > initial_offset[2] + max_z) {
                OutOfBounds = true;
                break;
            }

            if (OutOfBounds) {
                // Avoid Memory Leak
                new_trajectory.reset();
                continue;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(new_trajectory.get())) {
                collision_id_counter_++;
                /*if (collision_id_counter_ > 1000 * j) {
                    break;
                }*/
                // Avoid Memory Leak
                new_trajectory.reset();
                continue;
            }

            visualize_node(new_trajectory->TrajectoryPoints.back()->point, node_size, ns);
            ++accel_iteration;

            Eigen::Vector4d trajectory_point_gain = new_trajectory->TrajectoryPoints.back()->point;
            double result = segment_evaluator.computeFixedGainRaycasting(trajectory_point_gain, initial_offset);
            new_trajectory->gain = result;

            segment_evaluator.computeCostTwo(new_trajectory.get());
            segment_evaluator.computeScore(new_trajectory.get(), lambda, lambda2);

            if (new_trajectory->score > best_score_) {
                best_score_ = new_trajectory->score;
                best_trajectory = new_trajectory.get();
            }

            ROS_INFO("[KRH_NBVP_rw]: Best Score: %f", new_trajectory->score);

            kino_rrt_star::Trajectory* added = KinoRRTStar.addKDTreeTrajectory(std::move(new_trajectory));
            visualize_trajectory(added, ns);
        }

        if (accel_iteration == 0) {
            continue;
        }

        expanded_num_nodes += accel_iteration;

        if (j > N_termination) {
            ROS_INFO("[KRH_NBVP_rw]: KRH_NBVP Terminated");
            KinoRRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            changeState(STATE_STOPPED);
            break;
        }

        ++j;
    }

    ROS_INFO("[KRH_NBVP_rw]: Final Best Score: %f", best_score_);
    ROS_INFO("[KRH_NBVP_rw]: Node Iterations: %d", j);
    ROS_INFO("[KRH_NBVP_rw]: Full Node Iterations: %d", expanded_num_nodes);

    if (best_trajectory) {
        reset_velocity = false;
        next_best_trajectory = best_trajectory;
        KinoRRTStar.backtrackTrajectory(best_trajectory, best_branch, next_best_trajectory);
        visualize_best_trajectory(best_trajectory, ns);
    }
}

void KRH_NBVP_rw::captureOffset() {
    initial_offset = pose.head<3>();
    // Ground Height at Arming
    if (have_ground_z_) {
        initial_offset.z() = ground_z_;
    } else {
        initial_offset.z() = 0.0;
        ROS_WARN(
            "[KRH_NBVP_rw]: never saw the disarmed to armed edge, using z offset 0. Start the "
            "planner stack before arming to correct the barometric bias.");
    }

    geometry_msgs::Point offset_msg;
    offset_msg.x = initial_offset.x();
    offset_msg.y = initial_offset.y();
    offset_msg.z = initial_offset.z();
    pub_offset.publish(offset_msg);

    ROS_INFO("[KRH_NBVP_rw]: Start offset captured: [%.2f, %.2f, %.2f]", initial_offset.x(), initial_offset.y(), initial_offset.z());
}

mavros_msgs::PositionTarget KRH_NBVP_rw::makeSetpoint(const Eigen::Vector4d& waypoint) {
    mavros_msgs::PositionTarget sp;
    sp.header.frame_id = frame_id;
    sp.header.stamp = ros::Time::now();
    // Local NED Frame
    sp.coordinate_frame = 1;
    sp.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    sp.position.x = waypoint[0];
    sp.position.y = waypoint[1];
    sp.position.z = waypoint[2];
    sp.yaw = waypoint[3];
    return sp;
}

void KRH_NBVP_rw::rotate() {
    // Rotate 360 degrees
    const int steps = std::max(3, (int)std::ceil(360.0 / rotation_step_deg_));
    const double step = 2.0 * M_PI / steps;
    for (int s = 1; s <= steps; ++s) {
        Eigen::Vector4d wp = pose;
        wp[3] = pose[3] + s * step;
        pub_setpoint.publish(makeSetpoint(wp));
        ros::Duration(rotation_settle_).sleep();
    }
}

void KRH_NBVP_rw::retreat(const Eigen::Vector4d& waypoint) {
    // Straight Line to the Previous Node at max_velocity, one setpoint every 0.1 s like the trajectories
    const Eigen::Vector4d start = pose;
    const Eigen::Vector3d d = waypoint.head<3>() - start.head<3>();
    const int steps = std::max(1, (int)std::ceil(d.norm() / (max_velocity * 0.1)));
    for (int s = 1; s <= steps; ++s) {
        Eigen::Vector4d wp = waypoint;
        wp.head<3>() = start.head<3>() + d * ((double)s / steps);
        pub_setpoint.publish(makeSetpoint(wp));
        ros::Duration(0.1).sleep();
    }
}

void KRH_NBVP_rw::explorationSweep() {
    // Up, Rotate, Down
    const Eigen::Vector4d start = pose;
    const double ceiling = initial_offset[2] + (double)max_z - uav_radius;
    double z_top = std::min(start[2] + exploration_climb_, ceiling);
    if (z_top <= start[2] + 0.05) {
        ROS_WARN("[KRH_NBVP_rw]: Exploration sweep skipped: no headroom (z %.2f, ceiling %.2f).",
                 start[2], ceiling);
        return;
    }
    ROS_INFO("[KRH_NBVP_rw]: Exploration sweep: up %.2f -> %.2f m, rotate, down.", start[2], z_top);

    Eigen::Vector4d up = start;
    up[2] = z_top;
    pub_setpoint.publish(makeSetpoint(up));
    ros::Duration(exploration_settle_).sleep();

    const int steps = std::max(3, (int)std::ceil(360.0 / rotation_step_deg_));
    const double step = 2.0 * M_PI / steps;
    for (int i = 1; i <= steps; ++i) {
        Eigen::Vector4d wp = up;
        wp[3] = start[3] + i * step;
        pub_setpoint.publish(makeSetpoint(wp));
        ros::Duration(rotation_settle_).sleep();
    }

    if (exploration_return_) {
        pub_setpoint.publish(makeSetpoint(start));
        ros::Duration(exploration_settle_).sleep();
    }
    ROS_INFO("[KRH_NBVP_rw]: Exploration sweep done.");
}

bool KRH_NBVP_rw::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KRH_NBVP_rw]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    captureOffset();
    changeState(STATE_PLANNING);
    pending_exploration_ = exploration_initial_;

    res.success = true;
    res.message = "starting";
    return true;
}

bool KRH_NBVP_rw::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KRH_NBVP_rw]: " << ss.str());

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

bool KRH_NBVP_rw::callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KRH_NBVP_rw]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }
    captureOffset();

    std::stringstream ss;
    ss << "Getting initial position offset: [" << initial_offset[0] << ", " << initial_offset[1] << ", " << initial_offset[2] << "]";

    res.success = true;
    res.message = ss.str();
    return true;
}

void KRH_NBVP_rw::callbackState(const mavros_msgs::State::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    // First Arming Only
    if (msg->armed && !prev_armed_ && have_pose_ && !have_ground_z_) {
        ground_z_ = pose.z();
        have_ground_z_ = true;
        ROS_INFO("[KRH_NBVP_rw]: armed on the ground, latching z = %.2f m as the takeoff reference.",
                 ground_z_);
    }
    prev_armed_ = msg->armed;
}

void KRH_NBVP_rw::callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KRH_NBVP_rw]: getting LocalPose diagnostics");
    uav_local_pose = msg->pose;

    const geometry_msgs::Quaternion& q = uav_local_pose.orientation;

    // Check for NaNs or zero-length quaternion
    if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
        ROS_ERROR("[KRH_NBVP_rw]: Invalid quaternion received (contains NaNs)");
        return;
    }

    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm < 0.1 || norm > 1.1) {
        ROS_WARN_THROTTLE(5, "[KRH_NBVP_rw] Invalid quaternion detected. Norm: %.3f. Skipping this pose.", norm);
        return;
    }

    // Reject Wild Poses
    const geometry_msgs::Point& p = uav_local_pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x) > pose_max_distance_ || std::abs(p.y) > pose_max_distance_ ||
        std::abs(p.z) > pose_max_distance_) {
        ROS_WARN_THROTTLE(5, "[%s]: implausible pose [%.3g, %.3g, %.3g], skipping.",
                          "KRH_NBVP_rw", p.x, p.y, p.z);
        return;
    }
    if (have_pose_) {
        const double dt = (ros::Time::now() - last_pose_time_).toSec();
        const double jump = std::sqrt(std::pow(p.x - pose[0], 2) + std::pow(p.y - pose[1], 2) +
                                      std::pow(p.z - pose[2], 2));
        if (dt > 1e-3 && jump / dt > pose_max_speed_) {
            ROS_WARN_THROTTLE(5, "[%s]: pose jumped %.2f m in %.3f s, skipping.",
                              "KRH_NBVP_rw", jump, dt);
            return;
        }
    }

    double yaw = 0.0;
    try {
        yaw = mrs_lib::getYaw(uav_local_pose);
    } catch (const mrs_lib::AttitudeConverter::InvalidAttitudeException& e) {
        ROS_ERROR_THROTTLE(1.0, "[KRH_NBVP_rw]: Exception during getYaw(): %s — skipping this pose.", e.what());
        return;
    }

    pose = {uav_local_pose.position.x, uav_local_pose.position.y, uav_local_pose.position.z, yaw};
    last_pose_time_ = ros::Time::now();
    have_pose_ = true;
}

void KRH_NBVP_rw::callbackLocalVelocity(const geometry_msgs::TwistStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KRH_NBVP_rw]: getting LocalVelocity diagnostics");
    geometry_msgs::Twist uav_velocity = msg->twist;
    velocity = {uav_velocity.linear.x, uav_velocity.linear.y, uav_velocity.linear.z};
}

void KRH_NBVP_rw::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    const bool got_local_pose = have_pose_ && (ros::Time::now() - last_pose_time_).toSec() < 2.0;

    if (!got_local_pose) {
        ROS_INFO_THROTTLE(1.0, "[KRH_NBVP_rw]: waiting for data: LocalPose = FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[KRH_NBVP_rw]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[KRH_NBVP_rw]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[KRH_NBVP_rw]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            ROS_INFO("[KRH_NBVP_rw]: waiting for command");
            break;
        }
        case STATE_PLANNING: {
            if (pending_exploration_) {
                pending_exploration_ = false;
                explorationSweep();
            }

            retreating_ = false;
            planStep();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            // Retreat to Previous Node
            if (retreating_ && !executed_path_.empty()) {
                iteration_ += 1;
                retreat(executed_path_.back());
                changeState(STATE_MOVING);
                break;
            }

            iteration_ += 1;

            // Store Flown Path
            const kino_rrt_star::Trajectory* flown_parent = next_best_trajectory->parent;
            if (executed_path_.empty()) {
                executed_path_.push_back(flown_parent ? flown_parent->TrajectoryPoints.front()->point : pose);
            }
            if (flown_parent && flown_parent->parent) {
                executed_path_.push_back(flown_parent->TrajectoryPoints.back()->point);
            }
            executed_path_.push_back(next_best_trajectory->TrajectoryPoints.back()->point);

            visualize_frustum(next_best_trajectory->TrajectoryPoints.back().get());
            visualize_unknown_voxels(next_best_trajectory->TrajectoryPoints.back().get());

            mavros_msgs::PositionTarget setpoint_reference;

            setpoint_reference.header.frame_id = frame_id;
            setpoint_reference.header.stamp = ros::Time::now();
            setpoint_reference.coordinate_frame = 1;
            setpoint_reference.type_mask = 2496;

            if (next_best_trajectory->parent) {
                setpoint_reference.position.x = next_best_trajectory->parent->TrajectoryPoints.back()->point[0];
                setpoint_reference.position.y = next_best_trajectory->parent->TrajectoryPoints.back()->point[1];
                setpoint_reference.position.z = next_best_trajectory->parent->TrajectoryPoints.back()->point[2];
                setpoint_reference.velocity.x = next_best_trajectory->parent->TrajectoryPoints.back()->velocity[0];
                setpoint_reference.velocity.y = next_best_trajectory->parent->TrajectoryPoints.back()->velocity[1];
                setpoint_reference.velocity.z = next_best_trajectory->parent->TrajectoryPoints.back()->velocity[2];
                //setpoint_reference.acceleration_or_force.x = next_best_trajectory->parent->TrajectoryPoints.back()->acceleration[0];
                //setpoint_reference.acceleration_or_force.y = next_best_trajectory->parent->TrajectoryPoints.back()->acceleration[1];
                //setpoint_reference.acceleration_or_force.z = next_best_trajectory->parent->TrajectoryPoints.back()->acceleration[2];
                setpoint_reference.yaw = next_best_trajectory->parent->TrajectoryPoints.back()->point[3];
                pub_setpoint.publish(setpoint_reference);
            }

            ros::Duration(0.1).sleep();

            for (size_t i = 0; i < next_best_trajectory->TrajectoryPoints.size(); i++) {
                setpoint_reference.position.x = next_best_trajectory->TrajectoryPoints[i]->point[0];
                setpoint_reference.position.y = next_best_trajectory->TrajectoryPoints[i]->point[1];
                setpoint_reference.position.z = next_best_trajectory->TrajectoryPoints[i]->point[2];
                setpoint_reference.velocity.x = next_best_trajectory->TrajectoryPoints[i]->velocity[0];
                setpoint_reference.velocity.y = next_best_trajectory->TrajectoryPoints[i]->velocity[1];
                setpoint_reference.velocity.z = next_best_trajectory->TrajectoryPoints[i]->velocity[2];
                //setpoint_reference.acceleration_or_force.x = next_best_trajectory->TrajectoryPoints[i]->acceleration[0];
                //setpoint_reference.acceleration_or_force.y = next_best_trajectory->TrajectoryPoints[i]->acceleration[1];
                //setpoint_reference.acceleration_or_force.z = next_best_trajectory->TrajectoryPoints[i]->acceleration[2];
                setpoint_reference.yaw = next_best_trajectory->TrajectoryPoints[i]->point[3];
                pub_setpoint.publish(setpoint_reference);

                // Break before the end of the trajectory
                if (i >= next_best_trajectory->TrajectoryPoints.size() - 2) {
                    break;
                }

                ros::Duration(0.1).sleep();
            }

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            ROS_INFO("[KRH_NBVP_rw]: waiting for command");
            changeState(STATE_PLANNING);
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[KRH_NBVP_rw]: Total Iterations: %d", iteration_);
            ROS_INFO("[KRH_NBVP_rw]: Shutting down.");
            // Setpoint to stop velocity and acceleration controls
            mavros_msgs::PositionTarget setpoint_reference;
            setpoint_reference.header.frame_id = frame_id;
            setpoint_reference.header.stamp = ros::Time::now();
            setpoint_reference.coordinate_frame = 1;
            setpoint_reference.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            setpoint_reference.velocity.x = 0;
            setpoint_reference.velocity.y = 0;
            setpoint_reference.velocity.z = 0;
            setpoint_reference.acceleration_or_force.x = 0;
            setpoint_reference.acceleration_or_force.y = 0;
            setpoint_reference.acceleration_or_force.z = 0;
            pub_setpoint.publish(setpoint_reference);
            ros::shutdown();
            return;
        }
    }
}

void KRH_NBVP_rw::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[KRH_NBVP_rw]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[KRH_NBVP_rw]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void KRH_NBVP_rw::visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns) {
    visualization_msgs::Marker n;
    n.header.stamp = ros::Time::now();
    n.header.seq = node_id_counter_;
    n.header.frame_id = frame_id;
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

    node_id_counter_++;

    n.lifetime = ros::Duration(30.0);
    n.frame_locked = false;
    pub_markers.publish(n);
}

void KRH_NBVP_rw::visualize_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.stamp = ros::Time::now();
    trajectory_marker.header.frame_id = frame_id;
    trajectory_marker.id = trajectory_id_counter_;
    trajectory_marker.ns = "trajectory";
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;

    trajectory_marker.color.r = 1.0;
    trajectory_marker.color.g = 0.3;
    trajectory_marker.color.b = 0.7;
    trajectory_marker.color.a = 1.0;

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

void KRH_NBVP_rw::visualize_best_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
    kino_rrt_star::Trajectory* currentTrajectory = trajectory;

    while (currentTrajectory->parent) {
        visualization_msgs::Marker best_trajectory_marker;
        best_trajectory_marker.header.stamp = ros::Time::now();
        best_trajectory_marker.header.seq = best_trajectory_id_counter_;
        best_trajectory_marker.header.frame_id = frame_id;
        best_trajectory_marker.id = best_trajectory_id_counter_;
        best_trajectory_marker.ns = "best_trajectory";
        best_trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
        best_trajectory_marker.action = visualization_msgs::Marker::ADD;

        best_trajectory_marker.color.r = 0.7;
        best_trajectory_marker.color.g = 0.7;
        best_trajectory_marker.color.b = 0.3;
        best_trajectory_marker.color.a = 1.0;

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

void KRH_NBVP_rw::visualize_frustum(kino_rrt_star::Node* position) {
    Eigen::Vector4d trajectory_point_visualize = position->point;

    visualization_msgs::Marker frustum;
    frustum.header.frame_id = frame_id;
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

void KRH_NBVP_rw::visualize_unknown_voxels(kino_rrt_star::Node* position) {
    Eigen::Vector4d trajectory_point_visualize = position->point;

    voxblox::Pointcloud voxel_points;
    segment_evaluator.visualizeGain(trajectory_point_visualize, voxel_points);

    visualization_msgs::MarkerArray voxels_marker;
    for (size_t i = 0; i < voxel_points.size(); ++i) {
        visualization_msgs::Marker unknown_voxel;
        unknown_voxel.header.frame_id = frame_id;
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

void KRH_NBVP_rw::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void KRH_NBVP_rw::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void KRH_NBVP_rw::clearMarkers() {
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
