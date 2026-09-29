#include "motion_planning_real_world/KAEP/KAEP_rw.h"

KAEP_rw::KAEP_rw(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "KAEP_rw");

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
    param_loader.loadParam("local_planning/N_max", N_max);
    param_loader.loadParam("local_planning/N_termination", N_termination);
    param_loader.loadParam("local_planning/N_yaw_samples", num_yaw_samples);
    param_loader.loadParam("local_planning/radius", radius);
    param_loader.loadParam("local_planning/step_size", step_size);
    param_loader.loadParam("local_planning/tolerance", tolerance);
    param_loader.loadParam("local_planning/g_zero", g_zero);

    // RRT* Tree (global Planning)
    param_loader.loadParam("global_planning/N_min_nodes", N_min_nodes);
    param_loader.loadParam("global_planning/global_max_acceleration_iterations", global_max_accel_iterations);

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
    param_loader.loadParam("path/global_lambda", global_lambda);
    param_loader.loadParam("path/global_lambda2", global_lambda2);
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
    reset_velocity = false;
    node_size = 0.2;
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
    transformer_ = std::make_unique<mrs_lib::Transformer>("KAEP_rw");
    transformer_->setDefaultFrame(frame_id);
    transformer_->retryLookupNewest(true);

    set_variables = false;
    goto_global_planning = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();

    // Get Sampling Radius
    bounded_radius = sqrt(pow(min_x - max_x, 2.0) + pow(min_y - max_y, 2.0) + pow(min_z - max_z, 2.0));

    /* Publishers */
    pub_markers = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_out", 500);
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_node = nh_private_.advertise<cache_nodes::Node>("tree_node_out", 500);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_setpoint = nh_private_.advertise<mavros_msgs::PositionTarget>("setpoint_out", 10);
    pub_offset = nh_private_.advertise<geometry_msgs::Point>("offset_out", 10);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "KAEP_rw";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_local_pose_diag = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "local_pose_in", &KAEP_rw::callbackLocalPose, this);
    sub_state = nh_private_.subscribe("state_in", 10, &KAEP_rw::callbackState, this);
    sub_local_velocity_diag = mrs_lib::SubscribeHandler<geometry_msgs::TwistStamped>(shopts, "local_velocity_in", &KAEP_rw::callbackLocalVelocity, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &KAEP_rw::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &KAEP_rw::callbackStop, this);
    ss_offset = nh_private_.advertiseService("offset_in", &KAEP_rw::callbackOffset, this);

    /* Service Clients */
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &KAEP_rw::timerMain, this);

    is_initialized = true;
}

double KAEP_rw::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool KAEP_rw::isTrajectoryCollisionFree(kino_rrt_star::Trajectory* trajectory) const {
    kino_rrt_star::Node* node = trajectory->TrajectoryPoints.back().get();
    if (getMapDistance(node->point.head(3)) < uav_radius) {
        return false;
    }

    return true;
}

void KAEP_rw::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.2).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[KAEP_rw]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void KAEP_rw::planStep() {
    localPlanner();
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_trajectory = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[KAEP_rw]: Getting Global Frontiers");
        //getGlobalFrontiers(GlobalFrontiers);
        while (GlobalFrontiers.size() == 0 && g_zero != 0.0) {
            getGlobalFrontiers(GlobalFrontiers);
            if (GlobalFrontiers.size() == 0) {
                g_zero = g_zero / 2;
                // Ignore any gain smaller than 0.1
                if (g_zero < 1.0) {
                    g_zero = 0.0;
                }
                ROS_INFO("[KAEP_rw]: Changed g_zero to %f", g_zero);
            }
        }
        if (GlobalFrontiers.size() == 0) {
            changeState(STATE_STOPPED);
            return;
        }
        ROS_INFO("[KAEP_rw]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_trajectory);
        if (retreating_) {
            return;
        }

        if (go_terminate) {
            ROS_INFO("[KAEP_rw]: No information gain. Terminate.");
            changeState(STATE_STOPPED);
            return;
        }

        next_best_trajectory = best_global_trajectory;
        goto_global_planning = false;
    }
}

void KAEP_rw::localPlanner() {
    best_score_ = 0;
    kino_rrt_star::Trajectory* best_trajectory = nullptr;

    std::unique_ptr<kino_rrt_star::Node> root_node;
    std::unique_ptr<kino_rrt_star::Trajectory> Root;
    if (best_branch.size() > 1) {
        if (!reset_velocity) {
            root_node = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, best_branch[1]->TrajectoryPoints.back()->velocity, best_branch[1]->TrajectoryPoints.back()->acceleration);
        } else {
            root_node = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
        Root = std::make_unique<kino_rrt_star::Trajectory>(std::move(root_node));
    } else {
        if (!reset_velocity) {
            root_node = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
        } else {
            root_node = std::make_unique<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
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
    while (j < N_max || best_score_ <= g_zero) {
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
                ROS_WARN("[KAEP_rw]: Backtracking (%s after %.1fs, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", plan_elapsed, j, executed_path_.size());
                best_branch.clear();
                return;
            }
            ROS_INFO("[KAEP_rw]: Backtrack Rotation");
            rotate();
            plan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        // Add previous best branch
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
            std::pair<double, double> result_best = segment_evaluator.computeGainRaycasting(trajectory_point, true, initial_offset);
            raw_best->gain = result_best.first;

            if (result_best.second > M_PI) {
                result_best.second -= 2 * M_PI;
            }

            raw_best->TrajectoryPoints.back()->point[3] = result_best.second;
            KinoRRTStar.steer_trajectory_angular(nearest_trajectory_best, result_best.second, max_heading_velocity, max_heading_accel, raw_best);

            // Make sure the heading of the last node is correct
            raw_best->TrajectoryPoints.back()->point[3] = result_best.second;

            segment_evaluator.computeCostTwo(raw_best);
            segment_evaluator.computeScore(raw_best, lambda, lambda2);

            if (raw_best->score > best_score_) {
                best_score_ = raw_best->score;
                best_trajectory = raw_best;
            }

            ROS_INFO("[KAEP_rw]: Best Score BB: %f", raw_best->score);

            kino_rrt_star::Trajectory* added_bb = KinoRRTStar.addKDTreeTrajectory(std::move(raw_best_owned));
            visualize_trajectory(added_bb, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > g_zero) {
            break;
        }

        best_branch.clear();

        Eigen::Vector3d rand_point;
        KinoRRTStar.computeSamplingDimensions(bounded_radius, rand_point);
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
            KinoRRTStar.steer_trajectory_linear(nearest_trajectory, max_velocity, reset_velocity, accel, step_size, new_trajectory);

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

            trajectory_point = new_trajectory->TrajectoryPoints.back()->point;
            std::pair<double, double> result = segment_evaluator.computeGainRaycasting(trajectory_point, true, initial_offset);
            new_trajectory->gain = result.first;

            // Convert from [0, 2*PI[ to [-PI, PI[
            if (result.second > M_PI) {
                result.second -= 2 * M_PI;
            }

            new_trajectory->TrajectoryPoints.back()->point[3] = result.second;
            KinoRRTStar.steer_trajectory_angular(nearest_trajectory, result.second, max_heading_velocity, max_heading_accel, new_trajectory.get());

            // Make sure the heading of the last node is correct
            new_trajectory->TrajectoryPoints.back()->point[3] = result.second;

            segment_evaluator.computeCostTwo(new_trajectory.get());
            segment_evaluator.computeScore(new_trajectory.get(), lambda, lambda2);

            if (new_trajectory->score > best_score_) {
                best_score_ = new_trajectory->score;
                best_trajectory = new_trajectory.get();
            }

            ROS_INFO("[KAEP_rw]: Best Score: %f", new_trajectory->score);

            if (new_trajectory->gain >= 0.5) {
                cacheNode(new_trajectory.get());
            }

            kino_rrt_star::Trajectory* added = KinoRRTStar.addKDTreeTrajectory(std::move(new_trajectory));
            visualize_trajectory(added, ns);
        }

        if (accel_iteration == 0) {
            continue;
        }

        expanded_num_nodes += accel_iteration;

        if (j > N_termination) {
            ROS_INFO("[KAEP_rw]: Going to Global Planning");
            KinoRRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            goto_global_planning = true;
            return;
        }

        ++j;
    }

    ROS_INFO("[KAEP_rw]: Final Best Score: %f", best_score_);
    ROS_INFO("[KAEP_rw]: Node Iterations: %d", j);
    ROS_INFO("[KAEP_rw]: Full Node Iterations: %d", expanded_num_nodes);

    if (best_trajectory) {
        reset_velocity = false;
        next_best_trajectory = best_trajectory;
        KinoRRTStar.backtrackTrajectoryAEP(best_trajectory, best_branch);
        visualize_best_trajectory(best_trajectory, ns);
    }

    for (int k = 1; k < static_cast<int>(best_branch.size()); ++k) {
        if (best_branch[k]->gain > g_zero) {
            next_best_trajectory = best_branch[k].get();
            previous_best_global_trajectory = best_branch[k].get();
            break;
        }
    }
}

void KAEP_rw::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, kino_rrt_star::Trajectory*& best_global_trajectory) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[KAEP_rw]: Terminate AEP");

        KinoRRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    std::unique_ptr<kino_rrt_star::Node> global_root_node;

    if (!reset_velocity) {
        global_root_node = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
    } else {
        global_root_node = std::make_unique<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }

    Eigen::Vector3d global_root_point = global_root_node->point.head(3);
    std::unique_ptr<kino_rrt_star::Trajectory> global_root = std::make_unique<kino_rrt_star::Trajectory>(std::move(global_root_node));
    kino_rrt_star::Trajectory* global_root_ptr = KinoRRTStar.addKDTreeTrajectory(std::move(global_root));
    (void)global_root_ptr;

    std::vector<kino_rrt_star::Trajectory*> all_global_goals;

    int m = 0;
    ros::WallTime gplan_start_ = ros::WallTime::now();
    while (m < N_min_nodes || all_global_goals.size() <= 0) {
        // Backtrack When Stuck
        const double gplan_elapsed = (ros::WallTime::now() - gplan_start_).toSec();
        const bool g_boxed = gplan_elapsed > recovery_boxed_deadline_ && m < recovery_min_tree_;
        const bool g_timed = gplan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (g_boxed || g_timed)) {
            if (!executed_path_.empty()) {
                executed_path_.pop_back();
            }
            if (!executed_path_.empty()) {
                retreating_ = true;
                reset_velocity = true;
                ROS_WARN("[KAEP_rw]: Global Backtracking (%s after %.1fs, tree=%d) -> executed node %zu",
                         g_boxed ? "boxed-in" : "timeout", gplan_elapsed, m, executed_path_.size());
                return;
            }
            ROS_INFO("[KAEP_rw]: Backtrack Rotation");
            rotate();
            gplan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }
        Eigen::Vector3d global_rand_point;
        KinoRRTStar.computeSamplingDimensions(bounded_radius, global_rand_point);
        global_rand_point += global_root_point;

        kino_rrt_star::Trajectory* global_nearest_trajectory = nullptr;
        KinoRRTStar.findNearestKD(global_rand_point, global_nearest_trajectory);

        int accel_iteration = 0;
        int accel_tries = 0;
        while (accel_iteration < global_max_accel_iterations && accel_tries < 100 * global_max_accel_iterations) {
            accel_tries++;
            Eigen::Vector3d accel;
            KinoRRTStar.computeAccelerationSampling(max_accel, accel);

            std::unique_ptr<kino_rrt_star::Trajectory> global_new_trajectory;
            global_new_trajectory = std::make_unique<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory_linear(global_nearest_trajectory, max_velocity, reset_velocity, accel, step_size, global_new_trajectory);
            bool OutOfBounds = false;

            if (global_new_trajectory->TrajectoryPoints.back()->point[0] > initial_offset[0] + max_x || global_new_trajectory->TrajectoryPoints.back()->point[0] < initial_offset[0] + min_x || global_new_trajectory->TrajectoryPoints.back()->point[1] < initial_offset[1] + min_y || global_new_trajectory->TrajectoryPoints.back()->point[1] > initial_offset[1] + max_y || global_new_trajectory->TrajectoryPoints.back()->point[2] < initial_offset[2] + min_z || global_new_trajectory->TrajectoryPoints.back()->point[2] > initial_offset[2] + max_z) {
                OutOfBounds = true;
                break;
            }

            if (OutOfBounds) {
                // Avoid Memory Leak
                global_new_trajectory.reset();
                continue;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(global_new_trajectory.get())) {
                collision_id_counter_++;
                // Avoid Memory Leak
                global_new_trajectory.reset();
                continue;
            }

            visualize_node(global_new_trajectory->TrajectoryPoints.back()->point, node_size, ns);
            ++accel_iteration;

            segment_evaluator.computeCostTwo(global_new_trajectory.get());

            kino_rrt_star::Trajectory* added_global = KinoRRTStar.addKDTreeTrajectory(std::move(global_new_trajectory));
            visualize_trajectory(added_global, ns);

            bool goal_reached = getGlobalGoal(GlobalFrontiers, added_global);
            if (goal_reached) {
                segment_evaluator.computeSingleScore(added_global, global_lambda, global_lambda2);
                all_global_goals.push_back(added_global);
                goal_reached = false;
            }
        }

        if (accel_iteration == 0) {
            continue;
        }

        ++m;
    }

    ROS_INFO("[KAEP_rw]: Global Planner Ends");

    getBestGlobalTrajectory(all_global_goals, best_global_trajectory);
    all_global_goals.clear();
}

void KAEP_rw::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
    cache_nodes::BestNode srv;
    srv.request.threshold = g_zero;
    GlobalFrontiers.clear();
    if (sc_best_node.call(srv)) {
        double best_global_gain = -1.0;
        Eigen::Vector3d best_global_frontier = Eigen::Vector3d::Zero();
        for (int i = 0; i < srv.response.best_node.size(); ++i) {
            Eigen::Vector3d frontier;
            frontier[0] = srv.response.best_node[i].x;
            frontier[1] = srv.response.best_node[i].y;
            frontier[2] = srv.response.best_node[i].z;
            GlobalFrontiers.push_back(frontier);
        }
    }
}

bool KAEP_rw::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, kino_rrt_star::Trajectory* trajectory) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    for (size_t i = 0; i < GlobalFrontiers.size(); ++i) {
        goals_tree.addKDTreePoint(GlobalFrontiers[i]);
    }

    // Find the nearest node in the KD Tree
    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(trajectory->TrajectoryPoints.back()->point.head(3), nearest_goal);
    if (nearest_goal.size() <= 0) {
        goals_tree.clearKDTreePoints();
        return false;
    }

    if ((nearest_goal - trajectory->TrajectoryPoints.back()->point.head(3)).norm() < tolerance) {
        //ROS_INFO("[KAEP_rw]: Goal: [%f, %f, %f]", nearest_goal[0], nearest_goal[1], nearest_goal[2]);
        //ROS_INFO("[KAEP_rw]: RRT* Goal: [%f, %f, %f]", trajectory->TrajectoryPoints.back()->point[0], trajectory->TrajectoryPoints.back()->point[1], trajectory->TrajectoryPoints.back()->point[2]);

        Eigen::Vector4d trajectory_point_global = trajectory->TrajectoryPoints.back()->point;
        std::pair<double, double> result = segment_evaluator.computeGainRaycasting(trajectory_point_global, true, initial_offset);
        trajectory->gain = result.first;

        // Convert from [0, 2*PI[ to [-PI, PI[
        if (result.second > M_PI) {
            result.second -= 2 * M_PI;
        }

        trajectory->TrajectoryPoints.back()->point[3] = result.second;
        KinoRRTStar.steer_trajectory_angular(trajectory->parent, result.second, max_heading_velocity, max_heading_accel, trajectory);

        // Make sure the heading of the last node is correct
        trajectory->TrajectoryPoints.back()->point[3] = result.second;

        trajectory_point_global.head<3>() = nearest_goal;
        trajectory_point_global[3] = 0.0;
        std::pair<double, double> result_original = segment_evaluator.computeGainRaycasting(trajectory_point_global, true, initial_offset);
        //ROS_INFO("[KAEP_rw]: Goal Best Gain: %f", result_original.first);
        goals_tree.clearKDTreePoints();
        return true;
    }

    goals_tree.clearKDTreePoints();
    return false;
}

void KAEP_rw::getBestGlobalTrajectory(const std::vector<kino_rrt_star::Trajectory*>& global_goals, kino_rrt_star::Trajectory*& best_global_trajectory) {
    if (global_goals.size() == 0) {
        best_global_trajectory = nullptr;
        return;
    }

    best_global_trajectory = global_goals[0];

    /*// Cost Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_trajectory->cost > global_goals[i]->cost) {
            best_global_trajectory = global_goals[i];
        }
    }*/

    /*// Gain Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_trajectory->gain < global_goals[i]->gain) {
            best_global_trajectory = global_goals[i];
        }
    }*/

    // Score Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_trajectory->score < global_goals[i]->score) {
            best_global_trajectory = global_goals[i];
        }
    }

    ROS_INFO("[KAEP_rw]: Chosen Goal: [%f, %f, %f]", best_global_trajectory->TrajectoryPoints.back()->point[0], best_global_trajectory->TrajectoryPoints.back()->point[1], best_global_trajectory->TrajectoryPoints.back()->point[2]);
    ROS_INFO("[KAEP_rw]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_trajectory->gain, best_global_trajectory->cost2, best_global_trajectory->score);

    if (best_global_trajectory->gain < 0.2) {
        go_terminate = true;
    }

    visualize_best_trajectory(best_global_trajectory, ns);
}

void KAEP_rw::cacheNode(kino_rrt_star::Trajectory* trajectory) {
    if (!trajectory) {
        return;
    }
    cache_nodes::Node cached_node;
    cached_node.gain = trajectory->gain;
    cached_node.position.x = trajectory->TrajectoryPoints.back()->point[0];
    cached_node.position.y = trajectory->TrajectoryPoints.back()->point[1];
    cached_node.position.z = trajectory->TrajectoryPoints.back()->point[2];
    cached_node.yaw = trajectory->TrajectoryPoints.back()->point[3];
    pub_node.publish(cached_node);
}

void KAEP_rw::captureOffset() {
    initial_offset = pose.head<3>();
    // Ground Height at Arming
    if (have_ground_z_) {
        initial_offset.z() = ground_z_;
    } else {
        initial_offset.z() = 0.0;
        ROS_WARN(
            "[KAEP_rw]: never saw the disarmed to armed edge, using z offset 0. Start the "
            "planner stack before arming to correct the barometric bias.");
    }

    geometry_msgs::Point offset_msg;
    offset_msg.x = initial_offset.x();
    offset_msg.y = initial_offset.y();
    offset_msg.z = initial_offset.z();
    pub_offset.publish(offset_msg);

    ROS_INFO("[KAEP_rw]: Start offset captured: [%.2f, %.2f, %.2f]", initial_offset.x(), initial_offset.y(), initial_offset.z());
}

mavros_msgs::PositionTarget KAEP_rw::makeSetpoint(const Eigen::Vector4d& waypoint) {
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

void KAEP_rw::rotate() {
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

void KAEP_rw::retreat(const Eigen::Vector4d& waypoint) {
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

void KAEP_rw::explorationSweep() {
    // Up, Rotate, Down
    const Eigen::Vector4d start = pose;
    const double ceiling = initial_offset[2] + (double)max_z - uav_radius;
    double z_top = std::min(start[2] + exploration_climb_, ceiling);
    if (z_top <= start[2] + 0.05) {
        ROS_WARN("[KAEP_rw]: Exploration sweep skipped: no headroom (z %.2f, ceiling %.2f).",
                 start[2], ceiling);
        return;
    }
    ROS_INFO("[KAEP_rw]: Exploration sweep: up %.2f -> %.2f m, rotate, down.", start[2], z_top);

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
    ROS_INFO("[KAEP_rw]: Exploration sweep done.");
}

bool KAEP_rw::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KAEP_rw]: " << ss.str());

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

bool KAEP_rw::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KAEP_rw]: " << ss.str());

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

bool KAEP_rw::callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KAEP_rw]: " << ss.str());

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

void KAEP_rw::callbackState(const mavros_msgs::State::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    // First Arming Only
    if (msg->armed && !prev_armed_ && have_pose_ && !have_ground_z_) {
        ground_z_ = pose.z();
        have_ground_z_ = true;
        ROS_INFO("[KAEP_rw]: armed on the ground, latching z = %.2f m as the takeoff reference.",
                 ground_z_);
    }
    prev_armed_ = msg->armed;
}

void KAEP_rw::callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KAEP_rw]: getting LocalPose diagnostics");
    uav_local_pose = msg->pose;

    const geometry_msgs::Quaternion& q = uav_local_pose.orientation;

    // Check for NaNs or zero-length quaternion
    if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
        ROS_ERROR("[KAEP_rw]: Invalid quaternion received (contains NaNs)");
        return;
    }

    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm < 0.1 || norm > 1.1) {
        ROS_WARN_THROTTLE(5, "[KAEP_rw] Invalid quaternion detected. Norm: %.3f. Skipping this pose.", norm);
        return;
    }

    // Reject Wild Poses
    const geometry_msgs::Point& p = uav_local_pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x) > pose_max_distance_ || std::abs(p.y) > pose_max_distance_ ||
        std::abs(p.z) > pose_max_distance_) {
        ROS_WARN_THROTTLE(5, "[%s]: implausible pose [%.3g, %.3g, %.3g], skipping.",
                          "KAEP_rw", p.x, p.y, p.z);
        return;
    }
    if (have_pose_) {
        const double dt = (ros::Time::now() - last_pose_time_).toSec();
        const double jump = std::sqrt(std::pow(p.x - pose[0], 2) + std::pow(p.y - pose[1], 2) +
                                      std::pow(p.z - pose[2], 2));
        if (dt > 1e-3 && jump / dt > pose_max_speed_) {
            ROS_WARN_THROTTLE(5, "[%s]: pose jumped %.2f m in %.3f s, skipping.",
                              "KAEP_rw", jump, dt);
            return;
        }
    }

    double yaw = 0.0;
    try {
        yaw = mrs_lib::getYaw(uav_local_pose);
    } catch (const mrs_lib::AttitudeConverter::InvalidAttitudeException& e) {
        ROS_ERROR_THROTTLE(1.0, "[KAEP_rw]: Exception during getYaw(): %s — skipping this pose.", e.what());
        return;
    }

    pose = {uav_local_pose.position.x, uav_local_pose.position.y, uav_local_pose.position.z, yaw};
    last_pose_time_ = ros::Time::now();
    have_pose_ = true;
}

void KAEP_rw::callbackLocalVelocity(const geometry_msgs::TwistStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KAEP_rw]: getting LocalVelocity diagnostics");
    geometry_msgs::Twist uav_velocity = msg->twist;
    velocity = {uav_velocity.linear.x, uav_velocity.linear.y, uav_velocity.linear.z};
}

void KAEP_rw::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    const bool got_local_pose = have_pose_ && (ros::Time::now() - last_pose_time_).toSec() < 2.0;

    if (!got_local_pose) {
        ROS_INFO_THROTTLE(1.0, "[KAEP_rw]: waiting for data: LocalPose = FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[KAEP_rw]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[KAEP_rw]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[KAEP_rw]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            ROS_INFO("[KAEP_rw]: waiting for command");
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

            visualize_frustum(next_best_trajectory->TrajectoryPoints.back().get());
            visualize_unknown_voxels(next_best_trajectory->TrajectoryPoints.back().get());

            std::vector<mavros_msgs::PositionTarget> setpoint_targets;
            std::vector<Eigen::Vector4d> segment_ends;
            const kino_rrt_star::Trajectory* target_trajectory = next_best_trajectory;

            while (next_best_trajectory && next_best_trajectory->parent) {
                segment_ends.push_back(next_best_trajectory->TrajectoryPoints.back()->point);
                for (int i = next_best_trajectory->TrajectoryPoints.size() - 1; i >= 0; i--) {
                    mavros_msgs::PositionTarget setpoint_reference;

                    setpoint_reference.header.frame_id = frame_id;
                    setpoint_reference.header.stamp = ros::Time::now();
                    setpoint_reference.coordinate_frame = 1;
                    setpoint_reference.type_mask = 2496;

                    setpoint_reference.position.x = next_best_trajectory->TrajectoryPoints[i]->point[0];
                    setpoint_reference.position.y = next_best_trajectory->TrajectoryPoints[i]->point[1];
                    setpoint_reference.position.z = next_best_trajectory->TrajectoryPoints[i]->point[2];
                    setpoint_reference.velocity.x = next_best_trajectory->TrajectoryPoints[i]->velocity[0];
                    setpoint_reference.velocity.y = next_best_trajectory->TrajectoryPoints[i]->velocity[1];
                    setpoint_reference.velocity.z = next_best_trajectory->TrajectoryPoints[i]->velocity[2];
                    setpoint_reference.yaw = next_best_trajectory->TrajectoryPoints[i]->point[3];

                    setpoint_targets.push_back(setpoint_reference);
                }
                next_best_trajectory = next_best_trajectory->parent;
            }

            std::reverse(setpoint_targets.begin(), setpoint_targets.end());

            // Store Flown Path
            if (executed_path_.empty() && next_best_trajectory) {
                executed_path_.push_back(next_best_trajectory->TrajectoryPoints.back()->point);
            }
            executed_path_.insert(executed_path_.end(), segment_ends.rbegin(), segment_ends.rend());

            // Trim Flown Part of Branch
            for (size_t k = 1; k < best_branch.size(); ++k) {
                if (best_branch[k].get() == target_trajectory) {
                    best_branch.erase(best_branch.begin(), best_branch.begin() + (k - 1));
                    best_branch.front()->parent = nullptr;
                    break;
                }
            }

            for (size_t i = 0; i < setpoint_targets.size(); i++) {
                if (i >= setpoint_targets.size() - 2) {
                    break;
                }
                pub_setpoint.publish(setpoint_targets[i]);
                ros::Duration(0.1).sleep();
            }

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            ROS_INFO("[KAEP_rw]: waiting for command");
            changeState(STATE_PLANNING);
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[KAEP_rw]: Total Iterations: %d", iteration_);
            ROS_INFO("[KAEP_rw]: Closing output file.");
            ROS_INFO("[KAEP_rw]: Shutting down.");
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

void KAEP_rw::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[KAEP_rw]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[KAEP_rw]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void KAEP_rw::visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns) {
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

void KAEP_rw::visualize_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
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

void KAEP_rw::visualize_best_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
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

void KAEP_rw::visualize_frustum(kino_rrt_star::Node* position) {
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

void KAEP_rw::visualize_unknown_voxels(kino_rrt_star::Node* position) {
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

void KAEP_rw::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void KAEP_rw::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void KAEP_rw::clearMarkers() {
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
