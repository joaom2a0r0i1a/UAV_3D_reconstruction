#include "multi_motion_planning/KAEP_fleet.h"

KAEP_fleet::KAEP_fleet(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "KAEP_fleet");

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
    node_size = 0.2;

    // Get vertical FoV and setup camera
    vertical_fov = segment_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    segment_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    segment_evaluator.setEsdfMap(esdf_map_);

    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("KAEP_fleet");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
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
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 3);
    pub_reference = nh_private_.advertise<mrs_msgs::Reference>("reference_out", 3);
    pub_node = nh_private_.advertise<cache_nodes::Node>("tree_node_out", 500);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 30);
    pub_initial_reference = nh_private_.advertise<mrs_msgs::ReferenceStamped>("initial_reference_out", 15);
    pub_evade = nh_private_.advertise<multiagent_collision_check::Segment>("evasion_segment_out", 100);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "KAEP_fleet";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &KAEP_fleet::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &KAEP_fleet::callbackControlManagerDiag, this);
    // Paths of the other UAVs on their own queue, read right after each plan
    nh_evade_ = nh_private_;
    nh_evade_.setCallbackQueue(&evade_queue_);
    mrs_lib::SubscribeHandlerOptions shopts_evade = shopts;
    shopts_evade.nh = nh_evade_;
    sub_evade = mrs_lib::SubscribeHandler<multiagent_collision_check::Segment>(shopts_evade, "evasion_segment_in", &KAEP_fleet::callbackEvade, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &KAEP_fleet::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &KAEP_fleet::callbackStop, this);

    /* Service Clients */
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &KAEP_fleet::timerMain, this);

    is_initialized = true;
}

double KAEP_fleet::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool KAEP_fleet::isTrajectoryCollisionFree(kino_rrt_star::Trajectory* trajectory) const {
    int size = trajectory->TrajectoryPoints.size();
    int half_size = std::floor(size / 2);
    for (int i = half_size; i < size; ++i) {
        if (getMapDistance(trajectory->TrajectoryPoints[i]->point.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

void KAEP_fleet::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[KAEP_fleet]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void KAEP_fleet::planStep() {
    goto_global_planning = false;
    next_best_trajectory = nullptr;

    localPlanner();
    if (retreating_) {
        return;
    }
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_trajectory = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[KAEP_fleet]: Getting Global Frontiers");
        while (GlobalFrontiers.size() == 0 && g_zero != 0.0) {
            getGlobalFrontiers(GlobalFrontiers);
            if (GlobalFrontiers.size() == 0) {
                g_zero = g_zero / 2;
                // Ignore any gain smaller than 0.1
                if (g_zero < 0.5) {
                    g_zero = 0.0;
                }
                ROS_INFO("[KAEP_fleet]: Changed g_zero to %f", g_zero);
            }
        }
        if (GlobalFrontiers.size() == 0) {
            changeState(STATE_STOPPED);
            return;
        }
        ROS_INFO("[KAEP_fleet]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_trajectory);
        goto_global_planning = false;
        if (retreating_) {
            return;
        }
        next_best_trajectory = best_global_trajectory;
    }
}

void KAEP_fleet::localPlanner() {
    best_score_ = 0;
    kino_rrt_star::Trajectory* best_trajectory = nullptr;

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

    std::unique_ptr<kino_rrt_star::Node> root_node_owned;
    if (best_branch.size() > 1) {
        if (!reset_velocity) {
            root_node_owned = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, best_branch[1]->TrajectoryPoints.back()->velocity, best_branch[1]->TrajectoryPoints.back()->acceleration);
        } else {
            root_node_owned = std::make_unique<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
    } else {
        if (!reset_velocity) {
            root_node_owned = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
        } else {
            root_node_owned = std::make_unique<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
    }

    std::unique_ptr<kino_rrt_star::Trajectory> Root = std::make_unique<kino_rrt_star::Trajectory>(std::move(root_node_owned));
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
    while (j < N_max || best_score_ <= g_zero) {
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
                ROS_WARN("[KAEP_fleet]: Backtracking (%s, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", j, executed_path_.size());
                best_branch.clear();
                return;
            }
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
            std::pair<double, double> result_best = segment_evaluator.computeGainRaycasting(trajectory_point, true);
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

            ROS_INFO("[KAEP_fleet]: Best Score BB: %f", raw_best->score);

            KinoRRTStar.addKDTreeTrajectory(std::move(raw_best_owned));
            visualize_trajectory(raw_best, ns);

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
            std::unique_ptr<kino_rrt_star::Trajectory> new_trajectory = std::make_unique<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory_linear(nearest_trajectory, max_velocity, reset_velocity, accel, step_size, new_trajectory);

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

            trajectory_point = new_trajectory->TrajectoryPoints.back()->point;
            std::pair<double, double> result = segment_evaluator.computeGainRaycasting(trajectory_point, true);
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

            ROS_INFO("[KAEP_fleet]: Best Score: %f", new_trajectory->score);

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
            ROS_INFO("[KAEP_fleet]: Going to Global Planning");
            KinoRRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            goto_global_planning = true;
            return;
        }

        ++j;
    }

    ROS_INFO("[KAEP_fleet]: Final Best Score: %f", best_score_);
    ROS_INFO("[KAEP_fleet]: Node Iterations: %d", j);
    ROS_INFO("[KAEP_fleet]: Full Node Iterations: %d", expanded_num_nodes);

    if (best_trajectory) {
        reset_velocity = false;
        next_best_trajectory = best_trajectory;
        KinoRRTStar.backtrackTrajectoryAEP(best_trajectory, best_branch);
        visualize_best_trajectory(best_trajectory, ns);
    }

    // First Informative Trajectory, flown part trimmed after the trajectory walk
    for (size_t ki = 1; ki < best_branch.size(); ++ki) {
        if (best_branch[ki]->gain > g_zero) {
            next_best_trajectory = best_branch[ki].get();
            break;
        }
    }
}

void KAEP_fleet::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, kino_rrt_star::Trajectory*& best_global_trajectory) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[KAEP_fleet]: Terminate AEP");

        KinoRRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    std::unique_ptr<kino_rrt_star::Node> global_root_node_owned;
    if (!reset_velocity) {
        global_root_node_owned = std::make_unique<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
    } else {
        global_root_node_owned = std::make_unique<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    Eigen::Vector4d global_root_point = global_root_node_owned->point;
    auto global_root = std::make_unique<kino_rrt_star::Trajectory>(std::move(global_root_node_owned));
    kino_rrt_star::Trajectory* global_root_ptr = KinoRRTStar.addKDTreeTrajectory(std::move(global_root));
    (void)global_root_ptr;

    std::vector<kino_rrt_star::Trajectory*> all_global_goals;

    int m = 0;
    collision_id_counter_ = 0;
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
                ROS_WARN("[KAEP_fleet]: Global backtracking (%s, tree=%d) -> executed node %zu",
                         g_boxed ? "boxed-in" : "timeout", m, executed_path_.size());
                return;
            }
            ROS_INFO("[KAEP_fleet]: Backtrack Rotation");
            rotate();
            gplan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        Eigen::Vector3d global_rand_point;
        KinoRRTStar.computeSamplingDimensions(bounded_radius, global_rand_point);
        global_rand_point += global_root_point.head(3);

        kino_rrt_star::Trajectory* global_nearest_trajectory = nullptr;
        KinoRRTStar.findNearestKD(global_rand_point, global_nearest_trajectory);

        int accel_iteration = 0;
        int accel_tries = 0;
        while (accel_iteration < global_max_accel_iterations && accel_tries < 100 * global_max_accel_iterations) {
            accel_tries++;
            Eigen::Vector3d accel;
            KinoRRTStar.computeAccelerationSampling(max_accel, accel);

            std::unique_ptr<kino_rrt_star::Trajectory> global_new_trajectory = std::make_unique<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory_linear(global_nearest_trajectory, max_velocity, reset_velocity, accel, step_size, global_new_trajectory);

            if (global_new_trajectory->TrajectoryPoints.back()->point[0] > max_x || global_new_trajectory->TrajectoryPoints.back()->point[0] < min_x || global_new_trajectory->TrajectoryPoints.back()->point[1] < min_y || global_new_trajectory->TrajectoryPoints.back()->point[1] > max_y || global_new_trajectory->TrajectoryPoints.back()->point[2] < min_z || global_new_trajectory->TrajectoryPoints.back()->point[2] > max_z) {
                break;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(global_new_trajectory.get()) || multiagent::isInCollision(global_new_trajectory->TrajectoryPoints.front()->point, global_new_trajectory->TrajectoryPoints.back()->point, uav_radius, segments_)) {
                collision_id_counter_++;
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
            }
        }

        if (accel_iteration == 0) {
            continue;
        }

        ++m;
    }

    ROS_INFO("[KAEP_fleet]: Global Planner Ends");

    getBestGlobalTrajectory(all_global_goals, best_global_trajectory);
    all_global_goals.clear();
}

void KAEP_fleet::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
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

bool KAEP_fleet::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, kino_rrt_star::Trajectory* trajectory) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    if (GlobalFrontiers.empty()) {
        return false;
    }
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
        //ROS_INFO("[KAEP_fleet]: Goal: [%f, %f, %f]", nearest_goal[0], nearest_goal[1], nearest_goal[2]);
        //ROS_INFO("[KAEP_fleet]: RRT* Goal: [%f, %f, %f]", trajectory->TrajectoryPoints.back()->point[0], trajectory->TrajectoryPoints.back()->point[1], trajectory->TrajectoryPoints.back()->point[2]);

        Eigen::Vector4d trajectory_point_global = trajectory->TrajectoryPoints.back()->point;
        std::pair<double, double> result = segment_evaluator.computeGainRaycasting(trajectory_point_global, true);
        trajectory->gain = result.first;

        // Convert from [0, 2*PI[ to [-PI, PI[
        if (result.second > M_PI) {
            result.second -= 2 * M_PI;
        }

        trajectory->TrajectoryPoints.back()->point[3] = result.second;
        KinoRRTStar.steer_trajectory_angular(trajectory->parent, result.second, max_heading_velocity, max_heading_accel, trajectory);

        // Make sure the heading of the last node is correct
        trajectory->TrajectoryPoints.back()->point[3] = result.second;

        if (trajectory->gain < 0.1) {
            goals_tree.clearKDTreePoints();
            return false;
        }

        trajectory_point_global.head<3>() = nearest_goal;
        trajectory_point_global[3] = 0.0;
        std::pair<double, double> result_original = segment_evaluator.computeGainRaycasting(trajectory_point_global, true);
        ROS_INFO("[KAEP_fleet]: Goal Best Gain: %f", result_original.first);
        goals_tree.clearKDTreePoints();
        return true;
    }

    goals_tree.clearKDTreePoints();
    return false;
}

void KAEP_fleet::getBestGlobalTrajectory(const std::vector<kino_rrt_star::Trajectory*>& global_goals, kino_rrt_star::Trajectory*& best_global_trajectory) {
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

    //ROS_INFO("[KAEP_fleet]: Chosen Goal: [%f, %f, %f]", best_global_trajectory->TrajectoryPoints.back()->point[0], best_global_trajectory->TrajectoryPoints.back()->point[1], best_global_trajectory->TrajectoryPoints.back()->point[2]);
    //ROS_INFO("[KAEP_fleet]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_trajectory->gain, best_global_trajectory->cost2, best_global_trajectory->score);

    visualize_best_trajectory(best_global_trajectory, ns);
}

void KAEP_fleet::cacheNode(kino_rrt_star::Trajectory* trajectory) {
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

double KAEP_fleet::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {
    return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                   vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

void KAEP_fleet::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[KAEP_fleet]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();

    ROS_INFO("[KAEP_fleet]: Rotating 360 degrees");

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

    ROS_INFO("[KAEP_fleet]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();
}

void KAEP_fleet::rotate() {
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

bool KAEP_fleet::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KAEP_fleet]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;
}

bool KAEP_fleet::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KAEP_fleet]: " << ss.str());

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

void KAEP_fleet::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KAEP_fleet]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void KAEP_fleet::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KAEP_fleet]: getting UavState diagnostics");
    geometry_msgs::Pose uav_pose = msg->pose;
    geometry_msgs::Twist uav_velocity = msg->velocity;
    double yaw = mrs_lib::getYaw(uav_pose);
    pose = {uav_pose.position.x, uav_pose.position.y, uav_pose.position.z, yaw};
    velocity = {uav_velocity.linear.x, uav_velocity.linear.y, uav_velocity.linear.z};
}

void KAEP_fleet::callbackEvade(const multiagent_collision_check::Segment::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KAEP_fleet]: getting CollisionCheck diagnostics");

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
std::vector<std::vector<Eigen::Vector3d>*> KAEP_fleet::otherSegments() const {
    std::vector<std::vector<Eigen::Vector3d>*> others;
    for (size_t i = 0; i < agentsId_.size(); ++i) {
        if (agentsId_[i] != uav_id) {
            others.push_back(segments_[i]);
        }
    }
    return others;
}

bool KAEP_fleet::isPathClearOfOthers(const std::vector<Eigen::Vector3d>& path) const {
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

void KAEP_fleet::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[KAEP_fleet]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[KAEP_fleet]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[KAEP_fleet]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[KAEP_fleet]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KAEP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KAEP_fleet]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KAEP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KAEP_fleet]: waiting for command");
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
                    ROS_WARN("[KAEP_fleet]: Retreat blocked by another UAV, rotating instead");
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
                geometry_msgs::Point from, to;
                from.x = pose[0];
                from.y = pose[1];
                from.z = pose[2];
                to = current_waypoint_.position;
                retreat_segment.uav_path.push_back(from);
                retreat_segment.uav_path.push_back(to);
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
                ROS_WARN("[KAEP_fleet]: No trajectory chosen, planning again");
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
            std::vector<Eigen::Vector4d> segment_ends;
            const kino_rrt_star::Trajectory* target_trajectory = next_best_trajectory;

            while (next_best_trajectory && next_best_trajectory->parent) {
                segment_ends.push_back(next_best_trajectory->TrajectoryPoints.back()->point);
                for (int i = next_best_trajectory->TrajectoryPoints.size() - 1; i >= 0; i--) {
                    reference.position.x = next_best_trajectory->TrajectoryPoints[i]->point[0];
                    reference.position.y = next_best_trajectory->TrajectoryPoints[i]->point[1];
                    reference.position.z = next_best_trajectory->TrajectoryPoints[i]->point[2];
                    reference.heading = next_best_trajectory->TrajectoryPoints[i]->point[3];
                    srv_trajectory_reference.request.trajectory.points.push_back(reference);
                }
                next_best_trajectory = next_best_trajectory->parent;
            }

            std::reverse(srv_trajectory_reference.request.trajectory.points.begin(), srv_trajectory_reference.request.trajectory.points.end());

            // Recheck Against the Latest Paths of the Other UAVs
            std::vector<Eigen::Vector3d> planned_path;
            for (const auto& point : srv_trajectory_reference.request.trajectory.points) {
                planned_path.emplace_back(point.position.x, point.position.y, point.position.z);
            }
            if (!isPathClearOfOthers(planned_path)) {
                ROS_WARN("[KAEP_fleet]: Trajectory crosses the new path of another UAV, planning again");
                best_branch.clear();
                break;
            }

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

            multiagent_collision_check::Segment segment;
            segment.uav_id = uav_id;
            for (const auto& point : srv_trajectory_reference.request.trajectory.points) {
                pub_reference.publish(point);
                segment.uav_path.push_back(point.position);
            }
            ROS_INFO_STREAM("Publishing to pub_evade with segment: uav_id=" << segment.uav_id
                                                                            << " with trajectory points=" << segment.uav_path.size());
            pub_evade.publish(segment);

            bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

            if (!success_trajectory) {
                ROS_ERROR("[KAEP_fleet]: service call for trajectory reference failed");
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_trajectory_reference.response.success) {
                    ROS_ERROR("[KAEP_fleet]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                    changeState(STATE_STOPPED);
                    return;
                }
            }

            ros::Duration(1).sleep();

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KAEP_fleet]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_.heading - current_yaw), cos(current_waypoint_.heading - current_yaw)));
                ROS_INFO("[KAEP_fleet]: Distance to waypoint: %.2f", dist);
                if (dist <= 0.6 * step_size && yaw_difference <= 0.4 * M_PI) {
                    changeState(STATE_PLANNING);
                }
            } else {
                ROS_INFO("[KAEP_fleet]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[KAEP_fleet]: Total Iterations: %d", iteration_);
            ROS_INFO("[KAEP_fleet]: Shutting down.");
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
                ROS_INFO("[KAEP_fleet]: tracker has goal");
            } else {
                ROS_INFO("[KAEP_fleet]: waiting for command");
            }
            break;
        }
    }
}

void KAEP_fleet::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[KAEP_fleet]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[KAEP_fleet]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

// Rotates the colors by 120 degrees of hue per UAV, UAV1 keeps the original colors
void KAEP_fleet::colorForUav(std_msgs::ColorRGBA& color) const {
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

void KAEP_fleet::visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns) {
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

void KAEP_fleet::visualize_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
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

void KAEP_fleet::visualize_best_trajectory(kino_rrt_star::Trajectory* trajectory, const std::string& ns) {
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

void KAEP_fleet::visualize_frustum(kino_rrt_star::Node* position) {
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

void KAEP_fleet::visualize_unknown_voxels(kino_rrt_star::Node* position) {
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

void KAEP_fleet::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void KAEP_fleet::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void KAEP_fleet::clearMarkers() {
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
