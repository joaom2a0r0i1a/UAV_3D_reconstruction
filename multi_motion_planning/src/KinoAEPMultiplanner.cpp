#include "multidrone_motion_planning/KinoAEPMultiplanner.h"

KinoAEPMultiPlanner::KinoAEPMultiPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {

    //ns = "uav1";

    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "KinoAEPMultiPlanner");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);
    param_loader.loadParam("uav_id", uav_id);

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
    param_loader.loadParam("path/uavs_min_distance", uavs_min_distance);
    param_loader.loadParam("path/lambda", lambda);
    param_loader.loadParam("path/lambda2", lambda2);
    param_loader.loadParam("path/global_lambda", global_lambda);
    param_loader.loadParam("path/global_lambda2", global_lambda2);
    param_loader.loadParam("path/max_acceleration_iterations", max_accel_iterations);

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
    transformer_ = std::make_unique<mrs_lib::Transformer>("KinoAEPMultiPlanner");
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
    shopts.node_name          = "KinoAEPMultiPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &KinoAEPMultiPlanner::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &KinoAEPMultiPlanner::callbackControlManagerDiag, this);
    sub_evade = mrs_lib::SubscribeHandler<multiagent_collision_check::Segment>(shopts, "evasion_segment_in", &KinoAEPMultiPlanner::callbackEvade, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &KinoAEPMultiPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &KinoAEPMultiPlanner::callbackStop, this);

    /* Service Clients */
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &KinoAEPMultiPlanner::timerMain, this);

    is_initialized = true;
}

double KinoAEPMultiPlanner::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool KinoAEPMultiPlanner::isTrajectoryCollisionFree(const std::shared_ptr<kino_rrt_star::Trajectory>& trajectory) const {
    int size = trajectory->TrajectoryPoints.size();
    int half_size = std::floor(size/2);
    std::vector<std::shared_ptr<kino_rrt_star::Node>>::iterator start = trajectory->TrajectoryPoints.begin() + half_size;
    std::vector<std::shared_ptr<kino_rrt_star::Node>>::iterator end = trajectory->TrajectoryPoints.end();
    std::vector<std::shared_ptr<kino_rrt_star::Node>> sliced_nodes(start, end);

    for (const std::shared_ptr<kino_rrt_star::Node>& node : sliced_nodes) {
        if (getMapDistance(node->point.head(3)) < uav_radius) {
            return false;
        }
    }

    return true;
}

void KinoAEPMultiPlanner::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[KinoAEPMultiPlanner]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void KinoAEPMultiPlanner::KinoAEP() {
    localPlanner();
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_trajectory = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[KinoAEPMultiPlanner]: Getting Global Frontiers");
        while (GlobalFrontiers.size() == 0 && g_zero != 0.0) {
            getGlobalFrontiers(GlobalFrontiers);
            if (GlobalFrontiers.size() == 0) {
                g_zero = g_zero / 2;
                // Ignore any gain smaller than 0.1
                if (g_zero < 0.5) {
                    g_zero = 0.0;
                }
                ROS_INFO("[KinoAEPMultiPlanner]: Changed g_zero to %f", g_zero);
            }
        }
        if (GlobalFrontiers.size() == 0) {
            changeState(STATE_STOPPED);
            return;   
        }
        ROS_INFO("[KinoAEPMultiPlanner]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_trajectory);
        if (!backtrack) {
            next_best_trajectory = best_global_trajectory;
        } else {
            backtrack = false;
            return;
        }
        goto_global_planning = false;
    }
}

void KinoAEPMultiPlanner::localPlanner() {
    best_score_ = 0;
    std::shared_ptr<kino_rrt_star::Trajectory> best_trajectory = nullptr;

    // Multi-UAV remove previous planned agent path
    int k;
    for (k = 0; k < agentsId_.size(); k++) {
        if (agentsId_[k] == uav_id) {
            break;
        }
    }
    if (k < agentsId_.size()) {
        segments_[k]->clear();
        segments_[k]->push_back(Eigen::Vector4d(pose[0], pose[1], pose[2], pose[3]));
    }

    int uav_index;
    previous_trajectory_points.clear();
    eth_mav_msgs::EigenTrajectoryPoint previous_trajectory_point;
    for(uav_index = 0; uav_index < agentsId_.size(); uav_index++) {
        for (size_t path_index = 0; path_index < (*segments_[uav_index]).size(); ++path_index) {
            const Eigen::Vector4d& vec = (*segments_[uav_index])[path_index];
            previous_trajectory_point.position_W = vec.head(3);
            previous_trajectory_point.setFromYaw(vec[3]);
            previous_trajectory_points.push_back(previous_trajectory_point);
        }
    }
    
    std::shared_ptr<kino_rrt_star::Node> root_node;
    std::shared_ptr<kino_rrt_star::Trajectory> Root;

    if (current_waypoint_) {
        if (!reset_velocity) {
            root_node = std::make_shared<kino_rrt_star::Node>(next_start, velocity, Eigen::Vector3d::Zero());
        } else {
            root_node = std::make_shared<kino_rrt_star::Node>(next_start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
        Root = std::make_shared<kino_rrt_star::Trajectory>(root_node);
    } else if (best_branch.size() > 1) {
        if (!reset_velocity) {
            root_node = std::make_shared<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, best_branch[1]->TrajectoryPoints.back()->velocity, best_branch[1]->TrajectoryPoints.back()->acceleration);
        } else {
            root_node = std::make_shared<kino_rrt_star::Node>(best_branch[1]->TrajectoryPoints.back()->point, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
        Root = std::make_shared<kino_rrt_star::Trajectory>(root_node);
    } else {
        if (!reset_velocity) {
            root_node = std::make_shared<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
        } else {
            root_node = std::make_shared<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
        Root = std::make_shared<kino_rrt_star::Trajectory>(root_node);
    }
    
    Root->cost = 0.0;
    Root->score = 0.0;

    if (Root->score > best_score_) {
        best_score_ = Root->score;
        best_trajectory = Root;
    }

    KinoRRTStar.clearKDTree();
    KinoRRTStar.addKDTreeTrajectory(Root);
    clearMarkers();
    visualize_node(Root->TrajectoryPoints.back()->point, 2*node_size, ns);

    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    int expanded_num_nodes = 0;
    /*if (best_branch.size() > 0) {
        previous_trajectory = best_branch[0];
    }*/
    while (j < N_max || best_score_ <= g_zero) {
        /*// Backtrack
        if (collision_id_counter_ > 10000 * j) {
            if (previous_trajectory) {
                //next_best_trajectory = previous_trajectory;
                rotate();
                changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: Trying the existing Nodes");
                collision_id_counter_ = 0;
                break;
            }
            return;
        }*/

        if (collision_id_counter_ > 10000 * (j+1)) {
            if (previous_trajectory) {
                ROS_INFO("[KinoAEPMultiPlanner]: Backtracking to [%f, %f, %f]", previous_trajectory->TrajectoryPoints.back()->point[0], previous_trajectory->TrajectoryPoints.back()->point[1], previous_trajectory->TrajectoryPoints.back()->point[2]);
                next_best_trajectory = previous_trajectory;
                reset_velocity = true;
                return;
                //rotate();
                //changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: Trying the existing Nodes");
                rotate();
                collision_id_counter_ = 0;
            }
        }

        // Add previous best branch 
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue; // Skip first iteration (root)
            }
            
            const Eigen::Vector4d& node_position = best_branch[i]->TrajectoryPoints.back()->point;

            std::shared_ptr<kino_rrt_star::Trajectory> nearest_trajectory_best;
            KinoRRTStar.findNearestKD(node_position.head(3), nearest_trajectory_best);
            
            std::shared_ptr<kino_rrt_star::Trajectory> new_trajectory_best;
            new_trajectory_best = best_branch[i];
            new_trajectory_best->parent = nearest_trajectory_best;
            visualize_node(new_trajectory_best->TrajectoryPoints.back()->point, node_size, ns);

            trajectory_point.position_W = new_trajectory_best->TrajectoryPoints.back()->point.head(3);
            trajectory_point.setFromYaw(new_trajectory_best->TrajectoryPoints.back()->point[3]);

            if (multiagent::isInCollision(new_trajectory_best->parent->TrajectoryPoints.back()->point, new_trajectory_best->TrajectoryPoints.back()->point, uavs_min_distance, segments_)) {
                break;
            }

            //std::pair<double, double> result_best = segment_evaluator.computeGainOptimizedwithMultiplePriors(previous_trajectory_points, trajectory_point);
            std::pair<double, double> result_best = segment_evaluator.computeGainOptimizedAEP(trajectory_point);
            new_trajectory_best->gain = result_best.first;
            
            if (result_best.second > M_PI) {
                result_best.second -= 2*M_PI;
            }

            new_trajectory_best->TrajectoryPoints.back()->point[3] = result_best.second;
            KinoRRTStar.steer_trajectory_angular(nearest_trajectory_best, result_best.second, max_heading_velocity, max_heading_accel, new_trajectory_best);
            
            // Make sure the heading of the last node is correct
            new_trajectory_best->TrajectoryPoints.back()->point[3] = result_best.second;

            segment_evaluator.computeCostTwo(new_trajectory_best);
            segment_evaluator.computeScore(new_trajectory_best, lambda, lambda2);

            if (new_trajectory_best->score > best_score_) {
                best_score_ = new_trajectory_best->score;
                best_trajectory = new_trajectory_best;
            }

            ROS_INFO("[KinoAEPMultiPlanner]: Best Score BB: %f", new_trajectory_best->score);

            KinoRRTStar.addKDTreeTrajectory(new_trajectory_best);
            visualize_trajectory(new_trajectory_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > g_zero) {
            break;
        }
    
        best_branch.clear();

        Eigen::Vector3d rand_point;
        KinoRRTStar.computeSamplingDimensions(bounded_radius, rand_point);
        rand_point += root_node->point.head(3);

        std::shared_ptr<kino_rrt_star::Trajectory> nearest_trajectory;
        KinoRRTStar.findNearestKD(rand_point, nearest_trajectory);

        int accel_iteration = 0;
        int accel_tries = 0;
        while (accel_iteration < max_accel_iterations && accel_tries < 100 * max_accel_iterations) {
            accel_tries++;
            Eigen::Vector3d accel;
            KinoRRTStar.computeAccelerationSampling(max_accel, accel);
            std::shared_ptr<kino_rrt_star::Trajectory> new_trajectory;
            new_trajectory = std::make_shared<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory_linear(nearest_trajectory, max_velocity, reset_velocity, accel, step_size, new_trajectory);

           if (new_trajectory->TrajectoryPoints.back()->point[0] > max_x || new_trajectory->TrajectoryPoints.back()->point[0] < min_x 
                || new_trajectory->TrajectoryPoints.back()->point[1] < min_y || new_trajectory->TrajectoryPoints.back()->point[1] > max_y 
                || new_trajectory->TrajectoryPoints.back()->point[2] < min_z || new_trajectory->TrajectoryPoints.back()->point[2] > max_z) {
                new_trajectory.reset();
                collision_id_counter_++;
                break;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(new_trajectory)) {
                collision_id_counter_++;
               // Avoid Memory Leak
                new_trajectory.reset();
                continue;
            }

            bool in_collision = false;
            for (int k = 1; k < new_trajectory->TrajectoryPoints.size(); k++) {
                if (multiagent::isInCollision(new_trajectory->TrajectoryPoints[k-1]->point, new_trajectory->TrajectoryPoints[k]->point, uavs_min_distance, segments_)) {
                    collision_id_counter_++;
                    /*if (collision_id_counter_ > 10000 * j) {
                        break;
                    }*/
                // Avoid Memory Leak
                    new_trajectory.reset();
                    in_collision = true;
                    break;
                }
            }

            if (in_collision) {
                continue;
            }

            visualize_node(new_trajectory->TrajectoryPoints.back()->point, node_size, ns);
            ++accel_iteration;

            trajectory_point.position_W = new_trajectory->TrajectoryPoints.back()->point.head(3);
            trajectory_point.setFromYaw(new_trajectory->TrajectoryPoints.back()->point[3]);

            //std::pair<double, double> result = segment_evaluator.computeGainOptimizedwithMultiplePriors(previous_trajectory_points, trajectory_point);
            std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point);
            new_trajectory->gain = result.first;
            
            // Convert from [0, 2*PI[ to [-PI, PI[ 
            if (result.second > M_PI) {
                result.second -= 2*M_PI;
            }

            new_trajectory->TrajectoryPoints.back()->point[3] = result.second;
            KinoRRTStar.steer_trajectory_angular(nearest_trajectory, result.second, max_heading_velocity, max_heading_accel, new_trajectory);

            // Make sure the heading of the last node is correct
            new_trajectory->TrajectoryPoints.back()->point[3] = result.second;

            segment_evaluator.computeCostTwo(new_trajectory);
            segment_evaluator.computeScore(new_trajectory, lambda, lambda2);

            if (new_trajectory->score > best_score_) {
                best_score_ = new_trajectory->score;
                best_trajectory = new_trajectory;
            }

            ROS_INFO("[KinoAEPMultiPlanner]: Best Score: %f", new_trajectory->score);

            if (new_trajectory->gain >= 0.5) {
                cacheNode(new_trajectory);
            }

            KinoRRTStar.addKDTreeTrajectory(new_trajectory);
            visualize_trajectory(new_trajectory, ns);
        
        }

        if (accel_iteration == 0) {
            continue;
        }

        expanded_num_nodes += accel_iteration;

        if (j > N_termination) {
            ROS_INFO("[KinoAEPMultiPlanner]: Going to Global Planning");
            KinoRRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            best_trajectory.reset();
            Root.reset();
            goto_global_planning = true;
            return;
        }

        ++j;

    }
    
    ROS_INFO("[KinoAEPMultiPlanner]: Final Best Score: %f", best_score_);
    ROS_INFO("[KinoAEPMultiPlanner]: Node Iterations: %d", j);
    ROS_INFO("[KinoAEPMultiPlanner]: Full Node Iterations: %d", expanded_num_nodes);
    
    if (best_trajectory) {
        reset_velocity = false;
        next_best_trajectory = best_trajectory;
        KinoRRTStar.backtrackTrajectoryAEP(best_trajectory, best_branch);
        visualize_best_trajectory(best_trajectory, ns);
    }

    for (int k = 1; k < best_branch.size(); ++k) {
        if (best_branch[k]->gain > g_zero) {
            next_best_trajectory = best_branch[k];
            std::vector<std::shared_ptr<kino_rrt_star::Trajectory>>::iterator start = best_branch.begin() + k - 1;
            std::vector<std::shared_ptr<kino_rrt_star::Trajectory>>::iterator end = best_branch.end();
            std::vector<std::shared_ptr<kino_rrt_star::Trajectory>> sliced_branch(start, end);
            best_branch = sliced_branch;
            break;
        }
    }
}

void KinoAEPMultiPlanner::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, std::shared_ptr<kino_rrt_star::Trajectory>& best_global_trajectory) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[KinoAEPMultiPlanner]: Terminate AEP");

        KinoRRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    int uav_index;
    previous_trajectory_points.clear();
    eth_mav_msgs::EigenTrajectoryPoint previous_trajectory_point;
    for(uav_index = 0; uav_index < agentsId_.size(); uav_index++) {
        for (size_t path_index = 0; path_index < (*segments_[uav_index]).size(); ++path_index) {
            const Eigen::Vector4d& vec = (*segments_[uav_index])[path_index];
            previous_trajectory_point.position_W = vec.head(3);
            previous_trajectory_point.setFromYaw(vec[3]);
            previous_trajectory_points.push_back(previous_trajectory_point);
        }
    }

    std::shared_ptr<kino_rrt_star::Node> global_root_node;
    std::shared_ptr<kino_rrt_star::Trajectory> global_root;

    if (current_waypoint_) {
        if (!reset_velocity) {
            global_root_node = std::make_shared<kino_rrt_star::Node>(next_start, velocity, Eigen::Vector3d::Zero());
        } else {
            global_root_node = std::make_shared<kino_rrt_star::Node>(next_start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
    } else {
        if (!reset_velocity) {
            global_root_node = std::make_shared<kino_rrt_star::Node>(pose, velocity, Eigen::Vector3d::Zero());
        } else {
            global_root_node = std::make_shared<kino_rrt_star::Node>(pose, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        }
    }
    
    global_root = std::make_shared<kino_rrt_star::Trajectory>(global_root_node);
    KinoRRTStar.addKDTreeTrajectory(global_root); 

    std::vector<std::shared_ptr<kino_rrt_star::Trajectory>> all_global_goals;

    int m = 0;
    while (m < N_min_nodes || all_global_goals.size() <= 0) {
        if (collision_id_counter_ > 1000000000 * (m+1)) {
            /*if (previous_trajectory) {
                ROS_INFO("[KinoAEPlanner]: Global Backtracking to [%f, %f, %f]", previous_trajectory->TrajectoryPoints.back()->point[0], previous_trajectory->TrajectoryPoints.back()->point[1], previous_trajectory->TrajectoryPoints.back()->point[2]);
                next_best_trajectory = previous_trajectory;
                reset_velocity = true;
                backtrack = true;
                return;
                //rotate();
                //changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[KinoAEPlanner]: Trying the existing Nodes");
                rotate();
                collision_id_counter_ = 0;
            }*/
        }
        Eigen::Vector3d global_rand_point;
        KinoRRTStar.computeSamplingDimensions(bounded_radius, global_rand_point);
        global_rand_point += global_root_node->point.head(3);

        std::shared_ptr<kino_rrt_star::Trajectory> global_nearest_trajectory;
        KinoRRTStar.findNearestKD(global_rand_point, global_nearest_trajectory);

        int accel_iteration = 0;
        int accel_tries = 0;
        while (accel_iteration < global_max_accel_iterations && accel_tries < 100 * global_max_accel_iterations) {
            accel_tries++;
            Eigen::Vector3d accel;
            KinoRRTStar.computeAccelerationSampling(max_accel, accel);

            std::shared_ptr<kino_rrt_star::Trajectory> global_new_trajectory;
            global_new_trajectory = std::make_shared<kino_rrt_star::Trajectory>();
            KinoRRTStar.steer_trajectory_linear(global_nearest_trajectory, max_velocity, reset_velocity, accel, step_size, global_new_trajectory);

           if (global_new_trajectory->TrajectoryPoints.back()->point[0] > max_x || global_new_trajectory->TrajectoryPoints.back()->point[0] < min_x 
                || global_new_trajectory->TrajectoryPoints.back()->point[1] < min_y || global_new_trajectory->TrajectoryPoints.back()->point[1] > max_y 
                || global_new_trajectory->TrajectoryPoints.back()->point[2] < min_z || global_new_trajectory->TrajectoryPoints.back()->point[2] > max_z) {
                global_new_trajectory.reset();
                //collision_id_counter_++;
                break;
            }

            // Collision Check
            if (!isTrajectoryCollisionFree(global_new_trajectory) || multiagent::isInCollision(global_new_trajectory->TrajectoryPoints.front()->point, global_new_trajectory->TrajectoryPoints.back()->point, uavs_min_distance, segments_)) {
                collision_id_counter_++;
               // Avoid Memory Leak
                global_new_trajectory.reset();
                continue;
            }

            visualize_node(global_new_trajectory->TrajectoryPoints.back()->point, node_size, ns);
            ++accel_iteration;

            segment_evaluator.computeCostTwo(global_new_trajectory);

            KinoRRTStar.addKDTreeTrajectory(global_new_trajectory);
            visualize_trajectory(global_new_trajectory, ns);

            bool goal_reached;
            goal_reached = getGlobalGoal(GlobalFrontiers, global_new_trajectory);
            if (goal_reached) {
                segment_evaluator.computeSingleScore(global_new_trajectory, global_lambda, global_lambda2);
                all_global_goals.push_back(global_new_trajectory);
                goal_reached = false;
            }
        
        }

        if (accel_iteration == 0) {
            continue;
        }

        ++m;
    }

    ROS_INFO("[KinoAEPMultiPlanner]: Global Planner Ends");

    getBestGlobalTrajectory(all_global_goals, best_global_trajectory);
    all_global_goals.clear();
}

void KinoAEPMultiPlanner::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
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

bool KinoAEPMultiPlanner::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, std::shared_ptr<kino_rrt_star::Trajectory>& trajectory) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    for (size_t i = 1; i < GlobalFrontiers.size(); ++i) {
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
        //ROS_INFO("[KinoAEPMultiPlanner]: Goal: [%f, %f, %f]", nearest_goal[0], nearest_goal[1], nearest_goal[2]);
        //ROS_INFO("[KinoAEPMultiPlanner]: RRT* Goal: [%f, %f, %f]", trajectory->TrajectoryPoints.back()->point[0], trajectory->TrajectoryPoints.back()->point[1], trajectory->TrajectoryPoints.back()->point[2]);

        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_global;
        trajectory_point_global.position_W = trajectory->TrajectoryPoints.back()->point.head(3);
        trajectory_point_global.setFromYaw(trajectory->TrajectoryPoints.back()->point[3]);

        //std::pair<double, double> result = segment_evaluator.computeGainOptimizedwithMultiplePriors(previous_trajectory_points, trajectory_point_global);
        std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point_global);
        trajectory->gain = result.first;

        // Convert from [0, 2*PI[ to [-PI, PI[ 
        if (result.second > M_PI) {
            result.second -= 2*M_PI;
        }

        trajectory->TrajectoryPoints.back()->point[3] = result.second;
        KinoRRTStar.steer_trajectory_angular(trajectory->parent, result.second, max_heading_velocity, max_heading_accel, trajectory);

        // Make sure the heading of the last node is correct
        trajectory->TrajectoryPoints.back()->point[3] = result.second;

        trajectory_point_global.position_W = nearest_goal;
        trajectory_point_global.setFromYaw(0.0);
        std::pair<double, double> result_original = segment_evaluator.computeGainOptimizedAEP(trajectory_point_global);
        ROS_INFO("[KinoAEPMultiPlanner]: Goal Best Gain: %f", result_original.first);
        goals_tree.clearKDTreePoints();
        return true;
    }

    goals_tree.clearKDTreePoints();
    return false;
}

void KinoAEPMultiPlanner::getBestGlobalTrajectory(const std::vector<std::shared_ptr<kino_rrt_star::Trajectory>>& global_goals, std::shared_ptr<kino_rrt_star::Trajectory>& best_global_trajectory) {
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

    //ROS_INFO("[KinoAEPMultiPlanner]: Chosen Goal: [%f, %f, %f]", best_global_trajectory->TrajectoryPoints.back()->point[0], best_global_trajectory->TrajectoryPoints.back()->point[1], best_global_trajectory->TrajectoryPoints.back()->point[2]);
    //ROS_INFO("[KinoAEPMultiPlanner]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_trajectory->gain, best_global_trajectory->cost2, best_global_trajectory->score);

    visualize_best_trajectory(best_global_trajectory, ns);
}

void KinoAEPMultiPlanner::cacheNode(std::shared_ptr<kino_rrt_star::Trajectory> trajectory) {
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

double KinoAEPMultiPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint->position.x, waypoint->position.y, waypoint->position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

void KinoAEPMultiPlanner::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[KinoAEPMultiPlanner]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 0.5;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();

    ROS_INFO("[KinoAEPMultiPlanner]: Rotating 360 degrees");

    for (double i = 0.0; i <= 2.0; i = i + 0.4) {
        initial_reference.reference.position.x = pose[0];
        initial_reference.reference.position.y = pose[1];
        initial_reference.reference.position.z = pose[2] + 0.5;
        initial_reference.reference.heading = pose[3] + M_PI * i;
        pub_initial_reference.publish(initial_reference);
        // Max yaw rate is 0.5 rad/s so we wait 0.4*M_PI seconds between points
        ros::Duration(0.4*M_PI).sleep();
    }

    ros::Duration(0.5).sleep();

    /*ROS_INFO("[KinoAEPMultiPlanner]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(1).sleep();*/
}

void KinoAEPMultiPlanner::rotate() {
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
        ros::Duration(0.4*M_PI).sleep();
    }
}

bool KinoAEPMultiPlanner::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KinoAEPMultiPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;

}

bool KinoAEPMultiPlanner::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[KinoAEPMultiPlanner]: " << ss.str());

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

void KinoAEPMultiPlanner::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KinoAEPMultiPlanner]: getting ControlManager diagnostics");
    control_manager_diag = *msg;

    // If planner stops, set velocity to zero
    if (!control_manager_diag.tracker_status.have_goal && !reset_velocity) {
        reset_velocity = true;
    }
}

void KinoAEPMultiPlanner::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KinoAEPMultiPlanner]: getting UavState diagnostics");
    geometry_msgs::Pose uav_pose = msg->pose;
    geometry_msgs::Twist uav_velocity = msg->velocity;
    double yaw = mrs_lib::getYaw(uav_pose);
    pose = {uav_pose.position.x, uav_pose.position.y, uav_pose.position.z, yaw};
    velocity = {uav_velocity.linear.x, uav_velocity.linear.y, uav_velocity.linear.z};
}

void KinoAEPMultiPlanner::callbackEvade(const multiagent_collision_check::Segment::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[KinoAEPMultiPlanner]: getting CollisionCheck diagnostics");

    int i;
    for(i = 0; i < agentsId_.size(); i++) {
        if(agentsId_[i] == msg->uav_id) {
            break;
        }
    }

    // If no match was found, add the uav_id to the list of UAV IDs
    if (i == agentsId_.size()) {
        agentsId_.push_back(msg->uav_id);
        segments_.push_back(new std::vector<Eigen::Vector4d>);
    }

    // Update the segment list with the poses from msg
    segments_[i]->clear();
    /*for(std::vector<geometry_msgs::Point>::const_iterator it = msg->uav_path.begin(); it != msg->uav_path.end(); ++it) {
        segments_[i]->push_back(Eigen::Vector3d(it->x, it->y, it->z));
    }*/
   
    for (size_t j = 0; j < msg->uav_path.size(); ++j) {
        const geometry_msgs::Point& p = msg->uav_path[j];
        double yaw = msg->yaw[j];
        segments_[i]->push_back(Eigen::Vector4d(p.x, p.y, p.z, yaw));
    }
}

void KinoAEPMultiPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[KinoAEPMultiPlanner]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[KinoAEPMultiPlanner]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[KinoAEPMultiPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[KinoAEPMultiPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }
    
    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KinoAEPMultiPlanner]: tracker has goal");
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KinoAEPMultiPlanner]: tracker has goal");
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: waiting for command");
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
            KinoAEP();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            iteration_ += 1;

            if (!current_waypoint_) {
                current_waypoint_ = std::make_unique<mrs_msgs::Reference>();
            }

            current_waypoint_->position.x = next_best_trajectory->TrajectoryPoints.back()->point[0];
            current_waypoint_->position.y = next_best_trajectory->TrajectoryPoints.back()->point[1];
            current_waypoint_->position.z = next_best_trajectory->TrajectoryPoints.back()->point[2];
            current_waypoint_->heading = next_best_trajectory->TrajectoryPoints.back()->point[3];

            next_start[0] = current_waypoint_->position.x;
            next_start[1] = current_waypoint_->position.y;
            next_start[2] = current_waypoint_->position.z;
            next_start[3] = current_waypoint_->heading;

            visualize_frustum(next_best_trajectory->TrajectoryPoints.back());
            visualize_unknown_voxels(next_best_trajectory->TrajectoryPoints.back());

            mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;

            srv_trajectory_reference.request.trajectory.header.frame_id = ns + "/" + frame_id;
            srv_trajectory_reference.request.trajectory.header.stamp = ros::Time::now();
            srv_trajectory_reference.request.trajectory.input_id = iteration_;
            srv_trajectory_reference.request.trajectory.fly_now = true;
            srv_trajectory_reference.request.trajectory.use_heading = true;

            srv_trajectory_reference.request.trajectory.dt = 0.1;

            if (next_best_trajectory) {
                previous_trajectory = std::make_shared<kino_rrt_star::Trajectory>(*next_best_trajectory);
            }
            if (next_best_trajectory->parent) {
                std::shared_ptr<kino_rrt_star::Trajectory> previous_trajectory_parent = std::make_shared<kino_rrt_star::Trajectory>(*next_best_trajectory->parent);
                previous_trajectory->parent = previous_trajectory_parent;
            }

            std::reverse(previous_trajectory->TrajectoryPoints.begin(), previous_trajectory->TrajectoryPoints.end());

            mrs_msgs::Reference reference;

            while (next_best_trajectory && next_best_trajectory->parent) {
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
            
            multiagent_collision_check::Segment segment;
            segment.uav_id = uav_id;
            for (const auto& point : srv_trajectory_reference.request.trajectory.points) {
                pub_reference.publish(point);
                segment.uav_path.push_back(point.position);
                segment.yaw.push_back(point.heading);
            }
            ROS_INFO_STREAM("Publishing to pub_evade with segment: uav_id=" << segment.uav_id
                << " with trajectory points=" << segment.uav_path.size());
            pub_evade.publish(segment);

            bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

            if (!success_trajectory) {
                ROS_ERROR("[KinoAEPMultiPlanner]: service call for trajectory reference failed");
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_trajectory_reference.response.success) {
                    ROS_ERROR("[KinoAEPMultiPlanner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
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
                ROS_INFO("[KinoAEPMultiPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_->heading - current_yaw), cos(current_waypoint_->heading - current_yaw)));
                ROS_INFO("[KinoAEPMultiPlanner]: Distance to waypoint: %.2f", dist);
                if (dist <= 0.6*step_size && yaw_difference <= 0.4*M_PI) {
                    changeState(STATE_PLANNING);
                }
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[KinoAEPMultiPlanner]: Total Iterations: %d", iteration_);
            ROS_INFO("[KinoAEPMultiPlanner]: Shutting down.");
            // Multi-UAV remove final pose so drones don't collide when algorithm is finished
            int k;
            for (k = 0; k < agentsId_.size(); k++) {
                if (agentsId_[k] == uav_id) {
                    break;
                }
            }
            if (k < agentsId_.size()) {
                segments_[k]->clear();
                segments_[k]->push_back(Eigen::Vector4d(pose[0], pose[1], pose[2], pose[3]));
            }
            ros::shutdown();
            return;
        }
        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[KinoAEPMultiPlanner]: tracker has goal");
            } else {
                ROS_INFO("[KinoAEPMultiPlanner]: waiting for command");
            }
            break;
        }
    }
}

void KinoAEPMultiPlanner::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[KinoAEPMultiPlanner]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[KinoAEPMultiPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void KinoAEPMultiPlanner::visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns) {
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

    node_id_counter_++;

    n.lifetime = ros::Duration(30.0);
    n.frame_locked = false;
    pub_markers.publish(n);
}

void KinoAEPMultiPlanner::visualize_trajectory(const std::shared_ptr<kino_rrt_star::Trajectory> trajectory, const std::string& ns) {
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

void KinoAEPMultiPlanner::visualize_best_trajectory(const std::shared_ptr<kino_rrt_star::Trajectory> trajectory, const std::string& ns) {
    std::shared_ptr<kino_rrt_star::Trajectory> currentTrajectory = trajectory;
    
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

void KinoAEPMultiPlanner::visualize_frustum(std::shared_ptr<kino_rrt_star::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->point.head(3);
    trajectory_point_visualize.setFromYaw(position->point[3]);
    
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

void KinoAEPMultiPlanner::visualize_unknown_voxels(std::shared_ptr<kino_rrt_star::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->point.head(3);
    trajectory_point_visualize.setFromYaw(position->point[3]);

    voxblox::Pointcloud voxel_points;
    segment_evaluator.visualizeGainAEP(trajectory_point_visualize, voxel_points);
    
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

void KinoAEPMultiPlanner::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void KinoAEPMultiPlanner::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void KinoAEPMultiPlanner::clearMarkers() {
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
