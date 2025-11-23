#include "motion_planning/GAEP/GAEPlanner.h"

GAEPlanner::GAEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "GAEPlanner");

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

    // Camera
    param_loader.loadParam("camera/h_fov", horizontal_fov);
    param_loader.loadParam("camera/width", resolution_x);
    param_loader.loadParam("camera/height", resolution_y);
    param_loader.loadParam("camera/min_distance", min_distance);
    param_loader.loadParam("camera/max_distance", max_distance);

    // Planner
    param_loader.loadParam("path/uav_radius", uav_radius);
    param_loader.loadParam("path/lambda", lambda);
    param_loader.loadParam("path/global_lambda", global_lambda);

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
    transformer_ = std::make_unique<mrs_lib::Transformer>("GAEPlanner");
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
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_reference = nh_private_.advertise<mrs_msgs::Reference>("reference_out", 1);
    pub_node = nh_private_.advertise<cache_nodes::Node>("tree_node_out", 500);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_initial_reference = nh_private_.advertise<mrs_msgs::ReferenceStamped>("initial_reference_out", 5);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "GAEPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &GAEPlanner::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &GAEPlanner::callbackControlManagerDiag, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &GAEPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &GAEPlanner::callbackStop, this);

    /* Service Clients */
    sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_private_, "trajectory_generation_out");
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &GAEPlanner::timerMain, this);

    is_initialized = true;
}

double GAEPlanner::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool GAEPlanner::isPathCollisionFree(const std::vector<std::shared_ptr<geo_rrt::Node>>& path) const {
    for (const std::shared_ptr<geo_rrt::Node>& node : path) {
        if (getMapDistance(node->pose.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

bool GAEPlanner::isInsideBoundingBox(const std::shared_ptr<geo_rrt::Node>& Node) const {  
    if (Node->pose[0] > max_x || Node->pose[0] < min_x 
        || Node->pose[1] < min_y || Node->pose[1] > max_y 
        || Node->pose[2] < min_z || Node->pose[2] > max_z) {
        return false;
    }
    return true;
}

void GAEPlanner::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[GAEPlanner]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void GAEPlanner::AEP() {
    localPlanner();
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_node = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[GAEPlanner]: Getting Global Frontiers");
        getGlobalFrontiers(GlobalFrontiers);
        while (GlobalFrontiers.size() == 0 && g_zero != 0.0) {
            getGlobalFrontiers(GlobalFrontiers);
            if (GlobalFrontiers.size() == 0) {
                g_zero = g_zero / 2;
                // Ignore any gain smaller than 0.005
                if (g_zero < 0.005) {
                    g_zero = 0.0;
                }
                ROS_INFO("[KinoAEPlanner]: Changed g_zero to %f", g_zero);
            }
        }
        ROS_INFO("[GAEPlanner]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_node);
        std::shared_ptr<geo_rrt::Node> copy_best_node = std::make_shared<geo_rrt::Node>(*best_global_node);
        if (!backtrack) {
            double best_score_global = 0.0;
            while (copy_best_node && copy_best_node->parent) {
                if (copy_best_node->score > best_score_global) {
                    best_score_global = copy_best_node->score;
                    next_best_node = copy_best_node;
                }
                copy_best_node = copy_best_node->parent;
            }
            //next_best_node = best_global_node;
        } else {
            backtrack = false;
            return;
        }
        goto_global_planning = false;
    }
}

void GAEPlanner::localPlanner() {
    best_score_ = 0;
    std::shared_ptr<geo_rrt::Node> best_node = nullptr;

    ROS_INFO("[GAEPlanner]: Start Expanding Local");

    std::shared_ptr<geo_rrt::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<geo_rrt::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_shared<geo_rrt::Node>(best_branch[1]->pose);
    } else {
        root = std::make_shared<geo_rrt::Node>(pose);
    }

    trajectory_point.position_W = root->pose.head(3);
    trajectory_point.setFromYaw(root->pose[3]);

    auto [root_path_gain, root_max_gain, root_pose_angle] = segment_evaluator.computeGainOptimizedGEO(trajectory_point);
    root->path_gain = root_path_gain;
    root->max_gain = root_max_gain;
    //root->pose[3] = root_pose_angle;

    if (root->max_gain > 0.0) {
        root->score = root->path_gain / root->max_gain * 100.0;
    }

    /*if (root->score > best_score_) {
        best_score_ = root->score;
        best_node = root;
    }*/

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    clearMarkers();

    visualize_node(root->pose, ns);
    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    while (j < N_max || best_score_ <= g_zero) {
        // Backtrack
        if (collision_id_counter_ > 10000 * j) {
            if (previous_node) {
                ROS_INFO("[GAEPlanner]: Backtracking to [%f, %f, %f]", previous_node->pose[0], previous_node->pose[1], previous_node->pose[2]);
                next_best_node = previous_node;
                best_branch.clear();
                return;
                //rotate();
                //changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[GAEPlanner]: Backtrack Rotation");
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
            
            const Eigen::Vector4d& node_position = best_branch[i]->pose;

            std::shared_ptr<geo_rrt::Node> nearest_node_best;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            
            std::shared_ptr<geo_rrt::Node> new_node_best;
            new_node_best = std::make_shared<geo_rrt::Node>(node_position);
            new_node_best->parent = nearest_node_best;
            visualize_node(new_node_best->pose, ns);

            trajectory_point.position_W = new_node_best->pose.head(3);
            trajectory_point.setFromYaw(new_node_best->pose[3]);

            auto parent_node = new_node_best->parent;

            std::vector<eth_mav_msgs::EigenTrajectoryPoint> previous_trajectory;
            while (parent_node->parent) {
                previous_trajectory_point.position_W = parent_node->pose.head(3);
                previous_trajectory_point.setFromYaw(parent_node->pose[3]);
                previous_trajectory.push_back(previous_trajectory_point);
                parent_node = parent_node->parent;
            }

            /*std::pair<double, double> result = segment_evaluator.computeGainOptimizedwithMultiplePriors(previous_trajectory, trajectory_point);
            new_node_best->path_gain = result.first;
            new_node_best->pose[3] = result.second;*/

            auto [path_gain, max_gain, pose_angle] = segment_evaluator.computeGainOptimizedwithMultiplePriorsMax(previous_trajectory, trajectory_point);
            new_node_best->path_gain = new_node_best->parent->path_gain + path_gain;
            new_node_best->max_gain = new_node_best->parent->max_gain + max_gain;
            new_node_best->pose[3] = pose_angle;

            if (new_node_best->max_gain > 0.0) {
                new_node_best->score = new_node_best->path_gain / new_node_best->max_gain * 100.0;
            }

            segment_evaluator.computeCost(new_node_best);
            //segment_evaluator.computeScore(new_node_best, lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best;
            }

            //ROS_INFO("[GAEPlanner]: Best Gain Optimized BB: %f", new_node_best->path_gain);
            //ROS_INFO("[GAEPlanner]: Best Gain BB: %f", result2.first);
            //ROS_INFO("[GAEPlanner]: Best Cost BB: %f", new_node_best->cost);
            ROS_INFO("[GAEPlanner]: Best Score BB: %f", new_node_best->score);
            

            RRTStar.addKDTreeNode(new_node_best);
            visualize_edge(new_node_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > g_zero) {
            break;
        }
    
        best_branch.clear();

        Eigen::Vector3d rand_point;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
        rand_point += root->pose.head(3);

        std::shared_ptr<geo_rrt::Node> nearest_node;
        RRTStar.findNearestKD(rand_point, nearest_node);

        std::shared_ptr<geo_rrt::Node> new_node;
        RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

        // Collision Check
        std::vector<std::shared_ptr<geo_rrt::Node>> trajectory_segment;
        trajectory_segment.push_back(new_node);

        if (!isPathCollisionFree(trajectory_segment)) {
            trajectory_segment.clear();
            collision_id_counter_++;
            continue;
        }

        if (!isInsideBoundingBox(new_node)) {
            collision_id_counter_++;
            continue;
        }

        trajectory_segment.clear();
        visualize_node(new_node->pose, ns);

        trajectory_point.position_W = new_node->pose.head(3);
        trajectory_point.setFromYaw(new_node->pose[3]);

        auto parent_node = new_node->parent;

        std::vector<eth_mav_msgs::EigenTrajectoryPoint> previous_trajectory;
        //int size_previous_trajectory = 1;

        //while (parent_node->parent && size_previous_trajectory <= 5) {
        while (parent_node->parent) {
            previous_trajectory_point.position_W = parent_node->pose.head(3);
            previous_trajectory_point.setFromYaw(parent_node->pose[3]);
            previous_trajectory.push_back(previous_trajectory_point);
            //size_previous_trajectory++;
            parent_node = parent_node->parent;
        }

        //std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point);

        /*std::pair<double, double> result = segment_evaluator.computeGainOptimizedwithMultiplePriors(previous_trajectory, trajectory_point);
        new_node->path_gain = result.first;
        new_node->pose[3] = result.second;*/

        auto [path_gain, max_gain, pose_angle] = segment_evaluator.computeGainOptimizedwithMultiplePriorsMax(previous_trajectory, trajectory_point);
        new_node->path_gain = new_node->parent->path_gain + path_gain;
        new_node->max_gain = new_node->parent->max_gain + max_gain;
        new_node->pose[3] = pose_angle;
        
        if (new_node->max_gain > 0.0) {
            new_node->score = new_node->path_gain / new_node->max_gain * 100.0;
        }

        segment_evaluator.computeCost(new_node);
        //segment_evaluator.computeScore(new_node, lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node;
        }

        //ROS_INFO("[GAEPlanner]: Best Gain Optimized: %f", new_node->path_gain);
        //ROS_INFO("[GAEPlanner]: Best Gain: %f", result2.first);
        //ROS_INFO("[GAEPlanner]: Best Cost: %f", new_node->cost);
        //ROS_INFO("[GAEPlanner]: Best Path Gain: %f", new_node->path_gain);
        //ROS_INFO("[GAEPlanner]: Best Max Gain: %f", new_node->max_gain);
        ROS_INFO("[GAEPlanner]: Best Score: %f", new_node->score);

        RRTStar.addKDTreeNode(new_node);
        visualize_edge(new_node, ns);

        /*if (new_node->path_gain > g_zero) {
            cacheNode(new_node);
        }*/

        if (j > N_termination) {
            ROS_INFO("[GAEPlanner]: Going to Global Planning");
            RRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            goto_global_planning = true;
            return;
        }

        ++j;

    }
    
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    next_best_node = best_branch[1];

    ROS_INFO("[GAEPlanner]: Final Best Score: %f", best_score_);
    ROS_INFO("[GAEPlanner]: Final Best Gain: %f", next_best_node->path_gain);
    ROS_INFO("[GAEPlanner]: Final Best Maximum Gain: %f", next_best_node->max_gain);
    ROS_INFO("[GAEPlanner]: Node Iterations: %d", j);

    /*for (int k = 1; k < best_branch.size(); ++k) {
        if (best_branch[k]->score > g_zero) {
            next_best_node = best_branch[k];
            std::vector<std::shared_ptr<geo_rrt::Node>>::iterator start = best_branch.begin() + k - 1;
            std::vector<std::shared_ptr<geo_rrt::Node>>::iterator end = best_branch.end();
            std::vector<std::shared_ptr<geo_rrt::Node>> sliced_branch(start, end);
            best_branch = sliced_branch;
            break;
        }
    }*/
}

void GAEPlanner::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, std::shared_ptr<geo_rrt::Node>& best_global_node) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[GAEPlanner]: Terminate AEP");

        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    ROS_INFO("[GAEPlanner]: Start Expanding Global");
    
    std::shared_ptr<geo_rrt::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<geo_rrt::Node>(next_start);
    } else {
        root = std::make_shared<geo_rrt::Node>(pose);
    }

    RRTStar.addKDTreeNode(root);
    std::vector<std::shared_ptr<geo_rrt::Node>> all_global_goals;

    collision_id_counter_ = 0;
    int m = 0;
    while (m < N_min_nodes || all_global_goals.size() <= 0) {
        if (collision_id_counter_ > 10000 * (m+1)) {
            if (previous_node) {
                ROS_INFO("[GAEPlanner]: Backtracking to [%f, %f, %f]", previous_node->pose[0], previous_node->pose[1], previous_node->pose[2]);
                next_best_node = previous_node;
                backtrack = true;
                return;
                //rotate();
                //changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[GAEPlanner]: Backtrack Rotation");
                rotate();
                collision_id_counter_ = 0;
            }
        }

        Eigen::Vector3d rand_point_star;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
        rand_point_star += root->pose.head(3);

        std::shared_ptr<geo_rrt::Node> nearest_node_star;
        RRTStar.findNearestKD(rand_point_star, nearest_node_star);

        std::shared_ptr<geo_rrt::Node> new_node_star;
        RRTStar.steer_parent(nearest_node_star, rand_point_star, step_size, new_node_star);

        // Collision Check
        std::vector<std::shared_ptr<geo_rrt::Node>> trajectory_segment_star;
        trajectory_segment_star.push_back(new_node_star);

        if (!isPathCollisionFree(trajectory_segment_star)) {
            trajectory_segment_star.clear();
            continue;
        }

        if (!isInsideBoundingBox(new_node_star)) {
            continue;
        }

        trajectory_segment_star.clear();
        visualize_node(new_node_star->pose, ns);

        // Add Nodes
        std::vector<std::shared_ptr<geo_rrt::Node>> nearby_nodes_star;
        RRTStar.findNearbyKD(new_node_star, radius, nearby_nodes_star);
        RRTStar.chooseParent(new_node_star, nearby_nodes_star);

        RRTStar.addKDTreeNode(new_node_star);
        RRTStar.rewire(new_node_star, nearby_nodes_star, radius);
        visualize_edge(new_node_star, ns);

        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_global;        
        trajectory_point_global.position_W = new_node_star->pose.head(3);
        trajectory_point_global.setFromYaw(new_node_star->pose[3]);

        auto parent_node = new_node_star->parent;
        std::vector<eth_mav_msgs::EigenTrajectoryPoint> previous_trajectory;
        while (parent_node->parent) {
            previous_trajectory_point.position_W = parent_node->pose.head(3);
            previous_trajectory_point.setFromYaw(parent_node->pose[3]);
            previous_trajectory.push_back(previous_trajectory_point);
            parent_node = parent_node->parent;
        }

        auto [path_gain, max_gain, pose_angle] = segment_evaluator.computeGainOptimizedwithMultiplePriorsMax(previous_trajectory, trajectory_point_global);
        new_node_star->path_gain = new_node_star->parent->path_gain + path_gain;
        new_node_star->max_gain = new_node_star->parent->max_gain + max_gain;
        new_node_star->pose[3] = pose_angle;

        if (new_node_star->max_gain > 0.0) {
            new_node_star->score = new_node_star->path_gain / new_node_star->max_gain * 100.0;
        }

        //bool goal_reached = getGlobalGoal(GlobalFrontiers, new_node_star);
        std::pair<bool, std::shared_ptr<geo_rrt::Node>> result = getGlobalGoal(GlobalFrontiers, new_node_star);
        bool goal_reached = result.first;
        std::shared_ptr<geo_rrt::Node> end_node_global = result.second;        
        if (goal_reached) {
            //segment_evaluator.computeScore(new_node_star, global_lambda);
            //all_global_goals.push_back(new_node_star);
            all_global_goals.push_back(end_node_global);
            goal_reached = false;
        }
        ++m;
    }

    ROS_INFO("[GAEPlanner]: Global Planner Ends");

    getBestGlobalPath(all_global_goals, best_global_node);
    all_global_goals.clear();
}

void GAEPlanner::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
    cache_nodes::BestNode srv;
    srv.request.threshold = g_zero;
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

std::pair<bool, std::shared_ptr<geo_rrt::Node>> GAEPlanner::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, const std::shared_ptr<geo_rrt::Node>& node) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    for (size_t i = 1; i < GlobalFrontiers.size(); ++i) {
        goals_tree.addKDTreePoint(GlobalFrontiers[i]);
    }

    // Find the nearest node in the KD Tree
    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(node->pose.head(3), nearest_goal);
    if (nearest_goal.size() <= 0) {
        goals_tree.clearKDTreePoints();
        return std::make_pair(false, nullptr);
    }

    if ((nearest_goal - node->pose.head(3)).norm() < tolerance) {
        Eigen::Vector4d end_pose = {nearest_goal[0], nearest_goal[1], nearest_goal[2], 0.0};

        std::shared_ptr<geo_rrt::Node> end_point;
        end_point = std::make_shared<geo_rrt::Node>(end_pose);
        end_point->parent = node;

        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_global;       
        trajectory_point_global.position_W = nearest_goal;
        trajectory_point_global.setFromYaw(0.0);

        auto parent_node = end_point->parent;
        std::vector<eth_mav_msgs::EigenTrajectoryPoint> previous_trajectory;
        while (parent_node->parent) {
            previous_trajectory_point.position_W = parent_node->pose.head(3);
            previous_trajectory_point.setFromYaw(parent_node->pose[3]);
            previous_trajectory.push_back(previous_trajectory_point);
            parent_node = parent_node->parent;
        }

        auto [path_gain, max_gain, pose_angle] = segment_evaluator.computeGainOptimizedwithMultiplePriorsMax(previous_trajectory, trajectory_point_global);
        end_point->path_gain = end_point->parent->path_gain + path_gain;
        end_point->max_gain = end_point->parent->max_gain + max_gain;
        end_point->pose[3] = pose_angle;

        if (end_point->max_gain > 0.0) {
            end_point->score = end_point->path_gain / end_point->max_gain * 100.0;
        }

        if (end_point->path_gain < 0.1) {
            return std::make_pair(false, nullptr);
        }

        goals_tree.clearKDTreePoints();
        return std::make_pair(true, end_point);
    }

    goals_tree.clearKDTreePoints();
    return std::make_pair(false, nullptr);
}

void GAEPlanner::getBestGlobalPath(const std::vector<std::shared_ptr<geo_rrt::Node>>& global_goals, std::shared_ptr<geo_rrt::Node>& best_global_node) {
    if (global_goals.size() == 0) {
        best_global_node = nullptr;
        return;
    }

    best_global_node = global_goals[0];

    /*// Cost Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->cost > global_goals[i]->cost) {
            best_global_node = global_goals[i];
        }
    }*/

    /*// Gain Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->path_gain < global_goals[i]->path_gain) {
            best_global_node = global_goals[i];
        }
    }*/

    // Score Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->score < global_goals[i]->score) {
            best_global_node = global_goals[i];
        }
    }

    /*std::shared_ptr<geo_rrt::Node> auxiliar_node = best_global_node;

    // Skip the last best node
    if (auxiliar_node->parent) {
        auxiliar_node = auxiliar_node->parent;
    }

    // Update the yaw to follow the path
    while (auxiliar_node->parent) {
        double dx = auxiliar_node->pose.x() - auxiliar_node->parent->pose.x();
        double dy = auxiliar_node->pose.y() - auxiliar_node->parent->pose.y();
        auxiliar_node->pose[3] = std::atan2(dy, dx);

        auxiliar_node = auxiliar_node->parent;
    }*/

    ROS_INFO("[GAEPlanner]: Chosen Goal: [%f, %f, %f]", best_global_node->pose[0], best_global_node->pose[1], best_global_node->pose[2]);
    ROS_INFO("[GAEPlanner]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_node->path_gain, best_global_node->cost, best_global_node->score);

    visualize_path(best_global_node, ns);
}

/*void GAEPlanner::cacheNode(std::shared_ptr<geo_rrt::Node> Node) {
    if (!Node) {
        return;
    }
    cache_nodes::Node cached_node;
    cached_node.gain = Node->path_gain;
    cached_node.position.x = Node->pose[0];
    cached_node.position.y = Node->pose[1];
    cached_node.position.z = Node->pose[2];
    cached_node.yaw = Node->pose[3];
    pub_node.publish(cached_node);
}*/

double GAEPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint->position.x, waypoint->position.y, waypoint->position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

void GAEPlanner::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[GAEPlanner]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(3).sleep();

    ROS_INFO("[GAEPlanner]: Rotating 360 degrees");

    for (double i = 0.0; i <= 2.0; i = i + 0.4) {
        initial_reference.reference.position.x = pose[0];
        initial_reference.reference.position.y = pose[1];
        initial_reference.reference.position.z = pose[2] + 3;
        initial_reference.reference.heading = pose[3] + M_PI * i;
        pub_initial_reference.publish(initial_reference);
        // Max yaw rate is 0.5 rad/s so we wait 0.4*M_PI seconds between points
        ros::Duration(0.4*M_PI).sleep();
    }

    ros::Duration(0.5).sleep();

    ROS_INFO("[GAEPlanner]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(2).sleep();
}

void GAEPlanner::rotate() {
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

bool GAEPlanner::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[GAEPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;

}

bool GAEPlanner::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[GAEPlanner]: " << ss.str());

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

void GAEPlanner::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[GAEPlanner]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void GAEPlanner::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[GAEPlanner]: getting UavState diagnostics");
    //uav_state = *msg;
    geometry_msgs::Pose uav_state = msg->pose;
    double yaw = mrs_lib::getYaw(uav_state);
    pose = {uav_state.position.x, uav_state.position.y, uav_state.position.z, yaw};
}

void GAEPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[GAEPlanner]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[GAEPlanner]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[GAEPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[GAEPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }
    
    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[GAEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[GAEPlanner]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[GAEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[GAEPlanner]: waiting for command");
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
            AEP();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            iteration_ += 1;

            if (!current_waypoint_) {
                current_waypoint_ = std::make_unique<mrs_msgs::Reference>();
            }

            current_waypoint_->position.x = next_best_node->pose[0];
            current_waypoint_->position.y = next_best_node->pose[1];
            current_waypoint_->position.z = next_best_node->pose[2];
            current_waypoint_->heading = next_best_node->pose[3];

            next_start[0] = current_waypoint_->position.x;
            next_start[1] = current_waypoint_->position.y;
            next_start[2] = current_waypoint_->position.z;
            next_start[3] = current_waypoint_->heading;

            visualize_frustum(next_best_node);
            visualize_unknown_voxels(next_best_node);

            mrs_msgs::GetPathSrv srv_get_path;

            srv_get_path.request.path.header.frame_id = ns + "/" + frame_id;
            srv_get_path.request.path.header.stamp = ros::Time::now();
            srv_get_path.request.path.fly_now = false;
            srv_get_path.request.path.use_heading = true;

            mrs_msgs::Reference reference;

            if (next_best_node && next_best_node->parent) {
                previous_node = std::make_shared<geo_rrt::Node>(*next_best_node->parent);
            }

            while (next_best_node && next_best_node->parent) {
                reference.position.x = next_best_node->pose[0];
                reference.position.y = next_best_node->pose[1];
                reference.position.z = next_best_node->pose[2];
                reference.heading = next_best_node->pose[3];
                srv_get_path.request.path.points.push_back(reference);
                next_best_node = next_best_node->parent;
            }

            reference.position.x = next_best_node->pose[0];
            reference.position.y = next_best_node->pose[1];
            reference.position.z = next_best_node->pose[2];
            reference.heading = next_best_node->pose[3];
            srv_get_path.request.path.points.push_back(reference);

            std::reverse(srv_get_path.request.path.points.begin(), srv_get_path.request.path.points.end());

            for (const auto& point : srv_get_path.request.path.points) {
                pub_reference.publish(point);
            }

            bool success = sc_trajectory_generation.call(srv_get_path);

            if (!success) {
                ROS_ERROR("[GAEPlanner]: service call for trajectory failed");
                ROS_ERROR("[GAEPlanner]: Path size: %ld", srv_get_path.request.path.points.size());
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_get_path.response.success) {
                    ROS_ERROR("[GAEPlanner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
                    changeState(STATE_STOPPED);
                    return;
                }
            }

            mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
            srv_trajectory_reference.request.trajectory = srv_get_path.response.trajectory;
            srv_trajectory_reference.request.trajectory.fly_now = true;

            bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

            if (!success_trajectory) {
                ROS_ERROR("[GAEPlanner]: service call for trajectory reference failed");
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_trajectory_reference.response.success) {
                    ROS_ERROR("[GAEPlanner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
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
                ROS_INFO("[GAEPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_->heading - current_yaw), cos(current_waypoint_->heading - current_yaw)));
                ROS_INFO("[GAEPlanner]: Distance to waypoint: %.2f", dist);
                if (dist <= 0.6*step_size && yaw_difference <= 0.4*M_PI) {
                    changeState(STATE_PLANNING);
                }
            } else {
                ROS_INFO("[GAEPlanner]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[GAEPlanner]: Total Iterations: %d", iteration_);
            ROS_INFO("[GAEPlanner]: Shutting down.");
            ros::shutdown();
            return;
        }
        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[GAEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[GAEPlanner]: waiting for command");
            }
            break;
        }
    }
}

void GAEPlanner::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[GAEPlanner]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[GAEPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void GAEPlanner::visualize_node(const Eigen::Vector4d& pos, const std::string& ns) {
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

    node_id_counter_++;

    n.lifetime = ros::Duration(30.0);
    n.frame_locked = false;
    pub_markers.publish(n);
}

void GAEPlanner::visualize_edge(const std::shared_ptr<geo_rrt::Node> node, const std::string& ns) {
    visualization_msgs::Marker e;
    e.header.stamp = ros::Time::now();
    e.header.seq = edge_id_counter_;
    e.header.frame_id = ns + "/" + frame_id;
    e.id = edge_id_counter_;
    e.ns = "tree_branches";
    e.type = visualization_msgs::Marker::ARROW;
    e.action = visualization_msgs::Marker::ADD;
    //ROS_INFO("[GAEPlanner]: HERE 8");
    e.pose.position.x = node->parent->pose[0];
    e.pose.position.y = node->parent->pose[1];
    e.pose.position.z = node->parent->pose[2];
    //ROS_INFO("[GAEPlanner]: HERE 9");
    Eigen::Quaternion<double> q;
    Eigen::Vector3d init(1.0, 0.0, 0.0);
    Eigen::Vector3d dir(node->pose[0] - node->parent->pose[0],
                        node->pose[1] - node->parent->pose[1],
                        node->pose[2] - node->parent->pose[2]);
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

    edge_id_counter_++;

    e.lifetime = ros::Duration(30.0);
    e.frame_locked = false;
    pub_markers.publish(e);
}

void GAEPlanner::visualize_path(const std::shared_ptr<geo_rrt::Node> node, const std::string& ns) {
    std::shared_ptr<geo_rrt::Node> current = node;
    
    while (current->parent) {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = path_id_counter_;
        p.header.frame_id = ns + "/" + frame_id;
        p.id = path_id_counter_;
        p.ns = "path";
        p.type = visualization_msgs::Marker::ARROW;
        p.action = visualization_msgs::Marker::ADD;
        p.pose.position.x = current->parent->pose[0];
        p.pose.position.y = current->parent->pose[1];
        p.pose.position.z = current->parent->pose[2];

        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        Eigen::Vector3d dir(current->pose[0] - current->parent->pose[0],
                            current->pose[1] - current->parent->pose[1],
                            current->pose[2] - current->parent->pose[2]);
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

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter_++;
    }
}

void GAEPlanner::visualize_frustum(std::shared_ptr<geo_rrt::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->pose.head(3);
    trajectory_point_visualize.setFromYaw(position->pose[3]);
    
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

void GAEPlanner::visualize_unknown_voxels(std::shared_ptr<geo_rrt::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->pose.head(3);
    trajectory_point_visualize.setFromYaw(position->pose[3]);

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

void GAEPlanner::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void GAEPlanner::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void GAEPlanner::clearMarkers() {
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
