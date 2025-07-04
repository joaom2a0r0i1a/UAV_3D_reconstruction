#include "motion_planning_python/AEPReal/AEPlannerReal.h"

AEPlanner::AEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_) {

    //ns = "uav1";

    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "AEPlanner");

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
    param_loader.loadParam("bounded_box/planner_range", planner_range);

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
    //changeState(STATE_IDLE);
    iteration_ = 0;
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
    transformer_ = std::make_unique<mrs_lib::Transformer>("AEPlanner");
    transformer_->setDefaultFrame(frame_id);
    //transformer_->setDefaultPrefix(ns);
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
    shopts.node_name          = "AEPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_local_pose_diag = mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped>(shopts, "local_pose_in", &AEPlanner::callbackLocalPose, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &AEPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &AEPlanner::callbackStop, this);
    ss_offset = nh_private_.advertiseService("offset_in", &AEPlanner::callbackOffset, this);

    /* Service Clients */
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &AEPlanner::timerMain, this);

    is_initialized = true;
}

double AEPlanner::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool AEPlanner::isPathCollisionFree(const std::vector<std::shared_ptr<rrt_star::Node>>& path) const {
    for (const std::shared_ptr<rrt_star::Node>& node : path) {
        if (getMapDistance(node->point.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

void AEPlanner::GetTransformation() {
    // From Body Frame to Camera Frame
    ros::Duration(0.2).sleep();
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[AEPlanner]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void AEPlanner::AEP() {
    localPlanner();
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_node = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[AEPlanner]: Getting Global Frontiers");
        getGlobalFrontiers(GlobalFrontiers);
        if (GlobalFrontiers.size() == 0) {
            changeState(STATE_STOPPED);
            return;   
        }
        /*for (size_t i = 0; i < GlobalFrontiers.size(); ++i) {
            std::cout << "Global Frontier: " << GlobalFrontiers[i] << std::endl;
            //std::cout << "Gain: " << GlobalFrontiers[i] << std::endl;
        }*/
        ROS_INFO("[AEPlanner]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_node);
        
        if (go_terminate) {
            ROS_INFO("[AEPlanner]: No information gain. Terminate.");
            changeState(STATE_STOPPED);
            return;   
        }
        
        next_best_node = best_global_node;
        goto_global_planning = false;
    }
}

void AEPlanner::localPlanner() {
    best_score_ = 0;
    std::shared_ptr<rrt_star::Node> best_node = nullptr;
    //std::shared_ptr<rrt_star::Node> root = std::make_shared<rrt_star::Node>(pose);
    //std::shared_ptr<rrt_star::Node> root = std::make_shared<rrt_star::Node>(best_branch[1]->point);
    std::shared_ptr<rrt_star::Node> root;
    if (best_branch.size() > 1) {
        root = std::make_shared<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_shared<rrt_star::Node>(pose);
    }
    trajectory_point.position_W = root->point.head(3);
    trajectory_point.setFromYaw(root->point[3]);
    //std::pair<double, double> result = segment_evaluator.computeGainAEP(trajectory_point);
    std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point, initial_offset);
    root->gain = result.first;
    root->point[3] = result.second;
    //segment_evaluator.computeGainFromsampledYaw(root, num_yaw_samples, trajectory_point);

    root->cost = 0;
    root->score = root->gain;

    if (root->score > best_score_) {
        best_score_ = root->score;
        best_node = root;
    }

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    /*if (root->gain > g_zero) {
        cacheNode(root);
    }*/
    //cacheNode(root);
    clearMarkers();

    visualize_node(root->point, ns);
    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    if (best_branch.size() > 0) {
        previous_root = best_branch[0];
    }
    while (j < N_max || best_score_ <= g_zero) {
        /*// Backtrack
        if (collision_id_counter_ > 10000 * j) {
            if (previous_root) {
                //next_best_node = previous_root;
                rotate();
                changeState(STATE_WAITING_INITIALIZE);
            } else {
                ROS_INFO("[AEPlanner]: Enough");
                collision_id_counter_ = 0;
                break;
            }
            return;
        }*/

        // Add previous best branch 
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue; // Skip first iteration (root)
            }
            
            const Eigen::Vector4d& node_position = best_branch[i]->point;

            std::shared_ptr<rrt_star::Node> nearest_node_best;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            
            std::shared_ptr<rrt_star::Node> new_node_best;
            new_node_best = std::make_shared<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;

            /*std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment_best;
            trajectory_segment_best.push_back(new_node_best->parent);
            trajectory_segment_best.push_back(new_node_best);

            bool success_collision_best = false;
            success_collision_best = isPathCollisionFree(trajectory_segment_best);

            if (!success_collision_best) {
                clear_node();
                trajectory_segment_best.clear();
                break;
            }

            trajectory_segment_best.clear();*/
            visualize_node(new_node_best->point, ns);

            trajectory_point.position_W = new_node_best->point.head(3);
            trajectory_point.setFromYaw(new_node_best->point[3]);
            //std::pair<double, double> result = segment_evaluator.computeGainFromSampledYawAEP(trajectory_point);
            //std::pair<double, double> result = segment_evaluator.computeGainAEP(trajectory_point);
            //std::pair<double, double> result = segment_evaluator.computeGainRaycastingFromSampledYaw(trajectory_point);
            //std::pair<double, double> result = segment_evaluator.computeGainRaycastingFromOptimizedSampledYaw(trajectory_point);
            std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point, initial_offset);
            new_node_best->gain = result.first;
            new_node_best->point[3] = result.second;

            //segment_evaluator.computeGainFromsampledYaw(new_node_best, num_yaw_samples, trajectory_point);

            segment_evaluator.computeCost(new_node_best);
            segment_evaluator.computeScore(new_node_best, lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best;
            }

            //ROS_INFO("[AEPlanner]: Best Gain Optimized BB: %f", new_node_best->gain);
            //ROS_INFO("[AEPlanner]: Best Gain BB: %f", result2.first);
            //ROS_INFO("[AEPlanner]: Best Cost BB: %f", new_node_best->cost);
            ROS_INFO("[AEPlanner]: Best Score BB: %f", new_node_best->score);
            

            RRTStar.addKDTreeNode(new_node_best);
            //tree.push_back(new_node_best);
            visualize_edge(new_node_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > g_zero) {
            break;
        }
    
        best_branch.clear();

        Eigen::Vector3d rand_point;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
        rand_point += root->point.head(3);

        std::shared_ptr<rrt_star::Node> nearest_node;
        //RRTStar.findNearest(tree, rand_point, nearest_node);
        RRTStar.findNearestKD(rand_point, nearest_node);

        std::shared_ptr<rrt_star::Node> new_node;
        RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

        if (new_node->point[0] > initial_offset[0] + max_x || new_node->point[0] < initial_offset[0] + min_x || new_node->point[1] < initial_offset[1] + min_y || new_node->point[1] > initial_offset[1] + max_y || new_node->point[2] < initial_offset[2] + min_z || new_node->point[2] > initial_offset[2] + max_z) {
            continue;
        }

        //std::shared_ptr<rrt_star::Node> new_node;
        /*if (new_node->point[0] < 12 && new_node->point[0] > -12 && new_node->point[1] > -7.0 && new_node->point[1] < 7.0 && new_node->point[2] > 0 && new_node->point[2] < 12.5) {
            continue;
        }

        if (new_node->point[2] < 0.5 || new_node->point[2] > 14.5) {
            continue;
        }*/

        // Collision Check
        std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment;
        //trajectory_segment.push_back(new_node->parent);
        trajectory_segment.push_back(new_node);

        bool success_collision = false;
        success_collision = isPathCollisionFree(trajectory_segment);

        if (!success_collision) {
            //clear_node();
            trajectory_segment.clear();
            collision_id_counter_++;
            continue;
        }

        trajectory_segment.clear();
        visualize_node(new_node->point, ns);
        
        /*cache_nodes::Query srv;
        srv.request.point.x = new_node->point[0];
        srv.request.point.y = new_node->point[1];
        srv.request.point.z = new_node->point[2];

        bool success_query = sc_query.call(srv);

        if (!success_query) {
            ROS_WARN("[AEPlanner]: Service call for Gaussian process failed");
        }

        if (success_query && srv.response.sigma < sigma_threshold) {
            new_node->gain = srv.response.mu;
            new_node->point[3] = srv.response.yaw;
        } else {
            segment_evaluator.computeGainFromsampledYaw(new_node, num_yaw_samples, trajectory_point);
        }*/

        trajectory_point.position_W = new_node->point.head(3);
        trajectory_point.setFromYaw(new_node->point[3]);
        //std::pair<double, double> result = segment_evaluator.computeGainFromSampledYawAEP(trajectory_point);
        //std::pair<double, double> result = segment_evaluator.computeGainAEP(trajectory_point);
        //std::pair<double, double> result = segment_evaluator.computeGainRaycastingFromSampledYaw(trajectory_point);
        //std::pair<double, double> result = segment_evaluator.computeGainRaycastingFromOptimizedSampledYaw(trajectory_point);
        std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point, initial_offset);
        new_node->gain = result.first;
        new_node->point[3] = result.second;

        //segment_evaluator.computeGainFromsampledYaw(new_node, num_yaw_samples, trajectory_point);

        segment_evaluator.computeCost(new_node);
        segment_evaluator.computeScore(new_node, lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node;
        }

        //ROS_INFO("[AEPlanner]: Best Gain Optimized: %f", new_node->gain);
        //ROS_INFO("[AEPlanner]: Best Gain: %f", result2.first);
        //ROS_INFO("[AEPlanner]: Best Cost: %f", new_node->cost);
        ROS_INFO("[AEPlanner]: Best Score: %f", new_node->score);

        RRTStar.addKDTreeNode(new_node);
        visualize_edge(new_node, ns);

        if (new_node->gain > g_zero) {
            cacheNode(new_node);
        }
        //cacheNode(new_node);

        if (j > N_termination) {
            ROS_INFO("[AEPlanner]: Going to Global Planning");
            RRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            goto_global_planning = true;
            //changeState(STATE_STOPPED);
            return;
        }

        ++j;

    }
    
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    for (int k = 1; k < best_branch.size(); ++k) {
        if (best_branch[k]->gain > g_zero) {
            next_best_node = best_branch[k];
            std::vector<std::shared_ptr<rrt_star::Node>>::iterator start = best_branch.begin() + k - 1;
            std::vector<std::shared_ptr<rrt_star::Node>>::iterator end = best_branch.end();
            std::vector<std::shared_ptr<rrt_star::Node>> sliced_branch(start, end);
            best_branch = sliced_branch;
            break;
        }
    }
}

void AEPlanner::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, std::shared_ptr<rrt_star::Node>& best_global_node) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[AEPlanner]: Terminate AEP");

        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    std::shared_ptr<rrt_star::Node> root = std::make_shared<rrt_star::Node>(pose);
    RRTStar.addKDTreeNode(root);
    std::vector<std::shared_ptr<rrt_star::Node>> all_global_goals;

    //ROS_INFO("[AEPlanner]: I AM HERE");

    int m = 0;
    while (m < N_min_nodes || all_global_goals.size() <= 0) {
        // ADD LOGIC FOR BUILDING RRT* TREE TOWARDS GIVEN GOALS FOR N NUMBER OF ITERATIONS
        Eigen::Vector3d rand_point_star;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
        rand_point_star += root->point.head(3);

        std::shared_ptr<rrt_star::Node> nearest_node_star;
        RRTStar.findNearestKD(rand_point_star, nearest_node_star);

        std::shared_ptr<rrt_star::Node> new_node_star;
        RRTStar.steer_parent(nearest_node_star, rand_point_star, step_size, new_node_star);
        //RRTStar.steer(nearest_node_star, rand_point_star, step_size, new_node_star);

        /*if (new_node_star->point[0] < 12 && new_node_star->point[0] > -12 && new_node_star->point[1] > -7.0 && new_node_star->point[1] < 7.0 && new_node_star->point[2] > 0 && new_node_star->point[2] < 12.5) {
            continue;
        }

        if (new_node_star->point[2] < 0.5 || new_node_star->point[2] > 14.5) {
            continue;
        }*/

        // Collision Check
        std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment_star;
        //trajectory_segment.push_back(new_node->parent);
        trajectory_segment_star.push_back(new_node_star);

        bool success_collision_star = false;
        success_collision_star = isPathCollisionFree(trajectory_segment_star);

        if (!success_collision_star) {
            //clear_node();
            trajectory_segment_star.clear();
            continue;
        }

        trajectory_segment_star.clear();
        visualize_node(new_node_star->point, ns);

        // Add Nodes
        std::vector<std::shared_ptr<rrt_star::Node>> nearby_nodes_star;
        RRTStar.findNearbyKD(new_node_star, radius, nearby_nodes_star);
        RRTStar.chooseParent(new_node_star, nearby_nodes_star);

        RRTStar.addKDTreeNode(new_node_star);
        RRTStar.rewire(new_node_star, nearby_nodes_star, radius);
        visualize_edge(new_node_star, ns);

        // ADD LOGIC TO CHECK IF GOAL HAS BEEN REACHED
        bool goal_reached;
        goal_reached = getGlobalGoal(GlobalFrontiers, new_node_star); // NEED TO ADD FRONTIER HERE
        if (goal_reached) {
            segment_evaluator.computeScore(new_node_star, global_lambda);
            all_global_goals.push_back(new_node_star);
            goal_reached = false;
        }
        ++m;
    }

    ROS_INFO("[AEPlanner]: Global Planner Ends");

    // ADD LOGIC TO GET THE BEST PATH FOR THE REACHED GOALS (SMALLEST COST, I.E., SHORTEST PATH)
    getBestGlobalPath(all_global_goals, best_global_node);
    all_global_goals.clear();
}

void AEPlanner::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
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
            /*if (best_global_gain < srv.response.gain[i]) {
                best_global_gain = srv.response.gain[i];
                best_global_frontier = frontier;
                ROS_INFO("[AEPlanner]: Best Global Gains: %f", best_global_gain);
                GlobalFrontiers.push_back(best_global_frontier);
            }*/
            // Remove all elements except the last two
            //if (GlobalFrontiers.size() > 4) {
            //    GlobalFrontiers.erase(GlobalFrontiers.begin(), GlobalFrontiers.end() - 4);
            //}
            GlobalFrontiers.push_back(frontier);
        }
        return;
        //GlobalFrontiers.push_back(best_global_frontier);
    }
    else {
        return;
    }
}

bool AEPlanner::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, const std::shared_ptr<rrt_star::Node>& node) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    for (size_t i = 1; i < GlobalFrontiers.size(); ++i) {
        goals_tree.addKDTreePoint(GlobalFrontiers[i]);
    }
    //goals_tree.initializeKDTreeWithPoints(GlobalFrontiers);

    // Find the nearest node in the KD Tree
    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(node->point.head(3), nearest_goal);
    if (nearest_goal.size() <= 0) {
        goals_tree.clearKDTreePoints();
        return false;
    }

    if ((nearest_goal - node->point.head(3)).norm() < tolerance) {
        //ROS_INFO("[AEPlanner]: Goal: [%f, %f, %f]", nearest_goal[0], nearest_goal[1], nearest_goal[2]);
        //ROS_INFO("[AEPlanner]: RRT* Goal: [%f, %f, %f]", node->point[0], node->point[1], node->point[2]);    

        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_global;
        trajectory_point_global.position_W = node->point.head(3);
        trajectory_point_global.setFromYaw(node->point[3]);
        std::pair<double, double> result = segment_evaluator.computeGainOptimizedAEP(trajectory_point_global, initial_offset);
        //std::pair<double, double> result = segment_evaluator.computeGainRaycastingFromOptimizedSampledYaw(trajectory_point_global);

        node->gain = result.first;
        node->point[3] = result.second;

        trajectory_point_global.position_W = nearest_goal;
        trajectory_point_global.setFromYaw(0.0);
        std::pair<double, double> result_original = segment_evaluator.computeGainOptimizedAEP(trajectory_point_global, initial_offset);
        //ROS_INFO("[AEPlanner]: Goal Best Gain: %f", result_original.first);
        goals_tree.clearKDTreePoints();
        return true;
    }

    goals_tree.clearKDTreePoints();
    return false;
}

void AEPlanner::getBestGlobalPath(const std::vector<std::shared_ptr<rrt_star::Node>>& global_goals, std::shared_ptr<rrt_star::Node>& best_global_node) {
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
        if (best_global_node->gain < global_goals[i]->gain) {
            best_global_node = global_goals[i];
        }
    }*/

    // Score Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->score < global_goals[i]->score) {
            best_global_node = global_goals[i];
        }
    }

    std::shared_ptr<rrt_star::Node> auxiliar_node = best_global_node;

    // Skip the last best node
    if (auxiliar_node->parent) {
        auxiliar_node = auxiliar_node->parent;
    }

    // Update the yaw to follow the path
    while (auxiliar_node->parent) {
        double dx = auxiliar_node->point.x() - auxiliar_node->parent->point.x();
        double dy = auxiliar_node->point.y() - auxiliar_node->parent->point.y();
        auxiliar_node->point[3] = std::atan2(dy, dx);

        auxiliar_node = auxiliar_node->parent;
    }

    for (size_t i = 0; i < global_goals.size(); ++i) {
        //ROS_INFO("[AEPlanner]: Obtained Goal: [%f, %f, %f]", global_goals[i]->point[0], global_goals[i]->point[1], global_goals[i]->point[2]);
        //ROS_INFO("[AEPlanner]: Obtained Goal Gain, Cost & Score: [%f, %f, %f]", global_goals[i]->gain, global_goals[i]->cost, global_goals[i]->score);
        /*std::cout << "Obtained Goal X: " << global_goals[i]->point[0] << std::endl;
        std::cout << "Obtained Goal Y: " << global_goals[i]->point[1] << std::endl;
        std::cout << "Obtained Goal Z: " << global_goals[i]->point[2] << std::endl;
        std::cout << "Obtained Goal Cost: " << global_goals[i]->cost << std::endl;*/
    }

    ROS_INFO("[AEPlanner]: Chosen Goal: [%f, %f, %f]", best_global_node->point[0], best_global_node->point[1], best_global_node->point[2]);
    ROS_INFO("[AEPlanner]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_node->gain, best_global_node->cost, best_global_node->score);
    
    if (best_global_node->gain < 0.2) {
        go_terminate = true;
    }
    
    /*std::cout << "Chosen Goal X: " << best_global_node->point[0] << std::endl;
    std::cout << "Chosen Goal Y: " << best_global_node->point[1] << std::endl;
    std::cout << "Chosen Goal Z: " << best_global_node->point[2] << std::endl;
    std::cout << "Chosen Goal Gain: " << best_global_node->gain << std::endl;
    std::cout << "Chosen Goal Cost: " << best_global_node->cost << std::endl;*/

    visualize_path(best_global_node, ns);
}

void AEPlanner::cacheNode(std::shared_ptr<rrt_star::Node> Node) {
    if (!Node) {
        return;
    }
    cache_nodes::Node cached_node;
    cached_node.gain = Node->gain;
    cached_node.position.x = Node->point[0];
    cached_node.position.y = Node->point[1];
    cached_node.position.z = Node->point[2];
    cached_node.yaw = Node->point[3];
    pub_node.publish(cached_node);
}

double AEPlanner::distance(Eigen::Vector4d current_pose, Eigen::Vector3d waypoint) {
    double dx = waypoint[0] - current_pose[0];
    double dy = waypoint[1] - current_pose[1];
    double dz = waypoint[2] - current_pose[2];
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist;
}

bool AEPlanner::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[AEPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    interrupted_ = false;
    changeState(STATE_PLANNING);

    res.success = true;
    res.message = "starting";
    return true;

}

bool AEPlanner::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[AEPlanner]: " << ss.str());

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

bool AEPlanner::callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[NBVPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }
    initial_offset[0] = pose[0];
    initial_offset[1] = pose[1];
    initial_offset[2] = pose[2];
    
    geometry_msgs::Point offset_msg;
    offset_msg.x = initial_offset[0];
    offset_msg.y = initial_offset[1];
    offset_msg.z = initial_offset[2];
    
    pub_offset.publish(offset_msg);
    
    std::stringstream ss;
    ss << "Getting initial position offset: [" << initial_offset[0] << ", " << initial_offset[1] << ", " << initial_offset[2] << "]";

    res.success = true;
    res.message = ss.str();
    return true;

}

void AEPlanner::callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[AEPlanner]: getting LocalPose diagnostics");
    uav_local_pose = msg->pose;
    
    const geometry_msgs::Quaternion& q = uav_local_pose.orientation;

    // Check for NaNs or zero-length quaternion
    if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
        ROS_ERROR("[AEPlanner]: Invalid quaternion received (contains NaNs)");
        return;
    }
    
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);    
    if (norm < 0.1 || norm > 1.1) {
      ROS_WARN_THROTTLE(5, "[AEPlanner] Invalid quaternion detected. Norm: %.3f. Skipping this pose.", norm);
      return;
    }
    
    double yaw = 0.0;
    try {
        yaw = mrs_lib::getYaw(uav_local_pose);
    } catch (const mrs_lib::AttitudeConverter::InvalidAttitudeException& e) {
        ROS_ERROR_THROTTLE(1.0, "[AEPlanner]: Exception during getYaw(): %s — skipping this pose.", e.what());
        return;
    }
    
    pose = {uav_local_pose.position.x, uav_local_pose.position.y, uav_local_pose.position.z, yaw};
    //ROS_INFO("[AEPlanner]: [%f, %f, %f, %f]", uav_local_pose.position.x, uav_local_pose.position.y, uav_local_pose.position.z, yaw);
}

void AEPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    ready_to_plan_ = true;

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[AEPlanner]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[AEPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[AEPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }
    
    switch (state_) {
        case STATE_IDLE: {
            ROS_INFO("[AEPlanner]: waiting for command");
            break;
        }
        case STATE_PLANNING: {
            AEP();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            iteration_ += 1;

            visualize_frustum(next_best_node);
            visualize_unknown_voxels(next_best_node);

            active_waypoints.clear();

            while (next_best_node && next_best_node->parent) {
                mavros_msgs::PositionTarget setpoint_reference;
                    
                setpoint_reference.header.frame_id = frame_id;
                setpoint_reference.header.stamp = ros::Time::now();
                setpoint_reference.coordinate_frame = 1;
                setpoint_reference.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
                    
                setpoint_reference.position.x = next_best_node->point[0];
                setpoint_reference.position.y = next_best_node->point[1];
                setpoint_reference.position.z = next_best_node->point[2];
                setpoint_reference.yaw = next_best_node->point[3];    
                
                active_waypoints.push_back(setpoint_reference);
                next_best_node = next_best_node->parent;
            }

            std::reverse(active_waypoints.begin(), active_waypoints.end());

            current_wp_idx = 0;
            has_active_path = true;

            changeState(STATE_MOVING);
            break;
        }
        case STATE_MOVING: {
            if (!has_active_path || current_wp_idx >= active_waypoints.size()) {
                ROS_INFO("[AEPlanner]: Finished all waypoints.");
                has_active_path = false;
                changeState(STATE_PLANNING);
                break;
            }

            mavros_msgs::PositionTarget current_sp = active_waypoints[current_wp_idx];
            Eigen::Vector3d target = {
                current_sp.position.x,
                current_sp.position.y,
                current_sp.position.z
            };

            double dist = distance(pose, target);
            ROS_INFO_THROTTLE(1.0, "Distance to waypoint %lu: %f", current_wp_idx, dist);

            pub_setpoint.publish(current_sp);

            if (dist < 0.5) {
                ROS_INFO("Reached waypoint %lu", current_wp_idx);
                current_wp_idx++;
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[AEPlanner]: Total Iterations: %d", iteration_);
            ROS_INFO("[AEPlanner]: Shutting down.");
            ros::shutdown();
            return;
        }
    }
}

void AEPlanner::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (interrupted_ && old_state == STATE_STOPPED) {
        ROS_WARN("[AEPlanner]: Planning interrupted, not changing state.");
        return;
    }

    switch (new_state) {

        case STATE_PLANNING: {

            if (old_state == STATE_STOPPED) {
                replanning_counter_ = 0;
            }
        }

        default: {break;}
    }

    ROS_INFO("[AEPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void AEPlanner::visualize_node(const Eigen::Vector4d& pos, const std::string& ns) {
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

void AEPlanner::visualize_edge(const std::shared_ptr<rrt_star::Node> node, const std::string& ns) {
    visualization_msgs::Marker e;
    e.header.stamp = ros::Time::now();
    e.header.seq = edge_id_counter_;
    e.header.frame_id = frame_id;
    e.id = edge_id_counter_;
    e.ns = "tree_branches";
    e.type = visualization_msgs::Marker::ARROW;
    e.action = visualization_msgs::Marker::ADD;
    //ROS_INFO("[AEPlanner]: HERE 8");
    e.pose.position.x = node->parent->point[0];
    e.pose.position.y = node->parent->point[1];
    e.pose.position.z = node->parent->point[2];
    //ROS_INFO("[AEPlanner]: HERE 9");
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

    edge_id_counter_++;

    e.lifetime = ros::Duration(30.0);
    e.frame_locked = false;
    pub_markers.publish(e);
}

void AEPlanner::visualize_path(const std::shared_ptr<rrt_star::Node> node, const std::string& ns) {
    std::shared_ptr<rrt_star::Node> current = node;
    
    while (current->parent) {
        visualization_msgs::Marker p;
        p.header.stamp = ros::Time::now();
        p.header.seq = path_id_counter_;
        p.header.frame_id = frame_id;
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

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter_++;
    }
}

void AEPlanner::visualize_frustum(std::shared_ptr<rrt_star::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->point.head(3);
    trajectory_point_visualize.setFromYaw(position->point[3]);
    
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

void AEPlanner::visualize_unknown_voxels(std::shared_ptr<rrt_star::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->point.head(3);
    trajectory_point_visualize.setFromYaw(position->point[3]);

    voxblox::Pointcloud voxel_points;
    segment_evaluator.visualizeGainAEP(trajectory_point_visualize, voxel_points);
    
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

void AEPlanner::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void AEPlanner::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void AEPlanner::clearMarkers() {
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
