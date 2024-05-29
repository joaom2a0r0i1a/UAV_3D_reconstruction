#include "AEP/AEPlanner.h"

AEPlanner::AEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_) {

    ns = "uav1";

    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "AEPlanner");

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
    param_loader.loadParam("camera/frame_id", camera_frame_id);
    param_loader.loadParam("center/x", center_x);
    param_loader.loadParam("center/y", center_y);
    param_loader.loadParam("center/z", center_z);
    param_loader.loadParam("dimensions/x", dimensions_x);
    param_loader.loadParam("dimensions/y", dimensions_y);
    param_loader.loadParam("dimensions/z", dimensions_z);

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
    param_loader.loadParam("local_planning/sigma_thresh", sigma_threshold);

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

    // Timer
    param_loader.loadParam("timer_main/rate", timer_main_rate);

    // Initialize UAV as state IDLE
    state_ = STATE_IDLE;
    //changeState(STATE_IDLE);
    iteration_ = 0;

    // Get vertical FoV and setup camera
    vertical_fov = segment_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
    segment_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
            
    // Setup Tf Transformer
    transformer_ = std::make_unique<mrs_lib::Transformer>("AEPlanner");
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

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "AEPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &AEPlanner::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &AEPlanner::callbackControlManagerDiagnostics, this);
    //sub_constraints = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(shopts, "constraints_in");

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &AEPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &AEPlanner::callbackStop, this);
    //ss_reevaluate = nh_private_.advertiseService("reevaluate_in", &AEPlanner::callbackReevaluate, this);

    /* Service Clients */
    sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_private_, "trajectory_generation_out");
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");
    sc_query = mrs_lib::ServiceClientHandler<cache_nodes::Query>(nh_private_, "gp_query_out");

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
        getGlobalFrontiers(GlobalFrontiers);
        for (size_t i = 1; i < GlobalFrontiers.size(); ++i) {
            std::cout << "Global Frontier: " << GlobalFrontiers[i] << std::endl;
        }
        globalPlanner(GlobalFrontiers, best_global_node);
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
    segment_evaluator.computeGainFromsampledYaw(root, num_yaw_samples, trajectory_point);

    root->cost = 0;
    root->score = root->gain;

    if (root->score > best_score_) {
        best_score_ = root->score;
        best_node = root;
    }

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    cacheNode(root);
    clearMarkers();

    visualize_node(root->point, ns);
    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    while (j < N_max || best_score_ <= g_zero) {
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

            segment_evaluator.computeGainFromsampledYaw(new_node_best, num_yaw_samples, trajectory_point);

            segment_evaluator.computeCost(new_node_best);
            segment_evaluator.computeScore(new_node_best, lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best;
            }

            //ROS_INFO("[AEPlanner]: Best Gain BB: %f", new_node_best->gain);
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

        //std::shared_ptr<rrt_star::Node> new_node;
        if (new_node->point[0] < 12 && new_node->point[0] > -12 && new_node->point[1] > -7.0 && new_node->point[1] < 7.0 && new_node->point[2] > 0 && new_node->point[2] < 12.5) {
            continue;
        }

        if (new_node->point[2] < 0.5 || new_node->point[2] > 14.5) {
            continue;
        }

        /*std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment;
        trajectory_segment_best.push_back(new_node->parent);
        trajectory_segment_best.push_back(new_node);

        bool success_collision_best = false;
        success_collision_best = isPathCollisionFree(trajectory_segment);

        if (!success_collision_best) {
            clear_node();
            trajectory_segment_best.clear();
            break;
        }

        trajectory_segment_best.clear();*/
        visualize_node(new_node->point, ns);
        collision_id_counter_ = 0;

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

        segment_evaluator.computeGainFromsampledYaw(new_node, num_yaw_samples, trajectory_point);

        segment_evaluator.computeCost(new_node);
        segment_evaluator.computeScore(new_node, lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node;
        }

        //ROS_INFO("[AEPlanner]: Best Gain: %f", new_node->gain);
        //ROS_INFO("[AEPlanner]: Best Cost: %f", new_node->cost);
        ROS_INFO("[AEPlanner]: Best Score: %f", new_node->score);

        RRTStar.addKDTreeNode(new_node);
        visualize_edge(new_node, ns);
        cacheNode(new_node);

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

    for (int m = 1; m < N_min_nodes; ++m) {
        // ADD LOGIC FOR BUILDING RRT* TREE TOWARDS GIVEN GOALS FOR N NUMBER OF ITERATIONS
        Eigen::Vector3d rand_point_star;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
        rand_point_star += root->point.head(3);

        std::shared_ptr<rrt_star::Node> nearest_node_star;
        RRTStar.findNearestKD(rand_point_star, nearest_node_star);

        std::shared_ptr<rrt_star::Node> new_node_star;
        RRTStar.steer(nearest_node_star, rand_point_star, step_size, new_node_star);

        if (new_node_star->point[0] < 12 && new_node_star->point[0] > -12 && new_node_star->point[1] > -7.0 && new_node_star->point[1] < 7.0 && new_node_star->point[2] > 0 && new_node_star->point[2] < 12.5) {
            continue;
        }

        if (new_node_star->point[2] < 0.5 || new_node_star->point[2] > 14.5) {
            continue;
        }

        visualize_node(new_node_star->point, ns);
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
            all_global_goals.push_back(new_node_star);
            goal_reached = false;
        }
    }

    ROS_INFO("[AEPlanner]: HERE GLOBAL PLANNER END");

    // ADD LOGIC TO GET THE BEST PATH FOR THE REACHED GOALS (SMALLEST COST, I.E., SHORTEST PATH)
    getBestGlobalPath(all_global_goals, best_global_node);
    all_global_goals.clear();
}

void AEPlanner::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
    cache_nodes::BestNode srv;
    srv.request.threshold = 20;
    if (sc_best_node.call(srv)) {
        for (int i = 0; i < srv.response.best_node.size(); ++i) {
            Eigen::Vector3d frontier;
            frontier[0] = srv.response.best_node[i].x;
            frontier[1] = srv.response.best_node[i].y;
            frontier[2] = srv.response.best_node[i].z;
            GlobalFrontiers.push_back(frontier);
        }
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

    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->cost > global_goals[i]->cost) {
            best_global_node = global_goals[i];
        }
    }

    std::shared_ptr<rrt_star::Node> auxiliar_node = best_global_node;

    // Update the yaw to follow the path
    while (auxiliar_node->parent) {
        double dx = auxiliar_node->point.x() - auxiliar_node->parent->point.x();
        double dy = auxiliar_node->point.y() - auxiliar_node->parent->point.y();
        auxiliar_node->point[3] = std::atan2(dy, dx);

        auxiliar_node = auxiliar_node->parent;
    }

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

double AEPlanner::distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
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

/*bool AEPlanner::callbackReevaluate(cache_nodes::Reevaluate::Request& req, cache_nodes::Reevaluate::Response& res) {
    ROS_DEBUG_STREAM("Reevaluation Start!");

    for (std::vector<geometry_msgs::Point>::iterator iter = req.point.begin(); iter != req.point.end(); ++iter) {
        Eigen::Vector4d pos(iter->x, iter->y, iter->z, 0);
        std::shared_ptr<rrt_star::Node> node = std::make_shared<rrt_star::Node>(pos);
        eth_mav_msgs::EigenTrajectoryPoint traj_point;
        segment_evaluator.computeGainFromsampledYaw(node, num_yaw_samples, traj_point);
        res.gain.push_back(node->gain);
        res.yaw.push_back(node->point[3]);
    }

    ROS_DEBUG_STREAM("Reevaluation Finish!");

    return true;
}*/

void AEPlanner::callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[AEPlanner]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void AEPlanner::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[AEPlanner]: getting UavState diagnostics");
    uav_state = *msg;
    pose = {uav_state.pose.position.x, uav_state.pose.position.y, uav_state.pose.position.z, -3.0};
}

void AEPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[AEPlanner]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

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
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[AEPlanner]: waiting for command");
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO("[AEPlanner]: Total Iterations: %d", iteration_);
            break;
        }
        case STATE_PLANNING: {
            AEP();

            if (state_ == STATE_STOPPED) {
                break;
            }

            iteration_ += 1;

            current_waypoint_.position.x = next_best_node->point[0];
            current_waypoint_.position.y = next_best_node->point[1];
            current_waypoint_.position.z = next_best_node->point[2];
            current_waypoint_.heading = next_best_node->point[3];

            mrs_msgs::GetPathSrv srv_get_path;

            srv_get_path.request.path.header.frame_id = "uav1/" + frame_id;
            srv_get_path.request.path.header.stamp = ros::Time::now();
            srv_get_path.request.path.fly_now = false;
            srv_get_path.request.path.use_heading = true;

            mrs_msgs::Reference reference;

            while (next_best_node && next_best_node->parent) {
                reference.position.x = next_best_node->point[0];
                reference.position.y = next_best_node->point[1];
                reference.position.z = next_best_node->point[2];
                reference.heading = next_best_node->point[3];
                srv_get_path.request.path.points.push_back(reference);
                next_best_node = next_best_node->parent;
            }

            std::reverse(srv_get_path.request.path.points.begin(), srv_get_path.request.path.points.end());

            // Publish Point Reference
            //for (int i = 0; i < srv_get_path.request.path.points.size(); ++i) {
            //    pub_reference.publish(srv_get_path.request.path.points[i]);
            //}

            for (const auto& point : srv_get_path.request.path.points) {
                pub_reference.publish(point);
            }

            bool success = sc_trajectory_generation.call(srv_get_path);

            if (!success) {
                ROS_ERROR("[AEPlanner]: service call for trajectory failed");
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_get_path.response.success) {
                    ROS_ERROR("[AEPlanner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
                    changeState(STATE_STOPPED);
                    return;
                }
            }

            mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
            srv_trajectory_reference.request.trajectory = srv_get_path.response.trajectory;
            srv_trajectory_reference.request.trajectory.fly_now = true;

            bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

            if (!success_trajectory) {
                ROS_ERROR("[AEPlanner]: service call for trajectory reference failed");
                changeState(STATE_STOPPED);
                return;
            } else {
                if (!srv_trajectory_reference.response.success) {
                    ROS_ERROR("[AEPlanner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
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
                ROS_INFO("[AEPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;

                double dist = distance(current_waypoint_, current_pose);
                ROS_INFO("[AEPlanner]: Distance to waypoint: %.2f", dist);
                if (dist <= 0.8*step_size) {
                    changeState(STATE_PLANNING);
                }
            } else {
                ROS_INFO("[AEPlanner]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }

        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[AEPlanner]: waiting for command");
            }
            break;
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

void AEPlanner::visualize_edge(const std::shared_ptr<rrt_star::Node> node, const std::string& ns) {
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

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter_++;
    }
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
