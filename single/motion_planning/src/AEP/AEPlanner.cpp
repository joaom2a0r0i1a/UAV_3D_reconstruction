#include "motion_planning/AEP/AEPlanner.h"
#include "motion_planning/planner_helpers.h"

AEPlanner::AEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
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

    // RRT Tree
    param_loader.loadParam("local_planning/N_max", N_max);
    param_loader.loadParam("local_planning/N_termination", N_termination);
    param_loader.loadParam("local_planning/N_yaw_samples", num_yaw_samples);
    param_loader.loadParam("local_planning/radius", radius);
    param_loader.loadParam("local_planning/step_size", step_size);
    param_loader.loadParam("local_planning/min_edge_length", min_edge_length_, 0.2);
    param_loader.loadParam("local_planning/tolerance", tolerance);
    param_loader.loadParam("local_planning/g_zero", g_zero);
    param_loader.loadParam("local_planning/rrt_star", local_rrt_star, false);

    // RRT* Tree (global Planning)
    param_loader.loadParam("global_planning/N_min_nodes", N_min_nodes);
    param_loader.loadParam("global_planning/selection", global_selection, std::string("score"));

    param_loader.loadParam("evaluation/marginal_gain", marginal_gain, true);
    param_loader.loadParam("evaluation/compute", eval_compute, std::string("gpu"));
    param_loader.loadParam("evaluation/marginal_split", marginal_split, false);
    param_loader.loadParam("evaluation/objective", objective_, std::string("expdecay"));
    param_loader.loadParam("evaluation/benchmark", benchmark_mode, false);

    // Camera
    param_loader.loadParam("camera/h_fov", horizontal_fov);
    param_loader.loadParam("camera/width", resolution_x);
    param_loader.loadParam("camera/height", resolution_y);
    param_loader.loadParam("camera/min_distance", min_distance);
    param_loader.loadParam("camera/max_distance", max_distance);
    param_loader.loadParam("camera/pitch", camera_pitch_deg, 10.0);
    camera_pitch = camera_pitch_deg * M_PI / 180.0;

    // Planner
    param_loader.loadParam("path/uav_radius", uav_radius);
    param_loader.loadParam("path/collision_check_resolution", collision_check_resolution_, 0.1);
    param_loader.loadParam("path/waypoint_reach_distance", waypoint_reach_distance_, 0.5);
    param_loader.loadParam("path/optimistic_iterations", optimistic_iterations_, 5);
    param_loader.loadParam("path/recovery_enabled", recovery_enabled_, true);
    param_loader.loadParam("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    param_loader.loadParam("path/recovery_min_tree", recovery_min_tree_, 10);
    param_loader.loadParam("path/recovery_timeout", recovery_timeout_, 12.0);
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
    segment_evaluator.setObjective(objective_);

    // Setup Voxblox
    tsdf_map_ = voxblox_server_.getTsdfMapPtr();
    esdf_map_ = voxblox_server_.getEsdfMapPtr();
    segment_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
    segment_evaluator.setEsdfMap(esdf_map_);
            
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
    // Edges (not just nodes) must clear obstacles -> give the RRT* library a straight-segment collision test.
    RRTStar.setEdgeCollisionChecker(
        [this](const Eigen::Vector3d& a, const Eigen::Vector3d& b){ return isEdgeCollisionFree(a, b); });

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
    pub_gpu_debug = nh_private_.advertise<sensor_msgs::PointCloud2>("gpu_debug_map", 1, true);

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
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &AEPlanner::callbackControlManagerDiag, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &AEPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &AEPlanner::callbackStop, this);

    /* Service Clients */
    sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_private_, "trajectory_generation_out");
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &AEPlanner::timerMain, this);

    is_initialized = true;
}

double AEPlanner::getMapDistance(const Eigen::Vector3d& position) const { return planner_helpers::getMapDistance(voxblox_server_, position); }

bool AEPlanner::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const { return planner_helpers::isPathCollisionFree(voxblox_server_, path, uav_radius); }

// Sample the segment; require clearance >= uav_radius at each point. When optimistic_edges_ (first replans),
// unknown space is traversable (block only observed-and-close) to bootstrap away from spawn. Otherwise unknown
// counts as blocked (getMapDistance returns 0 for unobserved) so the tree can't route through unmapped pockets.
bool AEPlanner::isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const { return planner_helpers::isEdgeCollisionFree(voxblox_server_, from, to, uav_radius, collision_check_resolution_, optimistic_edges_); }

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
    if (benchmark_mode) {
        bench_ms_fused = bench_ms_split = bench_ms_mcpu = bench_ms_agpu = bench_ms_acpu = 0.0;
        bench_ms_v2 = bench_ms_v4 = 0.0;
        bench_v2_err_sum = bench_v2_err_max = 0.0; bench_nodes = 0;
    }

    goto_global_planning = false;

    localPlannerGPU();
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
        ROS_INFO("[AEPlanner]: Planning Path to Global Frontiers");
        globalPlanner(GlobalFrontiers, best_global_node);
        if (!backtrack) {
            next_best_node = best_global_node;
        } else {
            backtrack = false;
            goto_global_planning = false;
            return;
        }
        goto_global_planning = false;
    }

    if (benchmark_mode && bench_nodes > 0) {
        double n = (double)bench_nodes;
        ROS_INFO("\n=== AEP GAIN BENCHMARK (local+global, %d nodes) ===\n"
                 "marginal-gpu-fused : %9.3f ms | %7.4f ms/node\n"
                 "marginal-gpu-split : %9.3f ms | %7.4f ms/node\n"
                 "marginal-cpu-hash  : %9.3f ms | %7.4f ms/node\n"
                 "absolute-gpu       : %9.3f ms | %7.4f ms/node\n"
                 "absolute-cpu       : %9.3f ms | %7.4f ms/node\n"
                 "v2(gpu 1-parent)   : %9.3f ms | %7.4f ms/node\n"
                 "v4(gpu multi-anc)  : %9.3f ms | %7.4f ms/node\n"
                 "v2(gpu 1-parent) vs cpu-hash: mean err %.4f | max err %.4f\n"
                 "==================================================",
                 bench_nodes,
                 bench_ms_fused, bench_ms_fused / n, bench_ms_split, bench_ms_split / n,
                 bench_ms_mcpu, bench_ms_mcpu / n, bench_ms_agpu, bench_ms_agpu / n,
                 bench_ms_acpu, bench_ms_acpu / n, bench_ms_v2, bench_ms_v2 / n,
                 bench_ms_v4, bench_ms_v4 / n, bench_v2_err_sum / n, bench_v2_err_max);
    }
}

void AEPlanner::localPlannerGPU() {
    best_score_ = 0;
    rrt_star::Node* best_node = nullptr;

    int j = 1; // Total nodes processed
    auto tree_t0 = std::chrono::high_resolution_clock::now();

    ROS_INFO("[AEPlanner]: Start Expanding Local");

    // 1. SETUP ROOT
    std::unique_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_unique<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_unique<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_unique<rrt_star::Node>(pose);
    }

    RRTStar.clearKDTree();
    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));
    clearMarkers();

    // 2. MAP MANAGEMENT
    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    sensor_msgs::PointCloud2 debug_msg = segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_);
    pub_gpu_debug.publish(debug_msg);

    // 3. PHASE A: RE-EVALUATE PREVIOUS BEST BRANCH (BATCHED)
    if (best_branch.size() > 1) {
        std::vector<rrt_star::Node*> branch_candidates;

        bool isFirstIteration = true;
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) { isFirstIteration = false; continue; }

            const Eigen::Vector4d& node_position = best_branch[i]->point;

            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);

            auto new_node_best = std::make_unique<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;

            segment_evaluator.computeCost(new_node_best.get());
            rrt_star::Node* added_best = RRTStar.addKDTreeNode(std::move(new_node_best));

            branch_candidates.push_back(added_best);
        }

        if (!branch_candidates.empty()) {
            // Re-eval the re-added branch (a chain, so ancestors are ready) with the configured method.
            evaluateGains(branch_candidates);
            if (benchmark_mode) benchmarkGains(branch_candidates);
            for (rrt_star::Node* node : branch_candidates) {
                segment_evaluator.computeScore(node, lambda);
                if (node->score > best_score_) {
                    best_score_ = node->score;
                    best_node = node;
                }
            }
            // Re-added branch is now in the tree; it gets drawn by visualize_tree() in the expansion loop below.
            j += branch_candidates.size();
        }
    }

    best_branch.clear();

    // 4. PHASE B: RRT EXPANSION LOOP (BATCHED)
    const int BATCH_SIZE = 2 * N_max;   // scale the batch with the node budget (add only up to the limit)
    collision_id_counter_ = 0;
    ros::WallTime plan_start_ = ros::WallTime::now();   // bound the tree build so AEP() can never spin (single-threaded timer)

    while (j < N_max || best_score_ <= g_zero) {

        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int current_batch_cap = std::min(BATCH_SIZE, nodes_needed);
        if (current_batch_cap <= 0) break;

        // In-planner backtrack, wall-clock bounded: boxed-in (tree tiny past a short deadline) or timed-out (hard deadline).
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in  = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) executed_path_.pop_back();   // remove the current node we're on
            if (!executed_path_.empty()) {
                cacheHighGainNodes();               // don't lose frontier candidates before retreating
                retreating_ = true;                 // timerMain retreats to the previous node (new back())
                ROS_WARN("[AEPlanner]: Backtracking (%s, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", j, executed_path_.size());
                best_branch.clear();
                return;
            } else {
                rotate();                          // back at the start -> observe more
                plan_start_ = ros::WallTime::now();    // reset the deadline after rotating
                collision_id_counter_ = 0;
            }
        }

        std::vector<rrt_star::Node*> batch_nodes;
        batch_nodes.reserve(current_batch_cap);

        for (int k = 0; k < current_batch_cap && j <= N_termination; ++k) {
            // Boxed-in guard: when every sample collides, k-- keeps this inner loop spinning and the
            // outer WallTime check above is never reached. Bail to the outer loop so the backtrack fires.
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - plan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && j < recovery_min_tree_) || e > recovery_timeout_) break;
            }
            Eigen::Vector3d rand_point;
            RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
            rand_point += root_ptr->point.head(3);

            rrt_star::Node* nearest_node = nullptr;
            RRTStar.findNearestKD(rand_point, nearest_node);

            std::unique_ptr<rrt_star::Node> new_node;
            RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node, false, min_edge_length_);

            if (new_node->point[0] > max_x || new_node->point[0] < min_x ||
                new_node->point[1] < min_y || new_node->point[1] > max_y ||
                new_node->point[2] < min_z || new_node->point[2] > max_z) {
                k--; continue;
            }

            std::vector<rrt_star::Node*> segment = {new_node.get()};
            if (!isPathCollisionFree(segment)) {
                collision_id_counter_++;
                k--; continue;
            }
            // Endpoints are free; also require the whole PARENT EDGE to be clear (nodes-only misses walls between them).
            if (!isEdgeCollisionFree(nearest_node->point.head<3>(), new_node->point.head<3>())) {
                collision_id_counter_++;
                k--; continue;
            }

            new_node->gain = 0.0;
            new_node->score = 0.0;

            // RRT keeps the nearest steer parent; RRT* rewires (safe: batched eval + propagateCost run after the build).
            rrt_star::Node* added_node;
            if (local_rrt_star) {
                std::vector<rrt_star::Node*> nearby;
                RRTStar.findNearbyKD(new_node.get(), radius, nearby);
                if (nearby.empty()) nearby.push_back(nearest_node);   // guarantee a valid parent
                RRTStar.chooseParent(new_node.get(), nearby);         // sets parent + cost (edge-checked)
                if (!new_node->parent) { collision_id_counter_++; k--; continue; }  // every candidate edge blocked
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
                RRTStar.rewire(added_node, nearby, radius);           // updates cost on any re-parented node
            } else {
                segment_evaluator.computeCost(new_node.get());        // cost = parent->cost + edge
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
            }

            batch_nodes.push_back(added_node);
            j++;
        }

        if (batch_nodes.empty()) continue;

        // score_nodes = rescore set, gain_nodes = re-raycast set. RRT: both are the new batch; RRT* rewires so rescore the whole tree.
        std::vector<rrt_star::Node*> score_nodes = batch_nodes;
        std::vector<rrt_star::Node*> gain_nodes  = batch_nodes;
        if (local_rrt_star) {
            score_nodes = collectTreeNodes();
            sortByDepth(score_nodes);                            // ancestors first (cumulative score)
            best_score_ = 0.0; best_node = nullptr;
            if (marginal_gain) gain_nodes = score_nodes;
        }

        evaluateGains(gain_nodes);
        if (benchmark_mode) benchmarkGains(gain_nodes);

        for (rrt_star::Node* node : score_nodes) {
            segment_evaluator.computeScore(node, lambda);
            if (node->score > best_score_) {
                best_score_ = node->score;
                best_node = node;
            }
        }
        visualize_tree(collectTreeNodes(), ns);

        if (j >= N_termination) {
             logTreeNodes();
             cacheHighGainNodes();   // cache the final tree's frontier candidates before clearing
             ROS_INFO("[AEPlanner]: Going to Global Planning");
             RRTStar.clearKDTree();
             best_branch.clear();
             clearMarkers();
             goto_global_planning = true;
             return;
        }
    }

    logTreeNodes();
    cacheHighGainNodes();   // cache frontier candidates once over the final tree (gains are settled)

    // FINALIZE
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    if (!benchmark_mode) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        if (best_node)
            ROS_INFO("[AEPlanner]: Chosen node [%.2f, %.2f, %.2f] score=%.3f gain=%.3f | local tree computed in %.1f ms",
                     best_node->point[0], best_node->point[1], best_node->point[2], best_node->score, best_node->gain, tree_ms);
        else
            ROS_INFO("[AEPlanner]: No node chosen | local tree computed in %.1f ms", tree_ms);
    }

    next_best_node = best_branch[1].get();
}

void AEPlanner::localPlanner() {
    best_score_ = 0;
    rrt_star::Node* best_node = nullptr;

    ROS_INFO("[AEPlanner]: Start Expanding Local");

    std::unique_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_unique<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_unique<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_unique<rrt_star::Node>(pose);
    }

    root->depth_buffer.clear();

    // Root sees nothing: an empty (max-range) buffer sized to the current voxel resolution.
    root->depth_buffer.resize(segment_evaluator.depthImagePixels(), -1.0f);

    RRTStar.clearKDTree();
    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));
    clearMarkers();

    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    float camera_pitch_rad = (float)camera_pitch;

    auto get_parent_cam_state = [&](const Eigen::Vector3d& body_pos, float yaw, float pitch_rad) -> std::pair<std::vector<float>, Eigen::Vector3d> {
    
        // 1. Precompute Trig (Ensure pitch is RADIANS!)
        float cos_y = cosf(yaw);
        float sin_y = sinf(yaw);
        float cos_p = cosf(pitch_rad); 
        float sin_p = sinf(pitch_rad);

        std::vector<float> R_flat(9);

        // Camera basis (Right/Down/Front) rotated through the Body->World chain, matching the GPU kernel.

        // R0: Camera Right Axis (x_cam=1) in World
        // Kernel Formula: dir_x = sin_y, dir_y = -cos_y, dir_z = 0
        R_flat[0] = sin_y;
        R_flat[1] = -cos_y;
        R_flat[2] = 0.0f;

        // R1: Camera Down Axis (y_cam=1) in World
        // Kernel Formula: dir_x = -sin_p*cos_y, dir_y = -sin_p*sin_y, dir_z = -cos_p
        R_flat[3] = -sin_p * cos_y;
        R_flat[4] = -sin_p * sin_y;
        R_flat[5] = -cos_p;

        // Front axis (z_cam) in world: dir = (cos_p*cos_y, cos_p*sin_y, -sin_p); pitch>0 looks down.
        R_flat[6] = cos_p * cos_y;
        R_flat[7] = cos_p * sin_y;
        R_flat[8] = -sin_p;

        return {R_flat, body_pos};
    };

    sensor_msgs::PointCloud2 debug_msg = segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_);
    ROS_WARN("GPU Debug Map Published! Check Rviz topic /gpu_debug_map");
    pub_gpu_debug.publish(debug_msg);

    visualize_node(root_ptr->point, ns);
    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    while (j < N_max || best_score_ <= g_zero) {
        // Backtrack
        if (collision_id_counter_ > 10000 * j) {
            ROS_INFO("[AEPlanner]: Backtrack Rotation");
            rotate();
            collision_id_counter_ = 0;
        }

        // Add previous best branch
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue; // Skip first iteration (root)
            }

            const Eigen::Vector4d& node_position = best_branch[i]->point;

            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);

            std::unique_ptr<rrt_star::Node> new_node_best = std::make_unique<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;
            visualize_node(new_node_best->point, ns);

            auto [parent_R, parent_cam_pos] = get_parent_cam_state(
                new_node_best->parent->point.head(3), 
                (float)new_node_best->parent->point[3], camera_pitch_rad /*pitch*/
            );

            std::pair<double, double> result = segment_evaluator.computeSingleParentMarginalGainGPU(
                new_node_best->point.x(), new_node_best->point.y(), new_node_best->point.z(),
                parent_cam_pos, new_node_best->parent->point[3], parent_R, new_node_best->parent->depth_buffer, new_node_best->depth_buffer);

            new_node_best->gain = result.first;
            new_node_best->point[3] = result.second;

            segment_evaluator.computeCost(new_node_best.get());
            segment_evaluator.computeScore(new_node_best.get(), lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best.get();
            }

            rrt_star::Node* added_node_best = RRTStar.addKDTreeNode(std::move(new_node_best));
            visualize_edge(added_node_best, ns);

            ++j;
        }

        if (j >= N_max && best_score_ > g_zero) {
            break;
        }
    
        best_branch.clear();

        Eigen::Vector3d rand_point;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
        rand_point += root_ptr->point.head(3);

        rrt_star::Node* nearest_node = nullptr;
        RRTStar.findNearestKD(rand_point, nearest_node);

        std::unique_ptr<rrt_star::Node> new_node;
        RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node, false, min_edge_length_);

        if (new_node->point[0] > max_x || new_node->point[0] < min_x || new_node->point[1] < min_y || new_node->point[1] > max_y || new_node->point[2] < min_z || new_node->point[2] > max_z) {
            continue;
        }

        // Collision Check
        std::vector<rrt_star::Node*> trajectory_segment;
        trajectory_segment.push_back(new_node.get());

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

        auto [parent_R, parent_cam_pos] = get_parent_cam_state(
            new_node->parent->point.head(3), 
            (float)new_node->parent->point[3], camera_pitch_rad
        );
        
        // 3. Compute CPU Buffer (using the internal map + passed R)
        std::vector<float> cpu_buffer = segment_evaluator.computeDepthBufferCPU(
            new_node->parent->point, // Pass Vector4d directly
            flat_map_,
            parent_R
        );

        // 4. Compare with GPU Buffer
        const std::vector<float>& gpu_buffer = new_node->parent->depth_buffer;

        if (new_node->parent->depth_buffer.size() > 0 && new_node->parent->depth_buffer[0] >= 0.0f) {
            if (cpu_buffer.size() == gpu_buffer.size()) {
                int mismatch_count = 0;
                float max_error = 0.0f;
                double total_error = 0.0;
                const float TOLERANCE = 0.05f; // 10cm tolerance

                // Iterate pixels
                for (size_t k = 0; k < cpu_buffer.size(); ++k) {
                    float val_cpu = cpu_buffer[k];
                    float val_gpu = gpu_buffer[k];
                    float diff = std::abs(val_cpu - val_gpu);

                    total_error += diff;
                    if (diff > max_error) max_error = diff;

                    if (diff > TOLERANCE) {
                        mismatch_count++;
                        // Print first 5 errors to diagnose specific rays
                        if (mismatch_count <= 10) {
                            int w = ceil((2.0f * 5.0f * tanf(1.51844f * 0.5f)) / 0.2f); // Recalc width for (u,v) debug
                            int u = k % w;
                            int v = k / w;
                            ROS_ERROR("[Buffer Fail] Ray %lu (u=%d, v=%d) | GPU: %.2f | CPU: %.2f | Diff: %.2f", 
                                    k, u, v, val_gpu, val_cpu, diff);
                        }
                    }
                }

                if (mismatch_count > 0) {
                    ROS_ERROR("CRITICAL: Depth Buffer Mismatch! Failures: %d / %lu (%.1f%%) | Max Error: %.2fm | Avg Error: %.2fm", 
                            mismatch_count, cpu_buffer.size(), 
                            (100.0f * mismatch_count / cpu_buffer.size()), 
                            max_error, (total_error / cpu_buffer.size()));

                    // --- TRANSFORM DEBUGGER ---
                    // Print the exact math used by CPU so we can verify GPU inputs
                    ROS_ERROR("--- DEBUGGING TRANSFORMS (CPU INPUTS) ---");
                    ROS_ERROR("Parent Position: [%.4f, %.4f, %.4f]", 
                            parent_cam_pos.x(), parent_cam_pos.y(), parent_cam_pos.z());
                    
                    ROS_ERROR("Rotation Matrix (Basis Vectors):");
                    ROS_ERROR("R0 (Right):   [%.4f, %.4f, %.4f]", parent_R[0], parent_R[1], parent_R[2]);
                    ROS_ERROR("R1 (Down):    [%.4f, %.4f, %.4f]", parent_R[3], parent_R[4], parent_R[5]);
                    ROS_ERROR("R2 (Forward): [%.4f, %.4f, %.4f]", parent_R[6], parent_R[7], parent_R[8]);
                    ROS_ERROR("-----------------------------------------");

                } else {
                    ROS_INFO("SUCCESS: GPU and CPU Depth Buffers MATCH. Size: %lu pixels. (Max Diff: %.2fm)", 
                            cpu_buffer.size(), max_error);
                }
            } else {
                ROS_ERROR("CRITICAL: Buffer Size Mismatch! CPU: %lu vs GPU: %lu", 
                        cpu_buffer.size(), gpu_buffer.size());
            }
        }


        // --- RUN CPU ---
        // --- RUN CPU ---
        trajectory_point = new_node->point;

        // 0. Prepare Separate Output Buffers
        std::vector<float> depth_buf_v2 = new_node->depth_buffer;
        std::vector<float> depth_buf_v4 = new_node->depth_buffer;

        // --- DECLARE RESULTS OUTSIDE THE TRACKING BLOCK ---
        std::pair<double, double> res_cpu = {0.0, 0.0};
        std::pair<double, double> res_cpu_hash = {0.0, 0.0};
        std::pair<double, double> res_gpu_marg_v2 = {0.0, 0.0};
        std::pair<double, double> res_gpu_marg_v4 = {0.0, 0.0};

        // --- 0. STATIC TRACKERS ---
        static int nodes_evaluated = 0;
        const int NODE_LIMIT = 5000;
        
        // Error trackers
        static double sum_abs_error_v2 = 0.0;
        static double sum_sq_error_v2 = 0.0;
        static double sum_raw_bias_v2 = 0.0;
        static double max_abs_error_v2 = 0.0;
        static double sum_gt_gain = 0.0;
        static double sum_abs_error_abs_method = 0.0; // To compare against naive Absolute
        
        // Timing trackers
        static double sum_time_hash = 0.0;
        static double sum_time_v2 = 0.0;

        // 1. Always run the evaluations

        auto start_cpu = std::chrono::high_resolution_clock::now();
        res_cpu = segment_evaluator.computeGainCPU_FlatMap(flat_map_, trajectory_point);
        auto end_cpu = std::chrono::high_resolution_clock::now();

        auto start_cpu_hash = std::chrono::high_resolution_clock::now();
        if (new_node->parent && new_node->parent->parent) {
            segment_evaluator.populateParentHistory(flat_map_, new_node->parent);
        }
        res_cpu_hash = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, new_node.get());
        auto end_cpu_hash = std::chrono::high_resolution_clock::now();

        auto start_gpu_marg_v2 = std::chrono::high_resolution_clock::now();
        res_gpu_marg_v2 = segment_evaluator.computeSingleParentMarginalGainGPU(
            new_node->point.x(), new_node->point.y(), new_node->point.z(),
            parent_cam_pos, new_node->parent->point[3], parent_R, 
            new_node->parent->depth_buffer, depth_buf_v2); 
        auto end_gpu_marg_v2 = std::chrono::high_resolution_clock::now();

        // --- Build the full ancestor chain for multi-ancestor marginal gain (v3) ---
        std::vector<Eigen::Vector3d> anc_positions;
        std::vector<double>          anc_yaws;
        std::vector<float>           anc_R_flat;     // 9 floats per ancestor
        std::vector<float>           anc_depth_flat; // p_width*p_height floats per ancestor

        // Per-ancestor depth-buffer size, taken from any evaluated ancestor.
        size_t per_anc = 0;
        for (auto* a = new_node->parent; a != nullptr; a = a->parent) {
            if (!a->depth_buffer.empty()) { per_anc = a->depth_buffer.size(); break; }
        }

        for (auto* a = new_node->parent; a != nullptr; a = a->parent) {
            auto [anc_R, anc_cam_pos] = get_parent_cam_state(
                a->point.head(3), (float)a->point[3], camera_pitch_rad);
            anc_positions.push_back(anc_cam_pos);
            anc_yaws.push_back(a->point[3]);
            anc_R_flat.insert(anc_R_flat.end(), anc_R.begin(), anc_R.end());
            if (!a->depth_buffer.empty()) {
                anc_depth_flat.insert(anc_depth_flat.end(),
                                      a->depth_buffer.begin(), a->depth_buffer.end());
            } else if (per_anc > 0) {
                // Root / unevaluated ancestor: pad with -1.0f sentinel (unknown)
                // so the flattened buffer stays index-aligned with the pose arrays.
                anc_depth_flat.insert(anc_depth_flat.end(), per_anc, -1.0f);
            }
        }

        // v4: same ancestor chain as v3, but the marcher traverses skip spans.
        auto start_gpu_marg_v4 = std::chrono::high_resolution_clock::now();
        res_gpu_marg_v4 = segment_evaluator.computeMultiAncestorMarginalGainGPU(
            new_node->point.x(), new_node->point.y(), new_node->point.z(),
            anc_positions, anc_yaws, anc_R_flat,
            anc_depth_flat, depth_buf_v4);
        auto end_gpu_marg_v4 = std::chrono::high_resolution_clock::now();

        // 1.5 Physical sanity check (marginal gain <= absolute gain)
        const double EPSILON = 1e-3; 
        double absolute_baseline = res_cpu.first;

        // Helper lambda to log anomalies to both ROS and a file
        auto log_anomaly = [&](const std::string& method_name, double marg_gain) {
            double overshoot = marg_gain - absolute_baseline;
            
            // Print to ROS console
            ROS_ERROR("CRITICAL ANOMALY: %s marginal gain (%.4f) exceeded Absolute gain (%.4f) by %.4f!", 
                      method_name.c_str(), marg_gain, absolute_baseline, overshoot);
            
            // Append to log file
            std::string anomaly_file = "saferail_triggers.log";
            std::ofstream log_file(anomaly_file, std::ios::app);
            if (log_file.is_open()) {
                log_file << "Node_ID: " << nodes_evaluated 
                         << " | Method: " << method_name 
                         << " | Marg_Gain: " << marg_gain 
                         << " | Abs_Gain: " << absolute_baseline 
                         << " | Overshoot: +" << overshoot << "\n";
                log_file.close();
            } else {
                ROS_ERROR("Failed to open %s to record the anomaly.", anomaly_file.c_str());
            }
        };

        // Trigger the check for all methods
        if (res_cpu_hash.first > absolute_baseline + EPSILON) {
            log_anomaly("CPU_Hash_Map", res_cpu_hash.first);
        }
        if (res_gpu_marg_v2.first > absolute_baseline + EPSILON) {
            log_anomaly("GPU_Marg_V2", res_gpu_marg_v2.first);
        }
        if (res_gpu_marg_v4.first > absolute_baseline + EPSILON) {
            log_anomaly("GPU_Marg_V4", res_gpu_marg_v4.first);
        }

        // 2. Only log and accumulate under the benchmark limit
        if (nodes_evaluated < NODE_LIMIT) {
            double ms_cpu       = std::chrono::duration<double, std::milli>(end_cpu - start_cpu).count();
            double ms_cpu_hash  = std::chrono::duration<double, std::milli>(end_cpu_hash - start_cpu_hash).count();
            double ms_gpu_marg_v2 = std::chrono::duration<double, std::milli>(end_gpu_marg_v2 - start_gpu_marg_v2).count();
            double ms_gpu_marg_v4 = std::chrono::duration<double, std::milli>(end_gpu_marg_v4 - start_gpu_marg_v4).count();

            ROS_INFO(
                "\n--- GAIN EVALUATION TRIPLE THREAT ---\n"
                "1. CPU (Abs):  Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms\n"
                "2. CPU (Hash): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "3. GPU (Marg_v2): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "4. GPU (Marg_v4): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "---------------------------------------",
                res_cpu.first,      res_cpu.second,      ms_cpu,
                res_cpu_hash.first, res_cpu_hash.second, ms_cpu_hash, (ms_cpu / (ms_cpu_hash + 1e-5)),
                res_gpu_marg_v2.first, res_gpu_marg_v2.second, ms_gpu_marg_v2, (ms_cpu / (ms_gpu_marg_v2 + 1e-5)),
                res_gpu_marg_v4.first, res_gpu_marg_v4.second, ms_gpu_marg_v4, (ms_cpu / (ms_gpu_marg_v4 + 1e-5))
            );

            // Accumulate Execution Times
            sum_time_hash += ms_cpu_hash;
            sum_time_v2 += ms_gpu_marg_v2;

            // Extract Gains
            double gt_gain = res_cpu_hash.first;
            double abs_gain = res_cpu.first;
            double v2_gain = res_gpu_marg_v2.first;

            // Calculate Errors
            double raw_diff_v2 = v2_gain - gt_gain; 
            double abs_error_v2 = std::abs(raw_diff_v2);
            double abs_error_abs = std::abs(abs_gain - gt_gain); 

            // Accumulate Errors
            sum_abs_error_v2 += abs_error_v2;
            sum_sq_error_v2 += (abs_error_v2 * abs_error_v2);
            sum_raw_bias_v2 += raw_diff_v2;
            sum_gt_gain += gt_gain;
            sum_abs_error_abs_method += abs_error_abs;

            if (abs_error_v2 > max_abs_error_v2) {
                max_abs_error_v2 = abs_error_v2;
            }

            nodes_evaluated++;

            // Trigger the final report and file dump exactly once
            if (nodes_evaluated == NODE_LIMIT) {
                // Math Computations
                double mae_v2 = sum_abs_error_v2 / NODE_LIMIT;
                double mae_abs = sum_abs_error_abs_method / NODE_LIMIT;
                double improvement = mae_abs - mae_v2; 

                double rmse_v2 = std::sqrt(sum_sq_error_v2 / NODE_LIMIT);
                double mean_bias_v2 = sum_raw_bias_v2 / NODE_LIMIT;
                double mean_gt = sum_gt_gain / NODE_LIMIT;
                double avg_error_pct = (mae_v2 / (mean_gt + 1e-5)) * 100.0;

                double avg_time_hash = sum_time_hash / NODE_LIMIT;
                double avg_time_v2 = sum_time_v2 / NODE_LIMIT;
                double speedup = avg_time_hash / (avg_time_v2 + 1e-5);

                // --- 1. PRINT TO CONSOLE ---
                ROS_INFO("\n==================================================");
                ROS_INFO(" FINAL GPU(v2) vs CPU(Hash) ERROR REPORT");
                ROS_INFO(" Total Nodes Evaluated : %d", NODE_LIMIT);
                ROS_INFO(" Average Ground Truth  : %.4f voxels", mean_gt);
                ROS_INFO(" ------------------------------------------------");
                ROS_INFO(" Error (CPU Absolute)  : %.4f voxels", mae_abs);
                ROS_INFO(" Error (GPU V2)        : %.4f voxels (%.2f%%)", mae_v2, avg_error_pct);
                ROS_INFO(" V2 Improvement        : %.4f voxels closer to GT", improvement);
                ROS_INFO(" ------------------------------------------------");
                ROS_INFO(" Root Mean Sq (RMSE)   : %.4f voxels", rmse_v2);
                ROS_INFO(" Max Absolute Error    : %.4f voxels", max_abs_error_v2);
                ROS_INFO(" Directional Bias      : %+.4f", mean_bias_v2);
                ROS_INFO(" ------------------------------------------------");
                ROS_INFO(" Avg Hash Time         : %.4f ms", avg_time_hash);
                ROS_INFO(" Avg V2 Time           : %.4f ms", avg_time_v2);
                ROS_INFO(" Computational Speedup : %.2fx", speedup);
                ROS_INFO("==================================================\n");

                // --- 2. LOG TO CSV FILE ---
                std::string file_name = "gain_metrics_log.csv";
                std::ifstream fcheck(file_name);
                bool write_header = !fcheck.good(); 
                fcheck.close();

                std::ofstream log_file(file_name, std::ios::app);
                if (log_file.is_open()) {
                    if (write_header) {
                        log_file << "Nodes,Avg_GT_Gain,MAE_Absolute,MAE_V2,Improvement,RMSE_V2,Max_Error_V2,Bias_V2,Avg_Time_Hash_ms,Avg_Time_V2_ms,Speedup\n";
                    }
                    log_file << NODE_LIMIT << ","
                             << mean_gt << ","
                             << mae_abs << ","
                             << mae_v2 << ","
                             << improvement << ","
                             << rmse_v2 << ","
                             << max_abs_error_v2 << ","
                             << mean_bias_v2 << ","
                             << avg_time_hash << ","
                             << avg_time_v2 << ","
                             << speedup << "\n";
                    log_file.close();
                    ROS_INFO("Successfully appended results to %s", file_name.c_str());
                } else {
                    ROS_ERROR("Failed to open %s for logging.", file_name.c_str());
                }
            }
        }

        // Now res_gpu_marg_v2 is always in scope and has the correct calculated value!
        std::pair<double, double> result = res_gpu_marg_v2;

        /*trajectory_point.position_W = new_node->point.head(3);
        trajectory_point.setFromYaw(new_node->point[3]);

        std::pair<double, double> result = segment_evaluator.computeGainOptimizedRaycasting(trajectory_point);*/

        new_node->gain = result.first;
        new_node->point[3] = result.second;
        new_node->depth_buffer = depth_buf_v2;

        segment_evaluator.computeCost(new_node.get());
        segment_evaluator.computeScore(new_node.get(), lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node.get();
        }


        rrt_star::Node* added_node = RRTStar.addKDTreeNode(std::move(new_node));
        visualize_edge(added_node, ns);

        if (added_node->gain > g_zero) {
            cacheNode(added_node, added_node->gain, added_node->point[3]);   // legacy path: gain is absolute
        }

        if (j > N_termination) {
            ROS_INFO("[AEPlanner]: Going to Global Planning");
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

    next_best_node = best_branch[1].get();
}

std::vector<float> AEPlanner::parentCamRows(float yaw) { return segment_evaluator.parentCamRows(yaw); }









void AEPlanner::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    // Shared gain pipeline (core/rrt_construction). AEP always optimizes yaw and tracks the own-view
    // absolute gain/yaw alongside the marginal (for global-planner scoring).
    GainEvaluator::GainConfig cfg{marginal_gain, /*optimize_yaw=*/true, eval_compute, marginal_split, /*track_absolute=*/true};
    float marg_ms = 0.0f, abs_ms = 0.0f;
    segment_evaluator.evaluateGains(nodes, flat_map_, cfg, marg_ms, abs_ms);
    bench_kernel_ms_ = marg_ms;   // device (CUDA-event) ms of the marginal batch
}

double AEPlanner::computeV2SingleParent(rrt_star::Node* node) {
    const int per = segment_evaluator.depthImagePixels();
    rrt_star::Node* p = node->parent;
    double p_yaw = p ? p->point[3] : 0.0;
    std::vector<float> R = parentCamRows((float)p_yaw);
    Eigen::Vector3d p_pos = p ? p->point.head(3) : node->point.head(3);
    std::vector<float> p_depth;
    if (p && (int)p->depth_buffer.size() == per) p_depth = p->depth_buffer;
    else p_depth.assign((size_t)per, -1.0f);   // no parent view (root) -> absolute
    std::vector<float> out;
    auto r = segment_evaluator.computeSingleParentMarginalGainGPU(
        node->point.x(), node->point.y(), node->point.z(), p_pos, p_yaw, R, p_depth, out);
    return r.first;
}

// Per-node (non-batched) GPU multi-ancestor marginal gain; walks the full chain, comparable per-node to the batched methods.
double AEPlanner::computeV4MultiAncestor(rrt_star::Node* node, double* out_yaw) {
    const int per = segment_evaluator.depthImagePixels();
    std::vector<Eigen::Vector3d> anc_positions;
    std::vector<double>          anc_yaws;
    std::vector<float>           anc_R_flat;      // 9 floats per ancestor
    std::vector<float>           anc_depth_flat;  // 'per' floats per ancestor
    for (rrt_star::Node* a = node->parent; a != nullptr; a = a->parent) {
        anc_positions.push_back(a->point.head(3));
        anc_yaws.push_back(a->point[3]);
        std::vector<float> R = parentCamRows((float)a->point[3]);
        anc_R_flat.insert(anc_R_flat.end(), R.begin(), R.end());
        if ((int)a->depth_buffer.size() == per)
            anc_depth_flat.insert(anc_depth_flat.end(), a->depth_buffer.begin(), a->depth_buffer.end());
        else
            anc_depth_flat.insert(anc_depth_flat.end(), (size_t)per, -1.0f);  // root/unevaluated -> unknown
    }
    std::vector<float> out;
    auto r = segment_evaluator.computeMultiAncestorMarginalGainGPU(
        node->point.x(), node->point.y(), node->point.z(),
        anc_positions, anc_yaws, anc_R_flat, anc_depth_flat, out);
    if (out_yaw) *out_yaw = r.second;   // argmax yaw (free-yaw optimum); node->gain/yaw/buffer left untouched
    return r.first;
}

// Order nodes shallow-first so cumulative scoring sees each parent before its children.
void AEPlanner::sortByDepth(std::vector<rrt_star::Node*>& nodes) { segment_evaluator.sortByDepth(nodes); }

std::vector<rrt_star::Node*> AEPlanner::collectTreeNodes() { return planner_helpers::collectTreeNodes(RRTStar); }

// Telescoped path-union root->node: path_sum[n] = path_sum[parent] + (use_marginal ? gain : absolute_gain). Local scratch for global scoring.
std::unordered_map<rrt_star::Node*, double> AEPlanner::pathUnion(rrt_star::Node* root_ptr, bool use_marginal) {
    std::vector<rrt_star::Node*> tree = collectTreeNodes();
    sortByDepth(tree);   // parents before children so each path_sum[parent] is ready
    std::unordered_map<rrt_star::Node*, double> path_sum{{root_ptr, 0.0}};
    for (rrt_star::Node* n : tree)
        path_sum[n] = path_sum[n->parent] + (use_marginal ? n->gain : n->absolute_gain);
    return path_sum;
}



void AEPlanner::cacheHighGainNodes() {
    // Cache a frontier when its OWN view still sees new space (absolute_gain > g_zero), else marginal mode hides frontiers and never terminates.
    for (const auto& up : RRTStar.getNodes()) {
        rrt_star::Node* n = up.get();
        if (!n->parent) continue;                                 // skip root
        // Cache the ABSOLUTE gain AND the yaw that achieves it: in marginal mode point[3] is the marginal
        // yaw, which would aim the frontier away from the very space that qualified it.
        if (n->absolute_gain > g_zero) cacheNode(n, n->absolute_gain, n->absolute_yaw);
    }
}

// Per-node score dump over the final tree (once), so multi-batch runs don't re-log each batch.
void AEPlanner::logTreeNodes() { if (benchmark_mode) return; planner_helpers::logTreeNodes(RRTStar, lambda); }

void AEPlanner::benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();

    // Save real gain/yaw so the planner is undisturbed. Depth buffers are left as the FINAL (split)
    // timed pass sets them -- the next real batch's ancestors rely on those valid marginal buffers.
    std::vector<double> save_gain(n), save_yaw(n);
    for (size_t i = 0; i < n; ++i) { save_gain[i] = nodes[i]->gain; save_yaw[i] = nodes[i]->point[3]; }
    const bool save_marg = marginal_gain; const std::string save_comp = eval_compute; const bool save_split = marginal_split;

    // Single-shot host wall-clock of one evaluateGains(nodes) under a given config. One sample per
    // batch is enough -- the distribution comes from the many batches (each many nodes) over a run.
    auto timed = [&](bool marg, const std::string& comp, bool split) -> double {
        marginal_gain = marg; eval_compute = comp; marginal_split = split;
        auto t0 = std::chrono::high_resolution_clock::now();
        evaluateGains(nodes);
        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    // ---- COLD-FAIR ORDER: single-node GPU methods FIRST (so they never inherit a candidate depth
    //      buffer a batched pass just rendered), THEN the batched kernels (fused, split last). ----
    auto v2_0 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < n; ++i) computeV2SingleParent(nodes[i]);
    auto v2_1 = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < n; ++i) computeV4MultiAncestor(nodes[i]);
    auto v4_1 = std::chrono::high_resolution_clock::now();
    double t_v2 = std::chrono::duration<double, std::milli>(v2_1 - v2_0).count();
    double t_v4 = std::chrono::duration<double, std::milli>(v4_1 - v2_1).count();

    // CPU references for the gain study (absolute + marginal hash) and absolute-GPU.
    double t_acpu = timed(false, "cpu", false); std::vector<double> g_acpu(n); for (size_t i = 0; i < n; ++i) g_acpu[i] = nodes[i]->gain;
    double t_mcpu = timed(true,  "cpu", false); // CPU-hash marginal (value recomputed cleanly below)
    double t_agpu = timed(false, "gpu", false); std::vector<double> g_agpu(n); for (size_t i = 0; i < n; ++i) g_agpu[i] = nodes[i]->gain;

    // Batched kernels LAST. Capture BOTH host time and the CUDA-event DEVICE time (bench_kernel_ms_,
    // accumulated inside evaluateMarginalGainsBatched). Split runs last -> leaves correct depth buffers.
    double t_fused_host = timed(true, "gpu", false); double t_fused_dev = bench_kernel_ms_;
    std::vector<double> g_fused(n); for (size_t i = 0; i < n; ++i) g_fused[i] = nodes[i]->gain;
    double t_split_host = timed(true, "gpu", true);  double t_split_dev = bench_kernel_ms_;
    std::vector<double> g_split(n); for (size_t i = 0; i < n; ++i) g_split[i] = nodes[i]->gain;


    bench_ms_fused += t_fused_host; bench_ms_split += t_split_host; bench_ms_mcpu += t_mcpu;
    bench_ms_agpu += t_agpu; bench_ms_acpu += t_acpu; bench_ms_v2 += t_v2; bench_ms_v4 += t_v4;
    bench_nodes += (int)n;

    // Append one timing row per batch. Header written only when the file is new so
    // that appends across planning iterations (and across runs) stay tidy.
    auto open_csv = [](const std::string& path, const std::string& header) -> std::ofstream {
        bool exists = std::ifstream(path).good();
        std::ofstream f(path, std::ios::app);
        if (!exists && f.is_open()) f << header << "\n";
        return f;
    };
    {
        std::ofstream tf = open_csv("benchmark_timing.csv",
            "Phase,Nodes,v2_ms,v4_ms,fused_host_ms,split_host_ms,fused_dev_ms,split_dev_ms,abs_gpu_ms,abs_cpu_ms,cpu_hash_ms");
        if (tf.is_open())
            tf << phase << "," << n << "," << t_v2 << "," << t_v4 << "," << t_fused_host << ","
               << t_split_host << "," << t_fused_dev << "," << t_split_dev << "," << t_agpu << ","
               << t_acpu << "," << t_mcpu << "\n";
    }
    std::ofstream gf = open_csv("benchmark_gains.csv",
        "Phase,NodeIdx,Ancestors,g_fused,g_split,g_abs_gpu,g_abs_cpu,hash_cpu1p,v2_gpu1p,v4_multi");

    // Restore gain/yaw + config (leave depth buffers as the split pass set them).
    marginal_gain = save_marg; eval_compute = save_comp; marginal_split = save_split;
    for (size_t i = 0; i < n; ++i) { nodes[i]->gain = save_gain[i]; nodes[i]->point[3] = save_yaw[i]; }

    // Per-node check: GPU single-parent (v2) vs CPU hash (truth) against the same parent view; logs ancestor count for the gain study.
    for (size_t i = 0; i < n; ++i) {
        rrt_star::Node* nd = nodes[i];
        int anc = 0;
        for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) ++anc;
        double v2 = computeV2SingleParent(nd);
        double v4 = computeV4MultiAncestor(nd);   // before hash's populateParentHistory
        double sg = nd->gain, sy = nd->point[3];   // hash overwrites gain/yaw; restore right after
        if (nd->parent && nd->parent->parent) segment_evaluator.populateParentHistory(flat_map_, nd->parent);
        double hash = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, nd).first;
        nd->gain = sg; nd->point[3] = sy;
        double err = std::fabs(v2 - hash);
        bench_v2_err_sum += err; if (err > bench_v2_err_max) bench_v2_err_max = err;
        if (gf.is_open())
            gf << phase << "," << i << "," << anc << "," << g_fused[i] << "," << g_split[i] << "," << g_agpu[i]
               << "," << g_acpu[i] << "," << hash << "," << v2 << "," << v4 << "\n";
    }
}

void AEPlanner::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node*& best_global_node) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[AEPlanner]: Terminate AEP");

        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    ROS_INFO("[AEPlanner]: Start Expanding Global");

    std::unique_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_unique<rrt_star::Node>(next_start);
    } else {
        root = std::make_unique<rrt_star::Node>(pose);
    }

    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));
    root_ptr->gain = 0.0;
    root_ptr->absolute_gain = 0.0;
    auto tree_t0 = std::chrono::high_resolution_clock::now();

    // Cache the map on the GPU (needed for the batched marginal-gain eval).
    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    std::vector<rrt_star::Node*> all_global_goals;
    std::vector<rrt_star::Node*> frontier_nodes;   // every node that reached a frontier (goal candidates)
    collision_id_counter_ = 0;
    int m = 0;
    const int GLOBAL_BATCH = 2 * N_min_nodes;   // scale the batch with the node budget (add only up to the limit)

    ros::WallTime gplan_start_ = ros::WallTime::now();   // bound the global tree build so globalPlanner() can't spin either

    // Build the frontier KD-tree once; getGlobalGoal() only queries it.
    goals_tree.clearKDTreePoints();
    goals_tree.initializeKDTreeWithPoints(GlobalFrontiers);

    while (m < N_min_nodes || all_global_goals.empty()) {
        const double gplan_elapsed = (ros::WallTime::now() - gplan_start_).toSec();
        const bool g_boxed = gplan_elapsed > recovery_boxed_deadline_ && m < recovery_min_tree_;
        const bool g_timed = gplan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (g_boxed || g_timed)) {
            if (!executed_path_.empty()) executed_path_.pop_back();   // remove the current node we're on
            if (!executed_path_.empty()) {
                retreating_ = true;
                backtrack = true;
                ROS_WARN("[AEPlanner]: Global backtracking (%s, tree=%d) -> executed node %zu",
                         g_boxed ? "boxed-in" : "timeout", m, executed_path_.size());
                return;
            } else {
                ROS_INFO("[AEPlanner]: Backtrack Rotation");
                rotate();
                gplan_start_ = ros::WallTime::now();
                collision_id_counter_ = 0;
            }
        }

        // Build a batch of RRT* nodes; flag the ones that reach a frontier.
        int cap = std::min(GLOBAL_BATCH, (m < N_min_nodes) ? (N_min_nodes - m) : GLOBAL_BATCH);
        std::vector<rrt_star::Node*> batch_new, batch_frontier;
        for (int b = 0; b < cap; ++b) {
            // Boxed-in guard (see localPlannerGPU): b-- on collisions traps this inner loop before the
            // outer WallTime check runs. Bail to the outer loop so the global backtrack fires.
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - gplan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && m < recovery_min_tree_) || e > recovery_timeout_) break;
            }
            Eigen::Vector3d rand_point_star;
            RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
            rand_point_star += root_ptr->point.head(3);

            rrt_star::Node* nearest_node_star = nullptr;
            RRTStar.findNearestKD(rand_point_star, nearest_node_star);

            std::unique_ptr<rrt_star::Node> new_node_star;
            RRTStar.steer_parent(nearest_node_star, rand_point_star, step_size, new_node_star, false, min_edge_length_);

            // Global planner must respect the bounded box too (mirror the local check ~L368). steer_parent can
            // push the node past the box, and computeSamplingDimensions samples a bounded_radius (box-diagonal)
            // sphere around root, so without this the global tree grows outside x/y and ABOVE max_z.
            if (new_node_star->point[0] > max_x || new_node_star->point[0] < min_x ||
                new_node_star->point[1] < min_y || new_node_star->point[1] > max_y ||
                new_node_star->point[2] < min_z || new_node_star->point[2] > max_z) {
                b--; continue;
            }

            std::vector<rrt_star::Node*> segment_star = {new_node_star.get()};
            if (!isPathCollisionFree(segment_star)) { b--; continue; }
            if (!isEdgeCollisionFree(nearest_node_star->point.head<3>(), new_node_star->point.head<3>())) { b--; continue; }

            std::vector<rrt_star::Node*> nearby_nodes_star;
            RRTStar.findNearbyKD(new_node_star.get(), radius, nearby_nodes_star);
            if (nearby_nodes_star.empty()) nearby_nodes_star.push_back(nearest_node_star);   // guarantee a valid (edge-checked) parent
            RRTStar.chooseParent(new_node_star.get(), nearby_nodes_star);
            if (!new_node_star->parent) { b--; continue; }   // every candidate edge blocked
            rrt_star::Node* added_node_star = RRTStar.addKDTreeNode(std::move(new_node_star));
            RRTStar.rewire(added_node_star, nearby_nodes_star, radius);

            batch_new.push_back(added_node_star);
            if (getGlobalGoal(GlobalFrontiers, added_node_star)) {
                frontier_nodes.push_back(added_node_star);
                batch_frontier.push_back(added_node_star);
            }
            ++m;
        }
        visualize_tree(collectTreeNodes(), ns);

        // Pick nodes to (re)evaluate: marginal = whole tree (rewire restaled ancestries); absolute+gpu = new batch;
        // absolute+cpu = new frontiers only. absolute_gain is own-view (set once at birth), so it's always valid at qualification.
        std::vector<rrt_star::Node*> gain_nodes;
        if (marginal_gain)              gain_nodes = collectTreeNodes();
        else if (eval_compute == "gpu") gain_nodes = batch_new;
        else                            gain_nodes = batch_frontier;

        evaluateGains(gain_nodes);   // sets node->gain (+ node->absolute_gain/absolute_yaw via fillAbsoluteGains)
        if (benchmark_mode) benchmarkGains(gain_nodes, "global");

        // Qualify frontiers by OWN-VIEW absolute gain (not the marginal path-sum), else informative frontiers over seen space get dropped and AEP never terminates.
        all_global_goals.clear();
        for (rrt_star::Node* f : frontier_nodes)
            if (f->absolute_gain >= 0.1) all_global_goals.push_back(f);
    }

    // Score the qualified goals by the path-union of gains (own-view absolute, or de-overlapped marginal), discounted by cost.
    {
        const bool use_marginal = marginal_gain;
        ROS_INFO("[AEPlanner]: Global scoring mode = %s (path-union * discount)", use_marginal ? "MARGINAL" : "ABSOLUTE");
        std::unordered_map<rrt_star::Node*, double> path_sum = pathUnion(root_ptr, use_marginal);
        for (rrt_star::Node* g : all_global_goals) {
            g->gain  = path_sum[g];
            g->score = (objective_ == "rate_L") ? (path_sum[g] / (g->cost < 0.1 ? 0.1 : g->cost))
                                                : (path_sum[g] * exp(-global_lambda * g->cost));
        }
    }

    if (!benchmark_mode)
        for (rrt_star::Node* g : all_global_goals)
            ROS_INFO("[Goal] gain=%.3f score=%.3f", g->gain, g->score);

    ROS_INFO("[AEPlanner]: Global Planner Ends");
    if (!benchmark_mode) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        ROS_INFO("[AEPlanner]: Global tree computed in %.1f ms", tree_ms);
    }
    getBestGlobalPath(all_global_goals, best_global_node);   // logs the chosen goal
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
            GlobalFrontiers.push_back(frontier);
        }
    }
}

bool AEPlanner::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node* node) {
    if (GlobalFrontiers.empty()) return false;

    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(node->point.head(3), nearest_goal);

    return (nearest_goal.size() > 0 && (nearest_goal - node->point.head(3)).norm() < tolerance);
}

void AEPlanner::getBestGlobalPath(const std::vector<rrt_star::Node*>& global_goals, rrt_star::Node*& best_global_node) {
    if (global_goals.size() == 0) {
        best_global_node = nullptr;
        return;
    }

    best_global_node = global_goals[0];

    // Pick the goal per the configured criterion: "cost" (nearest), "gain" (most info), or "score".
    for (int i = 1; i < (int)global_goals.size(); ++i) {
        if (global_selection == "cost") {
            if (global_goals[i]->cost < best_global_node->cost) best_global_node = global_goals[i];
        } else if (global_selection == "gain") {
            if (global_goals[i]->gain > best_global_node->gain) best_global_node = global_goals[i];
        } else {
            if (global_goals[i]->score > best_global_node->score) best_global_node = global_goals[i];
        }
    }


    ROS_INFO("[AEPlanner]: Chosen Goal: [%f, %f, %f]", best_global_node->point[0], best_global_node->point[1], best_global_node->point[2]);
    ROS_INFO("[AEPlanner]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_node->gain, best_global_node->cost, best_global_node->score);

    visualize_path(best_global_node, ns);
}

void AEPlanner::cacheNode(rrt_star::Node* Node, double gain, double yaw) {
    if (!Node) {
        return;
    }
    cache_nodes::Node cached_node;
    cached_node.gain = gain;   // absolute gain, so the frontier server (threshold g_zero) keeps this node
    cached_node.position.x = Node->point[0];
    cached_node.position.y = Node->point[1];
    cached_node.position.z = Node->point[2];
    cached_node.yaw = yaw;
    pub_node.publish(cached_node);
}

double AEPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) { return planner_helpers::distance(waypoint, pose); }

void AEPlanner::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[AEPlanner]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(3).sleep();

    ROS_INFO("[AEPlanner]: Rotating 360 degrees");

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

    ROS_INFO("[AEPlanner]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(2).sleep();
}

void AEPlanner::rotate() {
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

    changeState(STATE_INITIALIZE);

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

void AEPlanner::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
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
    geometry_msgs::Pose uav_state = msg->pose;
    double yaw = mrs_lib::getYaw(uav_state);
    pose = {uav_state.position.x, uav_state.position.y, uav_state.position.z, yaw};
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
        // Give the TF tree a moment to populate; on a fast start the
        // body->camera transform may not be available yet.
        ros::Duration(0.5).sleep();
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
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEPlanner]: tracker has goal");
            } else {
                ROS_INFO("[AEPlanner]: waiting for command");
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
            // Optimistic edges (plan through unknown) only for the first optimistic_iterations_ replans to
            // bootstrap away from spawn; afterwards unknown counts as blocked so we never drive into a pocket.
            optimistic_edges_ = (iteration_ < optimistic_iterations_);

            retreating_ = false;   // fresh forward attempt; a boxed-in backtrack inside AEP() re-sets this
            {
                ros::WallTime plan_t0 = ros::WallTime::now();
                AEP();
                total_planning_ms_ += (ros::WallTime::now() - plan_t0).toSec() * 1000.0;
            }
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            // Boxed-in backtrack: fly to the previous node (back()); the backtrack already popped the current node.
            if (retreating_ && !executed_path_.empty()) {
                retreat_node_ = std::make_unique<rrt_star::Node>(executed_path_.back());
                retreat_node_->parent = nullptr;
                next_best_node = retreat_node_.get();
            }

            iteration_ += 1;

            if (!current_waypoint_) {
                current_waypoint_ = std::make_unique<mrs_msgs::Reference>();
            }

            current_waypoint_->position.x = next_best_node->point[0];
            current_waypoint_->position.y = next_best_node->point[1];
            current_waypoint_->position.z = next_best_node->point[2];
            current_waypoint_->heading = next_best_node->point[3];

            next_start[0] = current_waypoint_->position.x;
            next_start[1] = current_waypoint_->position.y;
            next_start[2] = current_waypoint_->position.z;
            next_start[3] = current_waypoint_->heading;

            visualize_frustum(next_best_node);
            visualize_unknown_voxels(next_best_node);

            mrs_msgs::Reference reference;

            waypoints_.clear();
            waypoint_index_ = 0;

            while (next_best_node && next_best_node->parent) {
                mrs_msgs::Reference ref;
                ref.position.x = next_best_node->point[0];
                ref.position.y = next_best_node->point[1];
                ref.position.z = next_best_node->point[2];
                ref.heading    = next_best_node->point[3];

                waypoints_.push_back(ref);

                next_best_node = next_best_node->parent;
            }
            std::reverse(waypoints_.begin(), waypoints_.end());

            // Retreat: retreat_node_ has no parent so the walk above is empty; fly straight to it (edge already flown/validated).
            if (waypoints_.empty() && next_best_node) {
                mrs_msgs::Reference ref;
                ref.position.x = next_best_node->point[0];
                ref.position.y = next_best_node->point[1];
                ref.position.z = next_best_node->point[2];
                ref.heading    = next_best_node->point[3];
                waypoints_.push_back(ref);
            }

            // Store the flown waypoints (forward moves only), seeding the tree root once (next_best_node is the root
            // here) so the stack holds the full path incl. the takeoff; back() = the current node.
            if (!retreating_) {
                if (executed_path_.empty() && next_best_node) {
                    executed_path_.emplace_back(next_best_node->point[0], next_best_node->point[1],
                                                next_best_node->point[2], next_best_node->point[3]);
                }
                for (const auto& wp : waypoints_) {
                    executed_path_.emplace_back(wp.position.x, wp.position.y, wp.position.z, wp.heading);
                }
            }

            mrs_msgs::ReferenceStamped initial_reference;
            initial_reference.header.frame_id = ns + "/" + frame_id;
            initial_reference.header.stamp = ros::Time::now();

            initial_reference.reference = waypoints_[0];
            pub_reference.publish(initial_reference.reference);
            pub_initial_reference.publish(initial_reference);

            ros::Duration(1).sleep();

            changeState(STATE_MOVING);
            break;
            
        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                const mrs_msgs::Reference& wp = waypoints_[waypoint_index_];
                std::unique_ptr<mrs_msgs::Reference> wp_ptr = std::make_unique<mrs_msgs::Reference>(wp);

                double dist = distance(wp_ptr, current_pose);
                double yaw_difference = fabs(atan2(sin(wp.heading - current_yaw), cos(wp.heading - current_yaw)));
                ROS_INFO("[AEPlanner]: WP %d/%zu: dist=%.2f, yaw=%.2f",
                        waypoint_index_+1,
                        waypoints_.size(),
                        dist, yaw_difference);

                if (dist < waypoint_reach_distance_ && yaw_difference < 0.4) {
                    waypoint_index_++;

                    if (waypoint_index_ >= waypoints_.size()) {
                        ROS_INFO("[AEPlanner]: Going to final waypoint, waiting for tracker to finish.");
                        break;
                    }

                    mrs_msgs::ReferenceStamped next_ref;
                    next_ref.header.frame_id = ns + "/" + frame_id;
                    next_ref.header.stamp = ros::Time::now();
                    next_ref.reference = waypoints_[waypoint_index_];

                    pub_reference.publish(next_ref.reference);
                    pub_initial_reference.publish(next_ref);

                    current_waypoint_ = std::make_unique<mrs_msgs::Reference>(next_ref.reference);

                    ROS_INFO("[AEPlanner]: Publishing next waypoint");
                }
            } else {
                ROS_INFO("[AEPlanner]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[AEPlanner]: Total Iterations: %d", iteration_);
            if (!stats_written_) {
                std::string log_dir;
                if (nh_private_.getParam("performance_log_dir", log_dir) && !log_dir.empty()) {
                    std::ofstream dl(log_dir + "/data_log.txt", std::ios::app);
                    if (dl.is_open())
                        dl << "total_planning_time_ms=" << total_planning_ms_ << " iterations=" << iteration_ << "\n";
                }
                stats_written_ = true;
            }
            ROS_INFO("[AEPlanner]: Shutting down.");
            ros::shutdown();
            return;
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

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[AEPlanner]: Planning interrupted, not changing state.");
        return;
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

void AEPlanner::visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns) {
    visualization_msgs::Marker edges;
    edges.header.stamp = ros::Time::now();
    edges.header.frame_id = ns + "/" + frame_id;
    edges.ns = "tree_branches";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.06;   // line width (thin lines render sub-pixel at maze scale and look transparent)
    edges.color.r = 1.0; edges.color.g = 0.3; edges.color.b = 0.7; edges.color.a = 1.0;
    edges.lifetime = ros::Duration(30.0);

    visualization_msgs::Marker pts;
    pts.header = edges.header;
    pts.ns = "nodes";
    pts.id = 0;
    pts.type = visualization_msgs::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = pts.scale.y = pts.scale.z = 0.2;
    pts.color.r = 0.4; pts.color.g = 0.7; pts.color.b = 0.2; pts.color.a = 1.0;
    pts.lifetime = ros::Duration(30.0);

    for (rrt_star::Node* node : nodes) {
        geometry_msgs::Point p;
        p.x = node->point[0]; p.y = node->point[1]; p.z = node->point[2];
        pts.points.push_back(p);
        if (node->parent) {
            geometry_msgs::Point pp;
            pp.x = node->parent->point[0]; pp.y = node->parent->point[1]; pp.z = node->parent->point[2];
            edges.points.push_back(pp);   // LINE_LIST consumes points in pairs (parent -> child = one segment)
            edges.points.push_back(p);
        }
    }
    pub_markers.publish(edges);
    pub_markers.publish(pts);
}

void AEPlanner::visualize_edge(rrt_star::Node* node, const std::string& ns) {
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

void AEPlanner::visualize_path(rrt_star::Node* node, const std::string& ns) {
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

        p.lifetime = ros::Duration(100.0);
        p.frame_locked = false;
        pub_markers.publish(p);

        current = current->parent;
        path_id_counter_++;
    }
}

void AEPlanner::visualize_frustum(rrt_star::Node* position) {
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

void AEPlanner::visualize_unknown_voxels(rrt_star::Node* position) {
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
        double vsz = segment_evaluator.getVoxelSize();
        unknown_voxel.scale.x = vsz;
        unknown_voxel.scale.y = vsz;
        unknown_voxel.scale.z = vsz;

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
