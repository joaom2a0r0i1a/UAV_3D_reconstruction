#include "motion_planning/AEP/AEPlanner.h"

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
            return;
        }
        goto_global_planning = false;
    }
}

void AEPlanner::localPlannerGPUBenchmark() {
    best_score_ = 0;
    std::shared_ptr<rrt_star::Node> best_node = nullptr;

    // BENCHMARKING ACCUMULATORS
    double total_time_gpu_calc = 0.0;
    double total_time_cpu_calc = 0.0;
    int total_nodes_evaluated = 0;

    int j = 1; // Total nodes processed

    ROS_INFO("[AEPlanner]: Start Expanding Local");

    // -------------------------------------------------------
    // 1. SETUP ROOT
    // -------------------------------------------------------
    std::shared_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_shared<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_shared<rrt_star::Node>(pose);
    }

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    clearMarkers();

    // -------------------------------------------------------
    // 2. MAP MANAGEMENT
    // -------------------------------------------------------
    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);
    
    sensor_msgs::PointCloud2 debug_msg = segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_);
    pub_gpu_debug.publish(debug_msg);

    // -------------------------------------------------------
    // 3. PHASE A: RE-EVALUATE PREVIOUS BEST BRANCH (BATCHED)
    // -------------------------------------------------------
    if (best_branch.size() > 1) {
        std::vector<std::shared_ptr<rrt_star::Node>> branch_candidates;
        std::vector<double> bx, by, bz;

        bool isFirstIteration = true;
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) { isFirstIteration = false; continue; }

            const Eigen::Vector4d& node_position = best_branch[i]->point;
            
            std::shared_ptr<rrt_star::Node> nearest_node_best;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            
            std::shared_ptr<rrt_star::Node> new_node_best;
            new_node_best = std::make_shared<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;

            segment_evaluator.computeCost(new_node_best);
            RRTStar.addKDTreeNode(new_node_best);

            branch_candidates.push_back(new_node_best);
            bx.push_back(node_position.x());
            by.push_back(node_position.y());
            bz.push_back(node_position.z());
        }

        if (!branch_candidates.empty()) {
            // --- BENCHMARK START ---
            auto t1 = std::chrono::high_resolution_clock::now();
            auto results = segment_evaluator.computeGainBatchGPU(bx, by, bz);
            auto t2 = std::chrono::high_resolution_clock::now();
            total_time_gpu_calc += std::chrono::duration<double, std::milli>(t2 - t1).count();

            auto t3 = std::chrono::high_resolution_clock::now();
            for (size_t k = 0; k < branch_candidates.size(); ++k) {
                eth_mav_msgs::EigenTrajectoryPoint dummy_pose;
                dummy_pose.position_W = Eigen::Vector3d(bx[k], by[k], bz[k]);
                segment_evaluator.computeGainCPU_FlatMap(flat_map_, dummy_pose);
            }
            auto t4 = std::chrono::high_resolution_clock::now();
            total_time_cpu_calc += std::chrono::duration<double, std::milli>(t4 - t3).count();
            
            total_nodes_evaluated += branch_candidates.size();
            // --- BENCHMARK END ---

            for (size_t k = 0; k < branch_candidates.size(); ++k) {
                auto node = branch_candidates[k];
                node->gain = results[k].first;
                node->point[3] = results[k].second; 
                segment_evaluator.computeScore(node, lambda);

                if (node->score > best_score_) {
                    best_score_ = node->score;
                    best_node = node;
                }
                visualize_edge(node, ns);
                visualize_node(node->point, ns);
            }
            j += branch_candidates.size();
        }
    }

    best_branch.clear();

    // -------------------------------------------------------
    // 4. PHASE B: RRT EXPANSION LOOP (BATCHED)
    // -------------------------------------------------------
    const int BATCH_SIZE = 100; 
    collision_id_counter_ = 0;

    while (j < N_max || best_score_ <= g_zero) {

        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int current_batch_cap = std::min(BATCH_SIZE, nodes_needed);
        if (current_batch_cap <= 0) break;

        if (collision_id_counter_ > 10000 * j) {
            if (previous_node) {
                ROS_INFO("[AEPlanner]: Backtracking...");
                next_best_node = previous_node;
                best_branch.clear();
                return;
            } else {
                rotate();
                collision_id_counter_ = 0;
            }
        }

        std::vector<std::shared_ptr<rrt_star::Node>> batch_nodes;
        std::vector<double> batch_x, batch_y, batch_z;
        batch_nodes.reserve(current_batch_cap);

        for (int k = 0; k < current_batch_cap && j <= N_termination; ++k) {
            Eigen::Vector3d rand_point;
            RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
            rand_point += root->point.head(3);

            std::shared_ptr<rrt_star::Node> nearest_node;
            RRTStar.findNearestKD(rand_point, nearest_node);

            std::shared_ptr<rrt_star::Node> new_node;
            RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

            if (new_node->point[0] > max_x || new_node->point[0] < min_x || 
                new_node->point[1] < min_y || new_node->point[1] > max_y || 
                new_node->point[2] < min_z || new_node->point[2] > max_z) {
                k--; continue;
            }

            std::vector<std::shared_ptr<rrt_star::Node>> segment = {new_node};
            if (!isPathCollisionFree(segment)) {
                collision_id_counter_++;
                k--; continue;
            }

            segment_evaluator.computeCost(new_node); 
            new_node->gain = 0.0;
            new_node->score = 0.0;
            
            RRTStar.addKDTreeNode(new_node);

            batch_nodes.push_back(new_node);
            batch_x.push_back(new_node->point.x());
            batch_y.push_back(new_node->point.y());
            batch_z.push_back(new_node->point.z());
            j++;
        }

        if (batch_nodes.empty()) continue;

        // --- BENCHMARK START ---
        auto t1 = std::chrono::high_resolution_clock::now();
        auto batch_results = segment_evaluator.computeGainBatchGPU(batch_x, batch_y, batch_z);
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time_gpu_calc += std::chrono::duration<double, std::milli>(t2 - t1).count();

        auto t3 = std::chrono::high_resolution_clock::now();
        for (size_t k = 0; k < batch_nodes.size(); ++k) {
             eth_mav_msgs::EigenTrajectoryPoint dummy_pose;
             dummy_pose.position_W = Eigen::Vector3d(batch_x[k], batch_y[k], batch_z[k]);
             segment_evaluator.computeGainCPU_FlatMap(flat_map_, dummy_pose);
        }
        auto t4 = std::chrono::high_resolution_clock::now();
        total_time_cpu_calc += std::chrono::duration<double, std::milli>(t4 - t3).count();
        
        total_nodes_evaluated += batch_nodes.size();
        // --- BENCHMARK END ---

        for (size_t i = 0; i < batch_nodes.size(); ++i) {
            auto node = batch_nodes[i];
            node->gain = batch_results[i].first;
            node->point[3] = batch_results[i].second;

            segment_evaluator.computeScore(node, lambda);

            if (node->score > best_score_) {
                best_score_ = node->score;
                best_node = node;
            }

            visualize_edge(node, ns);
            visualize_node(node->point, ns);

            if (node->gain > g_zero) {
                cacheNode(node);
            }

            if (j >= N_max && best_score_ > g_zero) break;
        }

        if (j > N_termination) {
             ROS_INFO("[AEPlanner]: Going to Global Planning");
             RRTStar.clearKDTree();
             best_branch.clear();
             clearMarkers();
             goto_global_planning = true;
             return;
        }
    }

    // -------------------------------------------------------
    // 5. SAVE BENCHMARK DATA (CSV LOGGING)
    // -------------------------------------------------------
    if (total_nodes_evaluated > 0) {
        // Calculate metrics
        double safe_gpu_time = (total_time_gpu_calc > 0) ? total_time_gpu_calc : 0.001;
        double speedup = total_time_cpu_calc / safe_gpu_time;
        double avg_cpu = total_time_cpu_calc / total_nodes_evaluated;
        double avg_gpu = total_time_gpu_calc / total_nodes_evaluated;

        // 1. Console Output (Immediate Feedback)
        ROS_INFO_THROTTLE(1.0, 
            "\n=== GPU BENCHMARK LOGGED ===\n"
            "Nodes: %d | CPU: %.2f ms | GPU: %.2f ms | Speedup: %.2fx",
            total_nodes_evaluated, total_time_cpu_calc, total_time_gpu_calc, speedup
        );

        // 2. File Output (Persistent Data)
        // Adjust path as needed, e.g., "/home/user/ros_ws/src/..."
        std::string log_path = "/home/joaomendes/motion_workspace/src/UAV_3D_reconstruction/motion_planning/tmux/one_drone/aeplanner_benchmark_2.csv"; 
        std::ofstream log_file;
        
        // Open in Append Mode
        log_file.open(log_path, std::ios_base::app); 

        if (log_file.is_open()) {
            // Optional: Write header if file is empty
            log_file.seekp(0, std::ios::end);
            if (log_file.tellp() == 0) {
                log_file << "Timestamp_Sec,Total_Nodes,Total_CPU_Time_ms,Total_GPU_Time_ms,Avg_CPU_ms,Avg_GPU_ms,Speedup_Factor\n";
            }

            // Get current time for the log entry
            double timestamp = ros::Time::now().toSec();

            log_file << std::fixed << std::setprecision(4)
                     << timestamp << ","
                     << total_nodes_evaluated << ","
                     << total_time_cpu_calc << ","
                     << total_time_gpu_calc << ","
                     << avg_cpu << ","
                     << avg_gpu << ","
                     << speedup << "\n";
            
            log_file.close();
        } else {
            ROS_WARN("[AEPlanner]: Failed to open benchmark log file at %s", log_path.c_str());
        }
    }

    // -------------------------------------------------------
    // 6. FINALIZE
    // -------------------------------------------------------
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    next_best_node = best_branch[1];
}

void AEPlanner::localPlannerGPU() {
    best_score_ = 0;
    std::shared_ptr<rrt_star::Node> best_node = nullptr;

    // BENCHMARKING ACCUMULATORS
    double total_time_gpu_calc = 0.0;
    double total_time_cpu_calc = 0.0;
    int total_nodes_evaluated = 0;

    int j = 1; // Total nodes processed

    ROS_INFO("[AEPlanner]: Start Expanding Local");

    // -------------------------------------------------------
    // 1. SETUP ROOT
    // -------------------------------------------------------
    std::shared_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_shared<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_shared<rrt_star::Node>(pose);
    }

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    clearMarkers();

    // -------------------------------------------------------
    // 2. MAP MANAGEMENT (DAY 4 OPTIMIZATION)
    // -------------------------------------------------------
    // We flatten and upload ONCE. The GPU wrapper keeps it persistent.
    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);
    
    // Visualize for Debug
    sensor_msgs::PointCloud2 debug_msg = segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_);
    pub_gpu_debug.publish(debug_msg);

    // -------------------------------------------------------
    // 3. PHASE A: RE-EVALUATE PREVIOUS BEST BRANCH (BATCHED)
    // -------------------------------------------------------
    if (best_branch.size() > 1) {
        std::vector<std::shared_ptr<rrt_star::Node>> branch_candidates;
        std::vector<double> bx, by, bz;

        // A.1 Collect Candidates (CPU Geometry)
        bool isFirstIteration = true;
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) { isFirstIteration = false; continue; }

            const Eigen::Vector4d& node_position = best_branch[i]->point;
            
            std::shared_ptr<rrt_star::Node> nearest_node_best;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            
            std::shared_ptr<rrt_star::Node> new_node_best;
            new_node_best = std::make_shared<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;

            segment_evaluator.computeCost(new_node_best);
            RRTStar.addKDTreeNode(new_node_best);

            branch_candidates.push_back(new_node_best);
            bx.push_back(node_position.x());
            by.push_back(node_position.y());
            bz.push_back(node_position.z());
        }

        // A.2 Compute Gain (GPU Batch) - Lightning Fast
        if (!branch_candidates.empty()) {
            // --- BENCHMARK START ---
            
            // 1. Measure GPU Batch Time
            auto t1 = std::chrono::high_resolution_clock::now();
            auto results = segment_evaluator.computeGainBatchGPU(bx, by, bz);
            auto t2 = std::chrono::high_resolution_clock::now();
            total_time_gpu_calc += std::chrono::duration<double, std::milli>(t2 - t1).count();

            // 2. Measure CPU Time (Simulation for Comparison)
            auto t3 = std::chrono::high_resolution_clock::now();
            for (size_t k = 0; k < branch_candidates.size(); ++k) {
                eth_mav_msgs::EigenTrajectoryPoint dummy_pose;
                dummy_pose.position_W = Eigen::Vector3d(bx[k], by[k], bz[k]);
                // CPU calculation on the same flat map for fairness
                segment_evaluator.computeGainCPU_FlatMap(flat_map_, dummy_pose);
            }
            auto t4 = std::chrono::high_resolution_clock::now();
            total_time_cpu_calc += std::chrono::duration<double, std::milli>(t4 - t3).count();
            
            total_nodes_evaluated += branch_candidates.size();
            // --- BENCHMARK END ---

            //auto results = segment_evaluator.computeGainBatchGPU(flat_map_, map_origin_, map_dim_, bx, by, bz);

            // A.3 Update & Add to Tree
            for (size_t k = 0; k < branch_candidates.size(); ++k) {
                auto node = branch_candidates[k];
                node->gain = results[k].first;
                node->point[3] = results[k].second; // Best Yaw

                //segment_evaluator.computeCost(node);
                segment_evaluator.computeScore(node, lambda);

                if (node->score > best_score_) {
                    best_score_ = node->score;
                    best_node = node;
                }

                //RRTStar.addKDTreeNode(node);
                visualize_edge(node, ns);
                visualize_node(node->point, ns);
            }
            j += branch_candidates.size();
        }
    }

    best_branch.clear();

    // -------------------------------------------------------
    // 4. PHASE B: RRT EXPANSION LOOP (BATCHED)
    // -------------------------------------------------------
    // We define a Batch Size (e.g., 32 or 64). 
    // We accumulate valid geometry candidates, then ask GPU for gains.
    const int BATCH_SIZE = 100; 
    collision_id_counter_ = 0;

    while (j < N_max || best_score_ <= g_zero) {

        // --- 1. Precise Batch Sizing Logic ---
        int nodes_needed = 0;
        if (j < N_max) {
             // Standard Phase: We MUST reach N_max.
             nodes_needed = N_max - j;
        } else {
             // Overtime Phase: We passed N_max but have no gain. Go up to N_termination.
             nodes_needed = N_termination - j;
        }
        // Clamp: Don't do more than BATCH_SIZE at once
        int current_batch_cap = std::min(BATCH_SIZE, nodes_needed);
        // Safety: If we hit the limit, stop.
        if (current_batch_cap <= 0) break;

        // --- Backtrack Check (Same as before) ---
        if (collision_id_counter_ > 10000 * j) {
            if (previous_node) {
                ROS_INFO("[AEPlanner]: Backtracking...");
                next_best_node = previous_node;
                best_branch.clear();
                return;
            } else {
                ROS_INFO("[AEPlanner]: Backtrack Rotation");
                rotate();
                collision_id_counter_ = 0;
            }
        }

        // --- BATCH COLLECTION ---
        std::vector<std::shared_ptr<rrt_star::Node>> batch_nodes;
        std::vector<double> batch_x, batch_y, batch_z;
        batch_nodes.reserve(current_batch_cap);

        // Try to fill the batch
        for (int k = 0; k < current_batch_cap && j <= N_termination; ++k) {
            
            // 1. Sample
            Eigen::Vector3d rand_point;
            RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
            rand_point += root->point.head(3);

            // 2. Nearest & Steer
            std::shared_ptr<rrt_star::Node> nearest_node;
            RRTStar.findNearestKD(rand_point, nearest_node);

            std::shared_ptr<rrt_star::Node> new_node;
            RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

            // 3. Bounds Check
            if (new_node->point[0] > max_x || new_node->point[0] < min_x || 
                new_node->point[1] < min_y || new_node->point[1] > max_y || 
                new_node->point[2] < min_z || new_node->point[2] > max_z) {
                k--; // Retry this slot
                continue;
            }

            // 4. Collision Check (CPU)
            std::vector<std::shared_ptr<rrt_star::Node>> segment = {new_node};
            if (!isPathCollisionFree(segment)) {
                collision_id_counter_++;
                k--; // Retry this slot
                continue;
            }

            segment_evaluator.computeCost(new_node); 
            new_node->gain = 0.0; // Placeholder
            new_node->score = 0.0; // Placeholder
            
            RRTStar.addKDTreeNode(new_node);

            // 5. Add to Batch (Geometry is valid, Gain is unknown)
            batch_nodes.push_back(new_node);
            batch_x.push_back(new_node->point.x());
            batch_y.push_back(new_node->point.y());
            batch_z.push_back(new_node->point.z());

            // Increment global counter (we count attempts/samples)
            j++;
        }

        // If we couldn't find any valid nodes (e.g. boxed in), skip gpu
        if (batch_nodes.empty()) continue;

        // --- BENCHMARK START ---
        
        // 1. Measure GPU
        auto t1 = std::chrono::high_resolution_clock::now();
        auto batch_results = segment_evaluator.computeGainBatchGPU(batch_x, batch_y, batch_z);
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time_gpu_calc += std::chrono::duration<double, std::milli>(t2 - t1).count();

        // 2. Measure CPU (Benchmark Only)
        auto t3 = std::chrono::high_resolution_clock::now();
        for (size_t k = 0; k < batch_nodes.size(); ++k) {
             eth_mav_msgs::EigenTrajectoryPoint dummy_pose;
             dummy_pose.position_W = Eigen::Vector3d(batch_x[k], batch_y[k], batch_z[k]);
             segment_evaluator.computeGainCPU_FlatMap(flat_map_, dummy_pose);
        }
        auto t4 = std::chrono::high_resolution_clock::now();
        total_time_cpu_calc += std::chrono::duration<double, std::milli>(t4 - t3).count();
        
        total_nodes_evaluated += batch_nodes.size();
        // --- BENCHMARK END ---

        // --- GPU BATCH COMPUTE ---
        // This is the Magic Line. 64 nodes processed in ~0.2ms total.
        //auto batch_results = segment_evaluator.computeGainBatchGPU(flat_map_, map_origin_, map_dim_, batch_x, batch_y, batch_z);

        // --- PROCESS RESULTS ---
        for (size_t i = 0; i < batch_nodes.size(); ++i) {
            auto node = batch_nodes[i];
            
            node->gain = batch_results[i].first;
            node->point[3] = batch_results[i].second; // Yaw from GPU

            //segment_evaluator.computeCost(node);
            segment_evaluator.computeScore(node, lambda);

            ROS_INFO("[AEPlanner]: Best Gain Optimized: %f", node->score);

            // Update Best
            if (node->score > best_score_) {
                best_score_ = node->score;
                best_node = node;
            }

            // Add to Tree
            //RRTStar.addKDTreeNode(node);
            visualize_edge(node, ns);
            visualize_node(node->point, ns);

            // Cache for Global
            if (node->gain > g_zero) {
                cacheNode(node);
            }

            // Strict Termination Logic:
            // Stop if we satisfied N_max AND found something useful
            if (j >= N_max && best_score_ > g_zero) {
                break;
            }
        }

        // Termination Check
        if (j > N_termination) {
             ROS_INFO("[AEPlanner]: Going to Global Planning");
             RRTStar.clearKDTree();
             best_branch.clear();
             clearMarkers();
             goto_global_planning = true;
             return;
        }
    }

    // -------------------------------------------------------
    // PRINT BENCHMARK REPORT
    // -------------------------------------------------------
    if (total_nodes_evaluated > 0) {
        double speedup = total_time_cpu_calc / (total_time_gpu_calc > 0 ? total_time_gpu_calc : 0.001);
        ROS_INFO_THROTTLE(1.0, 
            "\n=== GPU vs CPU EFFICIENCY REPORT (Same Nodes) ===\n"
            "Nodes Evaluated : %d\n"
            "Total CPU Time  : %8.3f ms\n"
            "Total GPU Time  : %8.3f ms\n"
            "Avg CPU / Node  : %8.4f ms\n"
            "Avg GPU / Node  : %8.4f ms\n"
            "SPEEDUP FACTOR  : %8.2f x\n"
            "=================================================",
            total_nodes_evaluated, 
            total_time_cpu_calc, total_time_gpu_calc,
            total_time_cpu_calc / total_nodes_evaluated,
            total_time_gpu_calc / total_nodes_evaluated,
            speedup
        );
    }

    // -------------------------------------------------------
    // 5. FINALIZE
    // -------------------------------------------------------
    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    next_best_node = best_branch[1];
}

void AEPlanner::localPlanner() {
    best_score_ = 0;
    std::shared_ptr<rrt_star::Node> best_node = nullptr;

    ROS_INFO("[AEPlanner]: Start Expanding Local");

    std::shared_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_shared<rrt_star::Node>(best_branch[1]->point);
    } else {
        root = std::make_shared<rrt_star::Node>(pose);
    }

    root->depth_buffer.clear();

    int p_width = ceil((2.0f * max_distance * tanf(horizontal_fov * 0.5f)) / 0.2f);
    int p_height = ceil((2.0f * max_distance * tanf(vertical_fov * 0.5f)) / 0.2f);

    // 2. Resize and Fill with MAX RANGE (Empty Space)
    // This matches your "Root is empty" concept, but provides valid memory.
    root->depth_buffer.resize(p_width * p_height, -1.0f);
    ROS_INFO("Root depth size: %lu", root->depth_buffer.size());

    RRTStar.clearKDTree();
    RRTStar.addKDTreeNode(root);
    clearMarkers();

    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    float camera_pitch_rad = 10.0f * M_PI / 180.0f;

    //Eigen::Matrix3f R_CB = T_C_B.getRotation().toImplementation().toRotationMatrix().cast<float>();

    // Helper: Get Parent Camera State (Rotation + Position)
    /*auto get_parent_cam_state = [&](const Eigen::Vector3d& body_pos, float yaw) -> std::pair<std::vector<float>, Eigen::Vector3d> {
        Eigen::Vector3d parent_pos = body_pos;

        // R_body: Body -> World Rotation
        // Col 0 = Body X (Front)
        // Col 1 = Body Y (Left)
        // Col 2 = Body Z (Up)
        Eigen::Matrix3d R_body;
        R_body = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

        std::vector<float> R_flat(9);

        // Goal: Construct R0, R1, R2 for the Kernel.
        // The Kernel's "compute_skip_distance" needs World->Camera projection.
        // This expects R0, R1, R2 to be the ROWS of R_wc.
        // Since R_wc = (R_cw)^T, the ROWS of R_wc are the COLUMNS of R_cw.
        
        // Columns of R_cw (Camera Axes in World Frame):
        // Col 0 (Cam Right) = Body -Y (Left)
        // Col 1 (Cam Down)  = Body -Z (Up)
        // Col 2 (Cam Front) = Body X  (Front)

        // --- CUDA R0: Camera X Axis (Right) = -Body Y (Col 1) ---
        R_flat[0] = -(float)R_body(0,1); // x component
        R_flat[1] = -(float)R_body(1,1); // y component
        R_flat[2] = -(float)R_body(2,1); // z component

        // --- CUDA R1: Camera Y Axis (Down) = -Body Z (Col 2) ---
        R_flat[3] = -(float)R_body(0,2);
        R_flat[4] = -(float)R_body(1,2);
        R_flat[5] = -(float)R_body(2,2);

        // --- CUDA R2: Camera Z Axis (Forward) = Body X (Col 0) ---
        R_flat[6] = (float)R_body(0,0);
        R_flat[7] = (float)R_body(1,0);
        R_flat[8] = (float)R_body(2,0);

        return {R_flat, parent_pos};
    };*/

    auto get_parent_cam_state = [&](const Eigen::Vector3d& body_pos, float yaw, float pitch_rad) -> std::pair<std::vector<float>, Eigen::Vector3d> {
    
        // 1. Precompute Trig (Ensure pitch is RADIANS!)
        float cos_y = cosf(yaw);
        float sin_y = sinf(yaw);
        float cos_p = cosf(pitch_rad); 
        float sin_p = sinf(pitch_rad);

        std::vector<float> R_flat(9);

        // 2. Explicit Basis Construction (Matches GPU Kernel Logic)
        // We derived these by rotating the Camera Basis vectors 
        // (Right, Down, Front) using the Body->World chain.

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

        // R2: Camera Front Axis (z_cam=1) in World
        // Kernel Formula: dir_x = cos_p*cos_y, dir_y = cos_p*sin_y, dir_z = -sin_p
        // Note: If Pitch=0, this is (cos_y, sin_y, 0) -> Points in Yaw dir. Correct.
        // Note: If Pitch>0 (Look Down), z becomes -sin_p (Negative). Correct.
        R_flat[6] = cos_p * cos_y;
        R_flat[7] = cos_p * sin_y;
        R_flat[8] = -sin_p;

        return {R_flat, body_pos};
    };

    sensor_msgs::PointCloud2 debug_msg = segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_);
    ROS_WARN("GPU Debug Map Published! Check Rviz topic /gpu_debug_map");
    pub_gpu_debug.publish(debug_msg);

    /*eth_mav_msgs::EigenTrajectoryPoint pose_trajectory;
    pose_trajectory.position_W = root->point.head(3);
    pose_trajectory.setFromYaw(root->point[3]);

    // 1. Run CPU (Reference)
    auto start_cpu = std::chrono::high_resolution_clock::now();
    std::pair<double, double> res_cpu = segment_evaluator.computeGainRaycasting(pose_trajectory);
    auto end_cpu = std::chrono::high_resolution_clock::now();

    auto start_cpu_flat = std::chrono::high_resolution_clock::now();
    std::pair<double, double> res_cpu_flat = segment_evaluator.computeGainCPU_FlatMap(flat_map_, pose_trajectory);
    auto end_cpu_flat = std::chrono::high_resolution_clock::now();

    // 3. Run GPU (Challenger)
    auto start_gpu = std::chrono::high_resolution_clock::now();
    std::pair<double, double> res_gpu = segment_evaluator.computeGainGPU({root->point.x()}, {root->point.y()}, {root->point.z()});
    auto end_gpu = std::chrono::high_resolution_clock::now();

    // 4. Results
    double cpu_ms = std::chrono::duration<double, std::milli>(end_cpu - start_cpu).count();
    double cpu_flat_ms = std::chrono::duration<double, std::milli>(end_cpu_flat - start_cpu_flat).count();
    double gpu_ms = std::chrono::duration<double, std::milli>(end_gpu - start_gpu).count();

    ROS_INFO_THROTTLE(0.5, 
        "\n--- GAIN CHECK ---\n"
        "CPU: Gain=%.2f | Yaw=%.2f | Time=%.3f ms\n"
        "CPU FLAT: Gain=%.2f | Yaw=%.2f | Time=%.3f ms\n"
        "GPU: Gain=%.2f | Yaw=%.2f | Time=%.3f ms\n"
        "------------------",
        res_cpu.first, res_cpu.second, cpu_ms,
        res_cpu_flat.first, res_cpu_flat.second, cpu_flat_ms,
        res_gpu.first, res_gpu.second, gpu_ms
    );*/

    visualize_node(root->point, ns);
    bool isFirstIteration = true;
    int j = 1; // initialized at one because of the root node
    collision_id_counter_ = 0;
    while (j < N_max || best_score_ <= g_zero) {
        // Backtrack
        if (collision_id_counter_ > 10000 * j) {
            if (previous_node) {
                ROS_INFO("[AEPlanner]: Backtracking to [%f, %f, %f]", previous_node->point[0], previous_node->point[1], previous_node->point[2]);
                next_best_node = previous_node;
                best_branch.clear();
                return;
            } else {
                ROS_INFO("[AEPlanner]: Backtrack Rotation");
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
            
            const Eigen::Vector4d& node_position = best_branch[i]->point;

            std::shared_ptr<rrt_star::Node> nearest_node_best;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            
            std::shared_ptr<rrt_star::Node> new_node_best;
            new_node_best = std::make_shared<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;
            visualize_node(new_node_best->point, ns);

            auto [parent_R, parent_cam_pos] = get_parent_cam_state(
                new_node_best->parent->point.head(3), 
                (float)new_node_best->parent->point[3], camera_pitch_rad /*pitch*/
            );

            std::pair<double, double> result = segment_evaluator.computeMarginalGainGPU(
                new_node_best->point.x(), new_node_best->point.y(), new_node_best->point.z(),
                parent_cam_pos, new_node_best->parent->point[3], parent_R, new_node_best->parent->depth_buffer, new_node_best->depth_buffer);

            /*trajectory_point.position_W = new_node_best->point.head(3);
            trajectory_point.setFromYaw(new_node_best->point[3]);

            std::pair<double, double> result = segment_evaluator.computeGainOptimizedRaycasting(trajectory_point);*/

            new_node_best->gain = result.first;
            new_node_best->point[3] = result.second;

            segment_evaluator.computeCost(new_node_best);
            segment_evaluator.computeScore(new_node_best, lambda);

            if (new_node_best->score > best_score_) {
                best_score_ = new_node_best->score;
                best_node = new_node_best;
            }

            //ROS_INFO("[AEPlanner]: Best Score BB: %f", new_node_best->score);
            

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
        rand_point += root->point.head(3);

        std::shared_ptr<rrt_star::Node> nearest_node;
        RRTStar.findNearestKD(rand_point, nearest_node);

        std::shared_ptr<rrt_star::Node> new_node;
        RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

        if (new_node->point[0] > max_x || new_node->point[0] < min_x || new_node->point[1] < min_y || new_node->point[1] > max_y || new_node->point[2] < min_z || new_node->point[2] > max_z) {
            continue;
        }

        // Collision Check
        std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment;
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
        trajectory_point.position_W = new_node->point.head(3);
        trajectory_point.setFromYaw(new_node->point[3]);

        // 0. Prepare Separate Output Buffers
        std::vector<float> depth_buf_v1 = new_node->depth_buffer;
        std::vector<float> depth_buf_v2 = new_node->depth_buffer;
        std::vector<float> depth_buf_v3 = new_node->depth_buffer;

        // --- DECLARE RESULTS OUTSIDE THE TRACKING BLOCK ---
        std::pair<double, double> res_cpu = {0.0, 0.0};
        std::pair<double, double> res_cpu_hash = {0.0, 0.0};
        std::pair<double, double> res_gpu_marg = {0.0, 0.0};
        std::pair<double, double> res_gpu_marg_v2 = {0.0, 0.0};
        std::pair<double, double> res_gpu_marg_v3 = {0.0, 0.0};

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

        // ==========================================================
        // 1. ALWAYS RUN THE EVALUATIONS
        // ==========================================================

        auto start_cpu = std::chrono::high_resolution_clock::now();
        res_cpu = segment_evaluator.computeGainCPU_FlatMap(flat_map_, trajectory_point);
        auto end_cpu = std::chrono::high_resolution_clock::now();

        auto start_cpu_hash = std::chrono::high_resolution_clock::now();
        if (new_node->parent && new_node->parent->parent) {
            segment_evaluator.populateParentHistory(flat_map_, new_node->parent);
        }
        res_cpu_hash = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, new_node);
        auto end_cpu_hash = std::chrono::high_resolution_clock::now();

        auto start_gpu_marg = std::chrono::high_resolution_clock::now();
        res_gpu_marg = segment_evaluator.computeMarginalGainGPU(
            new_node->point.x(), new_node->point.y(), new_node->point.z(),
            parent_cam_pos, new_node->parent->point[3], parent_R, 
            new_node->parent->depth_buffer, depth_buf_v1); 
        auto end_gpu_marg = std::chrono::high_resolution_clock::now();

        auto start_gpu_marg_v2 = std::chrono::high_resolution_clock::now();
        res_gpu_marg_v2 = segment_evaluator.computeMarginalGainGPU_v2(
            new_node->point.x(), new_node->point.y(), new_node->point.z(),
            parent_cam_pos, new_node->parent->point[3], parent_R, 
            new_node->parent->depth_buffer, depth_buf_v2); 
        auto end_gpu_marg_v2 = std::chrono::high_resolution_clock::now();

        auto start_gpu_marg_v3 = std::chrono::high_resolution_clock::now();
        res_gpu_marg_v3 = segment_evaluator.computeMarginalGainGPU_v3(
            new_node->point.x(), new_node->point.y(), new_node->point.z(),
            parent_cam_pos, new_node->parent->point[3], parent_R, 
            new_node->parent->depth_buffer, depth_buf_v3); 
        auto end_gpu_marg_v3 = std::chrono::high_resolution_clock::now();

        // ==========================================================
        // 1.5 PHYSICAL SANITY CHECK (Marginal Gain <= Absolute Gain)
        // ==========================================================
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
        if (res_gpu_marg.first > absolute_baseline + EPSILON) {
            log_anomaly("GPU_Marg_V1", res_gpu_marg.first);
        }
        if (res_gpu_marg_v2.first > absolute_baseline + EPSILON) {
            log_anomaly("GPU_Marg_V2", res_gpu_marg_v2.first);
        }
        if (res_gpu_marg_v3.first > absolute_baseline + EPSILON) {
            log_anomaly("GPU_Marg_V3", res_gpu_marg_v3.first);
        }

        // ==========================================================
        // 2. ONLY DO THE LOGGING AND ACCUMULATION UNDER THE LIMIT
        // ==========================================================
        if (nodes_evaluated < NODE_LIMIT) {
            double ms_cpu       = std::chrono::duration<double, std::milli>(end_cpu - start_cpu).count();
            double ms_cpu_hash  = std::chrono::duration<double, std::milli>(end_cpu_hash - start_cpu_hash).count();
            double ms_gpu_marg  = std::chrono::duration<double, std::milli>(end_gpu_marg - start_gpu_marg).count();
            double ms_gpu_marg_v2 = std::chrono::duration<double, std::milli>(end_gpu_marg_v2 - start_gpu_marg_v2).count();
            double ms_gpu_marg_v3 = std::chrono::duration<double, std::milli>(end_gpu_marg_v3 - start_gpu_marg_v3).count();

            ROS_INFO(
                "\n--- GAIN EVALUATION TRIPLE THREAT ---\n"
                "1. CPU (Abs):  Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms\n"
                "2. CPU (Hash): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "3. GPU (Marg): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "4. GPU (Marg_v2): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "5. GPU (Marg_v3): Gain=%6.2f | Yaw=%5.2f | Time=%7.4f ms | Speedup (vs CPU): %.1fx\n"
                "---------------------------------------",
                res_cpu.first,      res_cpu.second,      ms_cpu,
                res_cpu_hash.first, res_cpu_hash.second, ms_cpu_hash, (ms_cpu / (ms_cpu_hash + 1e-5)),
                res_gpu_marg.first, res_gpu_marg.second, ms_gpu_marg, (ms_cpu / (ms_gpu_marg + 1e-5)),
                res_gpu_marg_v2.first, res_gpu_marg_v2.second, ms_gpu_marg_v2, (ms_cpu / (ms_gpu_marg_v2 + 1e-5)),
                res_gpu_marg_v3.first, res_gpu_marg_v3.second, ms_gpu_marg_v3, (ms_cpu / (ms_gpu_marg_v3 + 1e-5))
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

        segment_evaluator.computeCost(new_node);
        segment_evaluator.computeScore(new_node, lambda);

        if (new_node->score > best_score_) {
            best_score_ = new_node->score;
            best_node = new_node;
        }

        //ROS_INFO("[AEPlanner]: Best Gain Optimized: %f", new_node->gain);
        //ROS_INFO("[AEPlanner]: Best Gain: %f", result2.first);
        //ROS_INFO("[AEPlanner]: Best Cost: %f", new_node->cost);
        //ROS_INFO("[AEPlanner]: Best Score: %f", new_node->score);

        RRTStar.addKDTreeNode(new_node);
        visualize_edge(new_node, ns);

        if (new_node->gain > g_zero) {
            cacheNode(new_node);
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

    next_best_node = best_branch[1];
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

    ROS_INFO("[AEPlanner]: Start Expanding Global");
    
    std::shared_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_shared<rrt_star::Node>(next_start);
    } else {
        root = std::make_shared<rrt_star::Node>(pose);
    }

    RRTStar.addKDTreeNode(root);
    std::vector<std::shared_ptr<rrt_star::Node>> all_global_goals;

    collision_id_counter_ = 0;
    int m = 0;
    while (m < N_min_nodes || all_global_goals.size() <= 0) {
        if (collision_id_counter_ > 10000 * (m+1)) {
            if (previous_node) {
                ROS_INFO("[AEPlanner]: Backtracking to [%f, %f, %f]", previous_node->point[0], previous_node->point[1], previous_node->point[2]);
                next_best_node = previous_node;
                backtrack = true;
                return;
            } else {
                ROS_INFO("[AEPlanner]: Backtrack Rotation");
                rotate();
                collision_id_counter_ = 0;
            }
        }

        Eigen::Vector3d rand_point_star;
        RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
        rand_point_star += root->point.head(3);

        std::shared_ptr<rrt_star::Node> nearest_node_star;
        RRTStar.findNearestKD(rand_point_star, nearest_node_star);

        std::shared_ptr<rrt_star::Node> new_node_star;
        RRTStar.steer_parent(nearest_node_star, rand_point_star, step_size, new_node_star);

        // Collision Check
        std::vector<std::shared_ptr<rrt_star::Node>> trajectory_segment_star;
        trajectory_segment_star.push_back(new_node_star);

        bool success_collision_star = false;
        success_collision_star = isPathCollisionFree(trajectory_segment_star);

        if (!success_collision_star) {
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

        bool goal_reached;
        goal_reached = getGlobalGoal(GlobalFrontiers, new_node_star);
        if (goal_reached) {
            segment_evaluator.computeScore(new_node_star, global_lambda);
            all_global_goals.push_back(new_node_star);
            goal_reached = false;
        }
        ++m;
    }

    ROS_INFO("[AEPlanner]: Global Planner Ends");

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
            GlobalFrontiers.push_back(frontier);
        }
    }
}

bool AEPlanner::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, const std::shared_ptr<rrt_star::Node>& node) {
    // Initialize KD Tree
    goals_tree.clearKDTreePoints();
    for (size_t i = 1; i < GlobalFrontiers.size(); ++i) {
        goals_tree.addKDTreePoint(GlobalFrontiers[i]);
    }

    // Find the nearest node in the KD Tree
    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(node->point.head(3), nearest_goal);
    if (nearest_goal.size() <= 0) {
        goals_tree.clearKDTreePoints();
        return false;
    }

    if ((nearest_goal - node->point.head(3)).norm() < tolerance) {
        eth_mav_msgs::EigenTrajectoryPoint trajectory_point_global;

        trajectory_point_global.position_W = node->point.head(3);
        trajectory_point_global.setFromYaw(node->point[3]);

        std::pair<double, double> result = segment_evaluator.computeGainOptimizedRaycasting(trajectory_point_global);

        node->gain = result.first;
        node->point[3] = result.second;

        if (node->gain < 0.1) {
            return false;
        }

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

    // Cost Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->cost > global_goals[i]->cost) {
            best_global_node = global_goals[i];
        }
    }

    /*// Gain Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->gain < global_goals[i]->gain) {
            best_global_node = global_goals[i];
        }
    }*/

    /*// Score Criteria
    for (int i = 0; i < global_goals.size(); ++i) {
        if (best_global_node->score < global_goals[i]->score) {
            best_global_node = global_goals[i];
        }
    }*/

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

    ROS_INFO("[AEPlanner]: Chosen Goal: [%f, %f, %f]", best_global_node->point[0], best_global_node->point[1], best_global_node->point[2]);
    ROS_INFO("[AEPlanner]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_node->gain, best_global_node->cost, best_global_node->score);

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

double AEPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint->position.x, waypoint->position.y, waypoint->position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
}

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
    //uav_state = *msg;
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
            AEP();
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
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

            if (next_best_node && next_best_node->parent) {
                previous_node = std::make_shared<rrt_star::Node>(*next_best_node->parent);
            }

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

                if (dist < 0.8 && yaw_difference < 0.4) {
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

void AEPlanner::visualize_frustum(std::shared_ptr<rrt_star::Node> position) {
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

void AEPlanner::visualize_unknown_voxels(std::shared_ptr<rrt_star::Node> position) {
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point_visualize;
    trajectory_point_visualize.position_W = position->point.head(3);
    trajectory_point_visualize.setFromYaw(position->point[3]);

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
