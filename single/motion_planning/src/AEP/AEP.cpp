#include "motion_planning/AEP/AEP.h"
#include "motion_planning/planner_helpers.h"

AEP::AEP(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {
    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "AEP");

    // Namespace
    param_loader.loadParam("uav_namespace", ns);

    // Frames, Coordinates and Dimensions
    param_loader.loadParam("frame_id", frame_id);
    param_loader.loadParam("body/frame_id", body_frame_id);
    param_loader.loadParam("camera/frame_id", camera_frame_id);

    // Bounded Box for Sampling
    if (!loadEnvironmentRegion(nh_private_, "planning_box", min_x, max_x, min_y, max_y, min_z, max_z)) {
        ros::shutdown();
        return;
    }

    // RRT Tree
    param_loader.loadParam("local_planning/N_max", N_max);
    param_loader.loadParam("local_planning/N_termination", N_termination);
    param_loader.loadParam("local_planning/N_yaw_samples", num_yaw_samples);
    param_loader.loadParam("local_planning/radius", radius);
    param_loader.loadParam("local_planning/step_size", step_size);
    param_loader.loadParam("local_planning/min_edge_length", min_edge_length_, 0.2);
    param_loader.loadParam("local_planning/tolerance", tolerance);
    param_loader.loadParam("local_planning/g_zero", g_zero);

    // RRT* Tree (global Planning)
    param_loader.loadParam("global_planning/N_min_nodes", N_min_nodes);
    param_loader.loadParam("global_planning/selection", global_selection, std::string("score"));

    // Gain Evaluation
    param_loader.loadParam("evaluation/marginal_gain", marginal_gain, true);
    param_loader.loadParam("evaluation/compute", eval_compute, std::string("gpu"));
    param_loader.loadParam("evaluation/marginal_split", marginal_split, false);
    param_loader.loadParam("evaluation/objective", objective_, std::string("expdecay"));
    param_loader.loadParam("benchmark/enabled", benchmark_mode, false);
    param_loader.loadParam("benchmark/suite", bench_suite_, std::string("timing"));

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
    transformer_ = std::make_unique<mrs_lib::Transformer>("AEP");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    set_variables = false;
    goto_global_planning = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();
    // Edge Collision Checker
    RRTStar.setEdgeCollisionChecker(
        [this](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return isEdgeCollisionFree(a, b); });

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
    shopts.node_name          = "AEP";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &AEP::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &AEP::callbackControlManagerDiag, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &AEP::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &AEP::callbackStop, this);

    /* Service Clients */
    sc_best_node = mrs_lib::ServiceClientHandler<cache_nodes::BestNode>(nh_private_, "best_node_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &AEP::timerMain, this);

    is_initialized = true;
}

double AEP::getMapDistance(const Eigen::Vector3d& position) const {
    return planner_helpers::getMapDistance(voxblox_server_, position);
}

bool AEP::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const {
    return planner_helpers::isPathCollisionFree(voxblox_server_, path, uav_radius);
}

bool AEP::isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
    return planner_helpers::isEdgeCollisionFree(voxblox_server_, from, to, uav_radius, collision_check_resolution_, optimistic_edges_);
}

void AEP::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[AEP]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

void AEP::planStep() {
    if (benchmark_mode) {
        bench_ = {};
    }

    goto_global_planning = false;

    localPlannerGPU();
    if (goto_global_planning) {
        // Clear variables from possible previous iterations
        best_global_node = nullptr;
        GlobalFrontiers.clear();

        // Compute the Global frontier and its path
        ROS_INFO("[AEP]: Getting Global Frontiers");
        getGlobalFrontiers(GlobalFrontiers);
        if (GlobalFrontiers.size() == 0) {
            changeState(STATE_STOPPED);
            return;
        }
        ROS_INFO("[AEP]: Planning Path to Global Frontiers");
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

    if (benchmark_mode) {
        planner_helpers::logBenchSummary(bench_);
    }
}

bool AEP::inBoundingBox(const Eigen::Vector4d& p) const {
    return planner_helpers::inBoundingBox(p, min_x, max_x, min_y, max_y, min_z, max_z);
}

// Add a Sampled Node
rrt_star::Node* AEP::expandTreeNode(rrt_star::Node* root_ptr) {
    Eigen::Vector3d rand_point;
    RRTStar.computeSamplingDimensions(bounded_radius, rand_point);
    rand_point += root_ptr->point.head(3);

    rrt_star::Node* nearest_node = nullptr;
    RRTStar.findNearestKD(rand_point, nearest_node);
    std::unique_ptr<rrt_star::Node> new_node;
    RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node, false, min_edge_length_);

    if (!inBoundingBox(new_node->point)) {
        return nullptr;
    }

    std::vector<rrt_star::Node*> segment = {new_node.get()};
    if (!isPathCollisionFree(segment)) {
        collision_id_counter_++;
        return nullptr;
    }
    if (!isEdgeCollisionFree(nearest_node->point.head<3>(), new_node->point.head<3>())) {
        collision_id_counter_++;
        return nullptr;
    }

    new_node->gain = 0.0;
    new_node->score = 0.0;
    segment_evaluator.computeCost(new_node.get());
    return RRTStar.addKDTreeNode(std::move(new_node));
}

void AEP::localPlannerGPU() {
    best_score_ = 0;
    rrt_star::Node* best_node = nullptr;
    int j = 1;
    auto tree_t0 = std::chrono::high_resolution_clock::now();
    ROS_INFO("[AEP]: Start Expanding Local");

    // Tree Root
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

    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);
    pub_gpu_debug.publish(segment_evaluator.visualizeGpuMap(flat_map_, map_origin_, map_dim_));

    // Reuse Previous Best Branch
    if (best_branch.size() > 1) {
        std::vector<rrt_star::Node*> branch_candidates;

        bool isFirstIteration = true;
        for (size_t i = 1; i < best_branch.size(); ++i) {
            if (isFirstIteration) {
                isFirstIteration = false;
                continue;
            }

            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(best_branch[i]->point.head(3), nearest_node_best);
            auto new_node_best = std::make_unique<rrt_star::Node>(best_branch[i]->point);
            new_node_best->parent = nearest_node_best;
            segment_evaluator.computeCost(new_node_best.get());
            branch_candidates.push_back(RRTStar.addKDTreeNode(std::move(new_node_best)));
        }

        if (!branch_candidates.empty()) {
            evaluateGains(branch_candidates);
            for (rrt_star::Node* node : branch_candidates) {
                segment_evaluator.computeScore(node, lambda);
                if (node->score > best_score_) {
                    best_score_ = node->score;
                    best_node = node;
                }
            }
            j += branch_candidates.size();
        }
    }
    best_branch.clear();

    // Batched RRT Expansion
    const int BATCH_SIZE = 2 * N_max;
    collision_id_counter_ = 0;
    ros::WallTime plan_start_ = ros::WallTime::now();

    while (j < N_max || best_score_ <= g_zero) {
        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int current_batch_cap = std::min(BATCH_SIZE, nodes_needed);
        if (current_batch_cap <= 0) {
            break;
        }

        // Backtrack When Stuck
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in  = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) {
                executed_path_.pop_back();
            }
            if (!executed_path_.empty()) {
                cacheHighGainNodes();
                retreating_ = true;
                ROS_WARN("[AEP]: Backtracking (%s, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", j, executed_path_.size());
                best_branch.clear();
                return;
            }
            rotate();
            plan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        std::vector<rrt_star::Node*> batch_nodes;
        batch_nodes.reserve(current_batch_cap);
        for (int k = 0; k < current_batch_cap && j <= N_termination; ++k) {
            // Stop When Stuck
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - plan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && j < recovery_min_tree_) || e > recovery_timeout_) {
                    break;
                }
            }
            rrt_star::Node* added_node = expandTreeNode(root_ptr);
            if (!added_node) {
                k--;
                continue;
            }
            batch_nodes.push_back(added_node);
            j++;
        }

        if (batch_nodes.empty()) {
            continue;
        }

        // Evaluate New Batch
        std::vector<rrt_star::Node*> score_nodes = batch_nodes;
        std::vector<rrt_star::Node*> gain_nodes  = batch_nodes;

        evaluateGains(gain_nodes);
        if (benchmark_mode) {
            benchmarkGains(gain_nodes);
        }

        for (rrt_star::Node* node : score_nodes) {
            segment_evaluator.computeScore(node, lambda);
            if (node->score > best_score_) {
                best_score_ = node->score;
                best_node = node;
            }
        }
        visualize_tree(collectTreeNodes(), ns);

        // Switch to Global Planner
        if (j >= N_termination) {
            logTreeNodes();
            cacheHighGainNodes();
            ROS_INFO("[AEP]: Going to Global Planning");
            RRTStar.clearKDTree();
            best_branch.clear();
            clearMarkers();
            goto_global_planning = true;
            return;
        }
    }

    logTreeNodes();
    cacheHighGainNodes();

    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathAEP(best_node, best_branch);
        visualize_path(best_node, ns);
    }

    if (!benchmark_mode) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        if (best_node) {
            ROS_INFO("[AEP]: Chosen node [%.2f, %.2f, %.2f] score=%.3f gain=%.3f | local tree computed in %.1f ms",
                     best_node->point[0], best_node->point[1], best_node->point[2], best_node->score, best_node->gain, tree_ms);
        } else {
            ROS_INFO("[AEP]: No node chosen | local tree computed in %.1f ms", tree_ms);
        }
    }

    next_best_node = best_branch[1].get();
}

void AEP::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    GainEvaluator::GainConfig cfg{marginal_gain, /*optimize_yaw=*/true, eval_compute, marginal_split, /*track_absolute=*/true};
    float marg_ms = 0.0f, abs_ms = 0.0f;
    segment_evaluator.evaluateGains(nodes, flat_map_, cfg, marg_ms, abs_ms);
    bench_kernel_ms_ = marg_ms;
}

std::vector<rrt_star::Node*> AEP::collectTreeNodes() {
    return planner_helpers::collectTreeNodes(RRTStar);
}

// Path Sum from Root
std::unordered_map<rrt_star::Node*, double> AEP::pathUnion(rrt_star::Node* root_ptr, bool use_marginal) {
    std::vector<rrt_star::Node*> tree = collectTreeNodes();
    rrt_star::sortByDepth(tree);
    std::unordered_map<rrt_star::Node*, double> path_sum{{root_ptr, 0.0}};
    for (rrt_star::Node* n : tree) {
        path_sum[n] = path_sum[n->parent] + (use_marginal ? n->gain : n->absolute_gain);
    }
    return path_sum;
}

void AEP::cacheHighGainNodes() {
    for (const auto& up : RRTStar.getNodes()) {
        rrt_star::Node* n = up.get();
        if (!n->parent) {
            continue;
        }
        if (n->absolute_gain > g_zero) {
            cacheNode(n, n->absolute_gain, n->absolute_yaw);
        }
    }
}

// Log Final Tree
void AEP::logTreeNodes() {
    if (benchmark_mode) {
        return;
    }
    planner_helpers::logTreeNodes(RRTStar, lambda);
}

// Run Benchmark Suite
void AEP::benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase) {
    planner_helpers::runBenchSuite(segment_evaluator, nodes, flat_map_, bench_, bench_suite_,
                                   /*optimize_yaw=*/true, marginal_split, iteration_, phase);
}

void AEP::globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node*& best_global_node) {
    if (GlobalFrontiers.size() == 0) {
        ROS_INFO("[AEP]: Terminate AEP");

        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);

        return;
    }

    ROS_INFO("[AEP]: Start Expanding Global");

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

    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    std::vector<rrt_star::Node*> all_global_goals;
    std::vector<rrt_star::Node*> frontier_nodes;
    collision_id_counter_ = 0;
    int m = 0;
    const int GLOBAL_BATCH = 2 * N_min_nodes;
    ros::WallTime gplan_start_ = ros::WallTime::now();

    // Frontier KD Tree
    goals_tree.clearKDTreePoints();
    goals_tree.initializeKDTreeWithPoints(GlobalFrontiers);

    while (m < N_min_nodes || all_global_goals.empty()) {
        const double gplan_elapsed = (ros::WallTime::now() - gplan_start_).toSec();
        const bool g_boxed = gplan_elapsed > recovery_boxed_deadline_ && m < recovery_min_tree_;
        const bool g_timed = gplan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (g_boxed || g_timed)) {
            if (!executed_path_.empty()) {
                executed_path_.pop_back();
            }
            if (!executed_path_.empty()) {
                retreating_ = true;
                backtrack = true;
                ROS_WARN("[AEP]: Global backtracking (%s, tree=%d) -> executed node %zu",
                         g_boxed ? "boxed-in" : "timeout", m, executed_path_.size());
                return;
            }
            ROS_INFO("[AEP]: Backtrack Rotation");
            rotate();
            gplan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        int cap = std::min(GLOBAL_BATCH, (m < N_min_nodes) ? (N_min_nodes - m) : GLOBAL_BATCH);
        std::vector<rrt_star::Node*> batch_new, batch_frontier;
        for (int b = 0; b < cap; ++b) {
            // Stop When Stuck
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - gplan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && m < recovery_min_tree_) || e > recovery_timeout_) {
                    break;
                }
            }

            Eigen::Vector3d rand_point_star;
            RRTStar.computeSamplingDimensions(bounded_radius, rand_point_star);
            rand_point_star += root_ptr->point.head(3);

            rrt_star::Node* nearest_node_star = nullptr;
            RRTStar.findNearestKD(rand_point_star, nearest_node_star);
            std::unique_ptr<rrt_star::Node> new_node_star;
            RRTStar.steer_parent(nearest_node_star, rand_point_star, step_size, new_node_star, false, min_edge_length_);

            // Stay Inside Bounded Box
            if (!inBoundingBox(new_node_star->point)) {
                b--;
                continue;
            }

            std::vector<rrt_star::Node*> segment_star = {new_node_star.get()};
            if (!isPathCollisionFree(segment_star)) {
                b--;
                continue;
            }
            if (!isEdgeCollisionFree(nearest_node_star->point.head<3>(), new_node_star->point.head<3>())) {
                b--;
                continue;
            }

            // Choose Parent and Rewire
            std::vector<rrt_star::Node*> nearby_nodes_star;
            RRTStar.findNearbyKD(new_node_star.get(), radius, nearby_nodes_star);
            if (nearby_nodes_star.empty()) {
                nearby_nodes_star.push_back(nearest_node_star);
            }
            RRTStar.chooseParent(new_node_star.get(), nearby_nodes_star);
            if (!new_node_star->parent) {
                b--;
                continue;
            }
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

        // Nodes to Evaluate
        std::vector<rrt_star::Node*> gain_nodes;
        if (marginal_gain) {
            gain_nodes = collectTreeNodes();
        } else if (eval_compute == "gpu") {
            gain_nodes = batch_new;
        } else {
            gain_nodes = batch_frontier;
        }

        evaluateGains(gain_nodes);
        if (benchmark_mode) {
            benchmarkGains(gain_nodes, "global");
        }

        // Qualify Frontier Goals
        all_global_goals.clear();
        for (rrt_star::Node* f : frontier_nodes) {
            if (f->absolute_gain >= 0.1) {
                all_global_goals.push_back(f);
            }
        }
    }

    // Score Goals
    const bool use_marginal = marginal_gain;
    ROS_INFO("[AEP]: Global scoring mode = %s (path-union * discount)", use_marginal ? "MARGINAL" : "ABSOLUTE");
    std::unordered_map<rrt_star::Node*, double> path_sum = pathUnion(root_ptr, use_marginal);
    for (rrt_star::Node* g : all_global_goals) {
        g->gain  = path_sum[g];
        g->score = (objective_ == "rate_L") ? (path_sum[g] / (g->cost < 0.1 ? 0.1 : g->cost))
                                            : (path_sum[g] * exp(-global_lambda * g->cost));
    }

    if (!benchmark_mode) {
        for (rrt_star::Node* g : all_global_goals) {
            ROS_INFO("[Goal] gain=%.3f score=%.3f", g->gain, g->score);
        }
    }

    ROS_INFO("[AEP]: Global Planner Ends");

    if (!benchmark_mode) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        ROS_INFO("[AEP]: Global tree computed in %.1f ms", tree_ms);
    }

    getBestGlobalPath(all_global_goals, best_global_node);
    all_global_goals.clear();
}

void AEP::getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers) {
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

bool AEP::getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node* node) {
    if (GlobalFrontiers.empty()) {
        return false;
    }

    Eigen::Vector3d nearest_goal;
    goals_tree.findNearestKDPoint(node->point.head(3), nearest_goal);

    return (nearest_goal.size() > 0 && (nearest_goal - node->point.head(3)).norm() < tolerance);
}

void AEP::getBestGlobalPath(const std::vector<rrt_star::Node*>& global_goals, rrt_star::Node*& best_global_node) {
    if (global_goals.size() == 0) {
        best_global_node = nullptr;
        return;
    }

    best_global_node = global_goals[0];

    // Pick Best Goal
    for (int i = 1; i < (int)global_goals.size(); ++i) {
        if (global_selection == "cost") {
            if (global_goals[i]->cost < best_global_node->cost) {
                best_global_node = global_goals[i];
            }
        } else if (global_selection == "gain") {
            if (global_goals[i]->gain > best_global_node->gain) {
                best_global_node = global_goals[i];
            }
        } else {
            if (global_goals[i]->score > best_global_node->score) {
                best_global_node = global_goals[i];
            }
        }
    }

    ROS_INFO("[AEP]: Chosen Goal: [%f, %f, %f]", best_global_node->point[0], best_global_node->point[1], best_global_node->point[2]);
    ROS_INFO("[AEP]: Chosen Goal Gain, Cost & Score: [%f, %f, %f]", best_global_node->gain, best_global_node->cost, best_global_node->score);

    visualize_path(best_global_node, ns);
}

void AEP::cacheNode(rrt_star::Node* Node, double gain, double yaw) {
    if (!Node) {
        return;
    }
    cache_nodes::Node cached_node;
    cached_node.gain = gain;
    cached_node.position.x = Node->point[0];
    cached_node.position.y = Node->point[1];
    cached_node.position.z = Node->point[2];
    cached_node.yaw = yaw;
    pub_node.publish(cached_node);
}

double AEP::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {
    return planner_helpers::distance(waypoint, pose);
}

void AEP::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[AEP]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(3).sleep();

    ROS_INFO("[AEP]: Rotating 360 degrees");

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

    ROS_INFO("[AEP]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(2).sleep();
}

void AEP::rotate() {
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

bool AEP::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[AEP]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;
}

bool AEP::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[AEP]: " << ss.str());

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

void AEP::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[AEP]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void AEP::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[AEP]: getting UavState diagnostics");
    geometry_msgs::Pose uav_state = msg->pose;
    double yaw = mrs_lib::getYaw(uav_state);
    pose = {uav_state.position.x, uav_state.position.y, uav_state.position.z, yaw};
}

void AEP::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[AEP]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[AEP]: main timer spinning");

    if (!set_variables) {
        // Wait for TF Tree
        ros::Duration(0.5).sleep();
        GetTransformation();
        ROS_INFO("[AEP]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[AEP]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEP]: tracker has goal");
            } else {
                ROS_INFO("[AEP]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEP]: tracker has goal");
            } else {
                ROS_INFO("[AEP]: waiting for command");
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
            // Optimistic Edges at Start
            optimistic_edges_ = (iteration_ < optimistic_iterations_);

            retreating_ = false;
            {
                ros::WallTime plan_t0 = ros::WallTime::now();
                planStep();
                total_planning_ms_ += (ros::WallTime::now() - plan_t0).toSec() * 1000.0;
            }
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            // Retreat to Previous Node
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

            // Retreat Waypoint
            if (waypoints_.empty() && next_best_node) {
                mrs_msgs::Reference ref;
                ref.position.x = next_best_node->point[0];
                ref.position.y = next_best_node->point[1];
                ref.position.z = next_best_node->point[2];
                ref.heading    = next_best_node->point[3];
                waypoints_.push_back(ref);
            }

            // Store Flown Path
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
                ROS_INFO("[AEP]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                const int wp_index = std::min(waypoint_index_, static_cast<int>(waypoints_.size()) - 1);
                const mrs_msgs::Reference& wp = waypoints_[wp_index];
                std::unique_ptr<mrs_msgs::Reference> wp_ptr = std::make_unique<mrs_msgs::Reference>(wp);

                double dist = distance(wp_ptr, current_pose);
                double yaw_difference = fabs(atan2(sin(wp.heading - current_yaw), cos(wp.heading - current_yaw)));
                ROS_INFO("[AEP]: WP %d/%zu: dist=%.2f, yaw=%.2f",
                         wp_index + 1,
                         waypoints_.size(),
                         dist, yaw_difference);

                if (waypoint_index_ == wp_index && dist < waypoint_reach_distance_ && yaw_difference < 0.4) {
                    waypoint_index_++;

                    if (waypoint_index_ >= waypoints_.size()) {
                        ROS_INFO("[AEP]: Going to final waypoint, waiting for tracker to finish.");
                        break;
                    }

                    mrs_msgs::ReferenceStamped next_ref;
                    next_ref.header.frame_id = ns + "/" + frame_id;
                    next_ref.header.stamp = ros::Time::now();
                    next_ref.reference = waypoints_[waypoint_index_];

                    pub_reference.publish(next_ref.reference);
                    pub_initial_reference.publish(next_ref);

                    current_waypoint_ = std::make_unique<mrs_msgs::Reference>(next_ref.reference);

                    ROS_INFO("[AEP]: Publishing next waypoint");
                }
            } else {
                ROS_INFO("[AEP]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[AEP]: Total Iterations: %d", iteration_);
            if (!stats_written_) {
                std::string log_dir;
                if (nh_private_.getParam("performance_log_dir", log_dir) && !log_dir.empty()) {
                    std::ofstream dl(log_dir + "/data_log.txt", std::ios::app);
                    if (dl.is_open()) {
                        dl << "total_planning_time_ms=" << total_planning_ms_ << " iterations=" << iteration_ << "\n";
                    }
                }
                stats_written_ = true;
            }
            ROS_INFO("[AEP]: Shutting down.");
            ros::shutdown();
            return;
        }
        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[AEP]: tracker has goal");
            } else {
                ROS_INFO("[AEP]: waiting for command");
            }
            break;
        }
    }
}

void AEP::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[AEP]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[AEP]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void AEP::visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns) {
    planner_helpers::visualize_tree(pub_markers, frame_id, ns, nodes);
}

void AEP::visualize_path(rrt_star::Node* node, const std::string& ns) {
    planner_helpers::visualize_path(pub_markers, frame_id, ns, node, path_id_counter_);
}

void AEP::visualize_frustum(rrt_star::Node* position) {
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

void AEP::visualize_unknown_voxels(rrt_star::Node* position) {
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

void AEP::clear_all_voxels() {
    planner_helpers::clear_all_voxels(pub_voxels);
}

void AEP::clearMarkers() {
    planner_helpers::clearMarkers(pub_markers, node_id_counter_, edge_id_counter_, path_id_counter_);
}
