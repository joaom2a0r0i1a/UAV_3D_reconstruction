#include "motion_planning/NBV/NBVplanner.h"
#include "motion_planning/planner_helpers.h"
#include <cstdlib>
#include <fstream>
#include <algorithm>

NBVPlanner::NBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {

    //ns = "uav1";

    /* Parameter loading */
    mrs_lib::ParamLoader param_loader(nh_private_, "NBVPlanner");

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
    param_loader.loadParam("rrt/N_max", N_max);
    param_loader.loadParam("rrt/N_termination", N_termination);
    param_loader.loadParam("rrt/N_yaw_samples", num_yaw_samples);
    param_loader.loadParam("rrt/radius", radius);
    param_loader.loadParam("rrt/step_size", step_size);
    param_loader.loadParam("rrt/min_edge_length", min_edge_length_, 0.2);
    param_loader.loadParam("rrt/fixed_step", fixed_step, false);
    param_loader.loadParam("rrt/execution_horizon", execution_horizon_, 1);
    param_loader.loadParam("rrt/tolerance", tolerance);
    param_loader.loadParam("rrt/rrt_star", local_rrt_star, false);

    param_loader.loadParam("evaluation/marginal_gain", marginal_gain, false);
    param_loader.loadParam("evaluation/optimize_yaw", optimize_yaw, false);
    param_loader.loadParam("evaluation/compute", eval_compute, std::string("cpu"));
    param_loader.loadParam("evaluation/marginal_split", marginal_split, false);
    param_loader.loadParam("evaluation/objective", objective_, std::string("expdecay"));

    // Benchmark / X2 timing
    param_loader.loadParam("benchmark/enabled", benchmark_mode, false);
    param_loader.loadParam("benchmark/timing_after_s", timing_after_s_, 600.0);
    param_loader.loadParam("benchmark/x2_max", x2_capture_max_, 10);

    // Camera
    param_loader.loadParam("camera/h_fov", horizontal_fov);
    param_loader.loadParam("camera/width", resolution_x);
    param_loader.loadParam("camera/height", resolution_y);
    param_loader.loadParam("camera/min_distance", min_distance);
    param_loader.loadParam("camera/max_distance", max_distance);

    // Planner
    param_loader.loadParam("path/uav_radius", uav_radius);
    param_loader.loadParam("path/collision_check_resolution", collision_check_resolution_, 0.1);
    param_loader.loadParam("path/optimistic_iterations", optimistic_iterations_, 5);
    param_loader.loadParam("path/recovery_enabled", recovery_enabled_, true);
    param_loader.loadParam("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    param_loader.loadParam("path/recovery_min_tree", recovery_min_tree_, 10);
    param_loader.loadParam("path/recovery_timeout", recovery_timeout_, 12.0);
    param_loader.loadParam("path/lambda", lambda);

    // Timer
    param_loader.loadParam("timer_main/rate", timer_main_rate);

    // Initialize UAV as state IDLE
    state_ = STATE_IDLE;
    iteration_ = 0;

    // Benchmark run state
    replan_count_ = 0;
    nbv_started_ = false;
    x2_timing_window_ = false;
    x2_capture_count_ = 0;

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
    transformer_ = std::make_unique<mrs_lib::Transformer>("NBVPlanner");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    set_variables = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();
    // Edges (not just nodes) must clear obstacles -> give the RRT* library a straight-segment collision test.
    RRTStar.setEdgeCollisionChecker(
        [this](const Eigen::Vector3d& a, const Eigen::Vector3d& b){ return isEdgeCollisionFree(a, b); });

    // Get Sampling Radius
    bounded_radius = sqrt(pow(min_x - max_x, 2.0) + pow(min_y - max_y, 2.0) + pow(min_z - max_z, 2.0));

    /* Publishers */
    pub_markers = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);
    pub_reference = nh_private_.advertise<mrs_msgs::Reference>("reference_out", 1);
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_initial_reference = nh_private_.advertise<mrs_msgs::ReferenceStamped>("initial_reference_out", 5);

    /* Subscribers */
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh                 = nh_private_;
    shopts.node_name          = "NBVPlanner";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.queue_size         = 10;
    shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

    sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &NBVPlanner::callbackUavState, this);
    sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &NBVPlanner::callbackControlManagerDiag, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &NBVPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &NBVPlanner::callbackStop, this);

    /* Service Clients */
    sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_private_, "trajectory_generation_out");
    sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &NBVPlanner::timerMain, this);

    is_initialized = true;
}

double NBVPlanner::getMapDistance(const Eigen::Vector3d& position) const { return planner_helpers::getMapDistance(voxblox_server_, position); }

bool NBVPlanner::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const { return planner_helpers::isPathCollisionFree(voxblox_server_, path, uav_radius); }

// Sample the segment; require clearance >= uav_radius at each point. When optimistic_edges_ (first replans),
// unknown space is traversable (block only observed-and-close) to bootstrap away from spawn. Otherwise unknown
// counts as blocked (getMapDistance returns 0 for unobserved) so the tree can't route through unmapped pockets.
bool NBVPlanner::isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const { return planner_helpers::isEdgeCollisionFree(voxblox_server_, from, to, uav_radius, collision_check_resolution_, optimistic_edges_); }

void NBVPlanner::GetTransformation() {
    // From Body Frame to Camera Frame
    auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
    if (!Message_C_B) {
        ROS_ERROR_THROTTLE(1.0, "[NBVPlanner]: could not get transform from body frame to the camera frame!");
        return;
    }

    T_C_B_message = Message_C_B.value();
    T_B_C_message = transformer_->inverse(T_C_B_message);

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);
    segment_evaluator.setCameraExtrinsics(T_C_B);
}

std::vector<float> NBVPlanner::parentCamRows(float yaw) { return segment_evaluator.parentCamRows(yaw); }

// Evaluate node gains per (marginal_gain, eval_compute), always at each node's fixed yaw.
void NBVPlanner::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    // Shared gain pipeline (core/rrt_construction); RH-NBVP does not track absolute gain.
    GainEvaluator::GainConfig cfg{marginal_gain, optimize_yaw, eval_compute, marginal_split, /*track_absolute=*/false};
    segment_evaluator.evaluateGains(nodes, flat_map_, cfg, last_marg_kernel_ms_, last_abs_kernel_ms_);
}

double NBVPlanner::computeSingleParentGainGPU(rrt_star::Node* node) {
    const int per = segment_evaluator.depthImagePixels();
    rrt_star::Node* p = node->parent;
    double p_yaw = p ? p->point[3] : 0.0;
    std::vector<float> R = parentCamRows((float)p_yaw);
    Eigen::Vector3d p_pos = p ? p->point.head(3) : node->point.head(3);
    std::vector<float> p_depth;
    if (p && (int)p->depth_buffer.size() == per) p_depth = p->depth_buffer;
    else p_depth.assign((size_t)per, -1.0f);
    std::vector<float> out;
    auto r = segment_evaluator.computeSingleParentMarginalGainGPU(
        node->point.x(), node->point.y(), node->point.z(), p_pos, p_yaw, R, p_depth, out, node->point[3]);
    return r.first;
}

// Order nodes shallow-first so cumulative scoring sees each parent before its children.
void NBVPlanner::sortByDepth(std::vector<rrt_star::Node*>& nodes) { segment_evaluator.sortByDepth(nodes); }

std::vector<rrt_star::Node*> NBVPlanner::collectTreeNodes() { return planner_helpers::collectTreeNodes(RRTStar); }

// Per-node score dump over the final tree (once), so multi-batch runs don't re-log each batch.
void NBVPlanner::logTreeNodes() { if (benchmark_mode) return; planner_helpers::logTreeNodes(RRTStar, lambda); }

// Time every gain variant (G_all / single-parent / absolute, GPU vs CPU) on the same nodes at
// their fixed yaw, then dump per-node gains to the X1 CSV. Restores each node's gain when done.
void NBVPlanner::benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();

    auto snapshot_gains = [&]() {
        std::vector<double> g(n);
        for (size_t i = 0; i < n; ++i) g[i] = nodes[i]->gain;
        return g;
    };

    const std::vector<double> saved_gain    = snapshot_gains();
    const bool                saved_marginal = marginal_gain;
    const std::string         saved_compute  = eval_compute;
    const bool                saved_split    = marginal_split;

    auto time_eval = [&](bool marginal, const std::string& compute, bool split) {
        marginal_gain = marginal; eval_compute = compute; marginal_split = split;
        auto t0 = std::chrono::high_resolution_clock::now();
        evaluateGains(nodes);
        return std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count();
    };

    double t_gall_gpu        = time_eval(true,  "gpu", false);
    double t_gall_gpu_kernel = last_marg_kernel_ms_;
    std::vector<double> g_gall_gpu = snapshot_gains();

    double t_gall_split = time_eval(true, "gpu", true);
    std::vector<double> g_gall_split = snapshot_gains();

    double t_g1p_cpu_hashmap = time_eval(true, "cpu", false);   // legacy HashMap pass, console summary only

    double t_abs_gpu        = time_eval(false, "gpu", false);
    double t_abs_gpu_kernel = last_abs_kernel_ms_;
    std::vector<double> g_abs_gpu = snapshot_gains();

    double t_abs_cpu = time_eval(false, "cpu", false);
    std::vector<double> g_abs_cpu = snapshot_gains();

    bench_ms_gall_gpu   += t_gall_gpu;
    bench_ms_gall_split += t_gall_split;
    bench_ms_g1p_cpu    += t_g1p_cpu_hashmap;
    bench_ms_abs_gpu    += t_abs_gpu;
    bench_ms_abs_cpu    += t_abs_cpu;
    bench_kernel_gall_gpu_ += t_gall_gpu_kernel;
    bench_kernel_abs_gpu_  += t_abs_gpu_kernel;
    bench_nodes += (int)n;

    // Per-replan samples for mean+/-std. transfer_ms = total wall time - device kernel time.
    ROS_INFO("[X2rep] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_gall_gpu, t_gall_gpu_kernel, t_gall_gpu - t_gall_gpu_kernel);
    ROS_INFO("[X2repABS] nodes=%zu total_ms=%.3f gain_computation_ms=%.3f cpu_to_gpu_transfer_ms=%.3f",
             n, t_abs_gpu, t_abs_gpu_kernel, t_abs_gpu - t_abs_gpu_kernel);

    marginal_gain = saved_marginal; eval_compute = saved_compute; marginal_split = saved_split;
    for (size_t i = 0; i < n; ++i) nodes[i]->gain = saved_gain[i];

    // CPU marginal baselines, depth-sequential so each node subtracts its ancestors' committed views.
    // G_single-parent and G_all are separate passes; observed sets are cleared before each.
    std::vector<rrt_star::Node*> depth_nodes = nodes;
    sortByDepth(depth_nodes);
    rrt_star::Node* tree_root = depth_nodes.empty() ? nullptr : depth_nodes.front();
    while (tree_root && tree_root->parent) tree_root = tree_root->parent;
    
    auto clear_observed = [&]() {
        if (tree_root) tree_root->observed_unknown_voxels.clear();
        for (rrt_star::Node* nd : depth_nodes) nd->observed_unknown_voxels.clear();
    };

    clear_observed();
    auto t0_g1p = std::chrono::high_resolution_clock::now();
    for (rrt_star::Node* nd : depth_nodes) {
        segment_evaluator.computeMarginalGainCPU_AllAncestors(flat_map_, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/true, /*commit_observed=*/true);
    }
    double t_g1p_cpu = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0_g1p).count();

    std::unordered_map<rrt_star::Node*, double> gall_of;
    clear_observed();
    auto t0_gall = std::chrono::high_resolution_clock::now();
    for (rrt_star::Node* nd : depth_nodes) {
        gall_of[nd] = segment_evaluator.computeMarginalGainCPU_AllAncestors(flat_map_, nd, optimize_yaw ? NAN : nd->point[3], /*one_parent_only=*/false, /*commit_observed=*/true).first;
    }
    double t_gall_cpu = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0_gall).count();

    std::vector<double> g_gall_cpu(n);
    for (size_t i = 0; i < n; ++i) g_gall_cpu[i] = gall_of[nodes[i]];   // back to input order for the CSV

    ROS_INFO("[X2cpu] nodes=%zu cpu_absolute_ms=%.3f cpu_gain_1parent_ms=%.3f", n, t_abs_cpu, t_g1p_cpu);
    ROS_INFO("[X1cpu] nodes=%zu cpu_gain_all_ms=%.3f", n, t_gall_cpu);

    // Per-node dump of all 6 gain variants -> CSV. cols: replan,depth,abs_cpu,abs_gpu,p1_cpu,p1_gpu,all_cpu,all_gpu
    const char* x1_csv_path = std::getenv("NBV_X1_CSV");
    std::ofstream x1;
    if (x1_csv_path) x1.open(x1_csv_path, std::ios::app);
    for (size_t i = 0; i < n; ++i) {
        rrt_star::Node* nd = nodes[i];
        double g1p_gpu = computeSingleParentGainGPU(nd);
        double saved = nd->gain;
        if (nd->parent && nd->parent->parent) segment_evaluator.populateParentHistory(flat_map_, nd->parent);
        double g1p_cpu = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, nd, nd->point[3]).first;
        nd->gain = saved;

        double err = std::fabs(g1p_gpu - g1p_cpu);
        bench_g1p_err_sum += err;
        if (err > bench_g1p_err_max) bench_g1p_err_max = err;

        if (x1.is_open()) {
            int depth = 0; for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) ++depth;
            x1 << replan_count_ << ',' << depth << ',' << g_abs_cpu[i] << ',' << g_abs_gpu[i] << ','
               << g1p_cpu << ',' << g1p_gpu << ',' << g_gall_cpu[i] << ',' << g_gall_gpu[i] << '\n';
        } else {
            ROS_INFO("[BENCH][%s] gall_gpu=%7.3f split=%7.3f | g1p_gpu=%7.3f g1p_cpu=%7.3f err=%.4f | abs_gpu=%7.3f abs_cpu=%7.3f",
                     phase, g_gall_gpu[i], g_gall_split[i], g1p_gpu, g1p_cpu, err, g_abs_gpu[i], g_abs_cpu[i]);
        }
    }
}

void NBVPlanner::NBV() {
    best_score_ = 0;
    ++replan_count_;
    // Only benchmark once sim-time >= threshold, so early collision-heavy replans don't skew timings.
    double sim_now = ros::Time::now().toSec();
    bool x2_was_open = x2_timing_window_;
    x2_timing_window_ = (sim_now >= timing_after_s_);
    if (x2_timing_window_ && !x2_was_open)
        ROS_WARN("[X2] timing window OPEN at sim_t=%.1fs (threshold=%.1fs, replan=%d)", sim_now, timing_after_s_, replan_count_);
    // One capture = one whole replan (both phases); the cap counts replans, incremented at the end.
    bool x2_do_capture = benchmark_mode && x2_timing_window_ && (x2_capture_count_ < x2_capture_max_);
    rrt_star::Node* best_node = nullptr;
    if (benchmark_mode) {
        bench_ms_gall_gpu = bench_ms_gall_split = bench_ms_g1p_cpu = bench_ms_abs_gpu = bench_ms_abs_cpu = 0.0;
        bench_g1p_err_sum = bench_g1p_err_max = 0.0; bench_nodes = 0;
        bench_kernel_gall_gpu_ = bench_kernel_abs_gpu_ = 0.0;
    }
    auto tree_t0 = std::chrono::high_resolution_clock::now();

    std::unique_ptr<rrt_star::Node> root;
    if (current_waypoint_) {
        root = std::make_unique<rrt_star::Node>(next_start);
    } else if (best_branch.size() > 1) {
        root = std::make_unique<rrt_star::Node>(prev_best_branch[1]);
    } else {
        root = std::make_unique<rrt_star::Node>(pose);
    }
    root->cost = 0;

    RRTStar.clearKDTree();
    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));
    clearMarkers();

    // Cache the map (needed for the fixed-yaw gpu + cpu-flatmap eval).
    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    root_ptr->depth_buffer.clear();
    best_score_ = root_ptr->score;
    best_node = root_ptr;
    visualize_node(root_ptr->point, ns);

    int j = 1;

    // PHASE A: re-add the UN-executed remainder of the previous best branch as a fixed chain. The drone
    // executed exec_horizon_limit_ steps last plan, so the new root is prev_best_branch[exec_horizon_limit_];
    // re-add from the next node (exec_horizon_limit_+1). For horizon 1 this is index 2 (the original value).
    const size_t reAddStart = (size_t)exec_horizon_limit_ + 1;
    if (prev_best_branch.size() > reAddStart) {
        std::vector<rrt_star::Node*> branch_candidates;
        for (size_t i = reAddStart; i < prev_best_branch.size(); ++i) {
            const Eigen::Vector4d& node_position = prev_best_branch[i];
            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(node_position.head(3), nearest_node_best);
            auto new_node_best = std::make_unique<rrt_star::Node>(node_position);
            new_node_best->parent = nearest_node_best;
            segment_evaluator.computeCost(new_node_best.get());
            rrt_star::Node* added = RRTStar.addKDTreeNode(std::move(new_node_best));
            branch_candidates.push_back(added);
        }
        if (!branch_candidates.empty()) {
            evaluateGains(branch_candidates);
            if (x2_do_capture) benchmarkGains(branch_candidates);
            for (rrt_star::Node* node : branch_candidates) {
                segment_evaluator.computeScore(node, lambda);
                if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
            }
            j += (int)branch_candidates.size();
        }
    }
    prev_best_branch.clear();
    best_branch.clear();

    // PHASE B: batched RRT/RRT* expansion with fixed-yaw gain.
    const int BATCH_SIZE = 2 * N_max;
    collision_id_counter_ = 0;
    bool terminated = false;

    // Full-algorithm timing (tree creation + gain eval + scoring), independent of the benchmark passes.
    double x2_tree_ms = 0.0, x2_eval_ms = 0.0, x2_score_ms = 0.0, x2_kernel_ms = 0.0;

    ros::WallTime plan_start_ = ros::WallTime::now();   // bound the tree build so NBV() can never spin (single-threaded timer)

    while (j < N_max || best_score_ == 0.0) {
        // In-planner backtrack (wall-clock bounded; replaces the old collisions>10000*j cap that could spin
        // for millions of tries). boxed-in = tree still tiny after a short deadline; timed-out = hard cap.
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in  = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) executed_path_.pop_back();   // remove the node we're on
            if (!executed_path_.empty()) {
                retreating_ = true;                                  // timerMain flies back to executed_path_.back()
                logTreeNodes();
                ROS_WARN("[NBVPlanner]: Backtracking (%s after %.1fs, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", plan_elapsed, j, executed_path_.size());
                best_branch.clear();
                prev_best_branch.clear();
                return;
            } else {
                ROS_INFO("[NBVPlanner]: Backtrack Rotation");
                rotate();
                plan_start_ = ros::WallTime::now();
                collision_id_counter_ = 0;
            }
        }

        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int cap = std::min(BATCH_SIZE, nodes_needed);
        if (cap <= 0) break;

        std::vector<rrt_star::Node*> batch_nodes;
        auto x2_tree0 = std::chrono::high_resolution_clock::now();
        for (int k = 0; k < cap && j <= N_termination; ++k) {
            // Boxed-in guard: when every sample collides, k-- keeps this inner loop spinning and the
            // outer WallTime check above is never reached. Bail to the outer loop so the backtrack fires.
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - plan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && j < recovery_min_tree_) || e > recovery_timeout_) break;
            }
            Eigen::Vector4d rand_point_yaw;
            RRTStar.computeSamplingDimensionsNBV(bounded_radius, rand_point_yaw);
            Eigen::Vector3d rand_point = rand_point_yaw.head(3) + root_ptr->point.head(3);

            rrt_star::Node* nearest_node = nullptr;
            RRTStar.findNearestKD(rand_point, nearest_node);
            std::unique_ptr<rrt_star::Node> new_node;
            RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node, fixed_step, min_edge_length_);

            if (new_node->point[0] > max_x || new_node->point[0] < min_x ||
                new_node->point[1] < min_y || new_node->point[1] > max_y ||
                new_node->point[2] < min_z || new_node->point[2] > max_z) { k--; continue; }

            std::vector<rrt_star::Node*> seg = {new_node.get()};
            if (!isPathCollisionFree(seg)) { collision_id_counter_++; k--; continue; }
            // Endpoints are free; also require the whole PARENT EDGE to be clear (nodes-only misses walls between them).
            if (!isEdgeCollisionFree(nearest_node->point.head<3>(), new_node->point.head<3>())) { collision_id_counter_++; k--; continue; }

            new_node->point[3] = rand_point_yaw[3];   // random sampled yaw (kept as-is unless optimize_yaw re-picks it in evaluateGains)
            new_node->gain = 0.0; new_node->score = 0.0; new_node->cum_gain = 0.0;

            rrt_star::Node* added_node;
            if (local_rrt_star) {
                std::vector<rrt_star::Node*> nearby;
                RRTStar.findNearbyKD(new_node.get(), radius, nearby);
                if (nearby.empty()) nearby.push_back(nearest_node);
                RRTStar.chooseParent(new_node.get(), nearby);
                if (!new_node->parent) { collision_id_counter_++; k--; continue; }  // every candidate edge blocked
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
                RRTStar.rewire(added_node, nearby, radius);
            } else {
                segment_evaluator.computeCost(new_node.get());
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
            }
            batch_nodes.push_back(added_node);
            j++;
        }
        x2_tree_ms += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - x2_tree0).count();

        if (batch_nodes.empty()) continue;

        // Fixed yaw is tree-independent, so RRT scores only the new batch. RRT* rewires -> rescore the
        // whole tree (costs changed); recompute marginal gains too (ancestry changed), absolute does not.
        std::vector<rrt_star::Node*> score_nodes = batch_nodes;
        std::vector<rrt_star::Node*> gain_nodes  = batch_nodes;
        if (local_rrt_star) {
            score_nodes = collectTreeNodes();
            sortByDepth(score_nodes);
            best_score_ = root_ptr->score; best_node = root_ptr;
            if (marginal_gain) gain_nodes = score_nodes;
        }

        auto x2_eval0 = std::chrono::high_resolution_clock::now();
        evaluateGains(gain_nodes);
        x2_eval_ms += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - x2_eval0).count();
        
        if (eval_compute == "gpu" && marginal_gain) x2_kernel_ms += last_marg_kernel_ms_;
        if (x2_do_capture) benchmarkGains(gain_nodes);

        auto x2_score0 = std::chrono::high_resolution_clock::now();
        for (rrt_star::Node* node : score_nodes) {
            segment_evaluator.computeScore(node, lambda);
            if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
        }
        x2_score_ms += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - x2_score0).count();
        visualize_tree(collectTreeNodes(), ns);

        if (j >= N_termination) { terminated = true; break; }
    }

    // Per-replan full-algorithm timing, logged only inside the timing window.
    if (benchmark_mode && x2_timing_window_) {
        double x2_full_ms = x2_tree_ms + x2_eval_ms + x2_score_ms;
        ROS_INFO("[X2full] nodes=%zu tree_construction_ms=%.3f gain_evaluation_ms=%.3f scoring_ms=%.3f full_algorithm_ms=%.3f gain_computation_ms=%.3f",
                 RRTStar.getNodes().size(), x2_tree_ms, x2_eval_ms, x2_score_ms, x2_full_ms, x2_kernel_ms);
    }
    if (x2_do_capture) ++x2_capture_count_;

    logTreeNodes();

    if (benchmark_mode && bench_nodes > 0) {
        double bn = (double)bench_nodes;
        ROS_INFO("\n=== NBVP GAIN BENCHMARK (fixed yaw, %d nodes) ===\n"
                 "marginal-gpu-G_all : %9.3f ms | %7.4f ms/node  total  [X2 ONLINE]\n"
                 "  |- gain computation : %9.3f ms | %7.4f ms/node  (CPU->GPU transfer = %9.3f ms)\n"
                 "marginal-gpu-split : %9.3f ms | %7.4f ms/node\n"
                 "marginal-cpu-1parent : %9.3f ms | %7.4f ms/node\n"
                 "absolute-gpu       : %9.3f ms | %7.4f ms/node  total\n"
                 "  |- gain computation : %9.3f ms | %7.4f ms/node  (CPU->GPU transfer = %9.3f ms)\n"
                 "absolute-cpu       : %9.3f ms | %7.4f ms/node\n"
                 "v2(gpu 1-parent) vs cpu-hash: mean err %.4f | max err %.4f\n"
                 "==================================================",
                 bench_nodes,
                 bench_ms_gall_gpu, bench_ms_gall_gpu/bn,
                 bench_kernel_gall_gpu_, bench_kernel_gall_gpu_/bn, bench_ms_gall_gpu - bench_kernel_gall_gpu_,
                 bench_ms_gall_split, bench_ms_gall_split/bn,
                 bench_ms_g1p_cpu, bench_ms_g1p_cpu/bn,
                 bench_ms_abs_gpu, bench_ms_abs_gpu/bn,
                 bench_kernel_abs_gpu_, bench_kernel_abs_gpu_/bn, bench_ms_abs_gpu - bench_kernel_abs_gpu_,
                 bench_ms_abs_cpu, bench_ms_abs_cpu/bn, bench_g1p_err_sum/bn, bench_g1p_err_max);
    }

    if (terminated) {
        ROS_INFO("[NBVPlanner]: RH-NBVP Terminated");
        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);
        return;
    }

    if (!benchmark_mode && best_node) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        ROS_INFO("[NBVPlanner]: Chosen node [%.2f, %.2f, %.2f] score=%.3f gain=%.3f | tree computed in %.1f ms",
                 best_node->point[0], best_node->point[1], best_node->point[2], best_node->score, best_node->gain, tree_ms);
    }

    if (best_node) {
        next_best_node = best_node;
        RRTStar.backtrackPathNode(best_node, best_branch, next_best_node);
        visualize_path(best_node, ns);
        prev_best_branch = best_branch;
    }
}

double NBVPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) { return planner_helpers::distance(waypoint, pose); }

void NBVPlanner::commandWaypoint(const Eigen::Vector4d& waypoint, const Eigen::Vector4d& prev_waypoint) {
    ROS_INFO("[NBVPlanner]: horizon step %zu/%d (plan %d) -> [%.2f, %.2f, %.2f] yaw=%.2f", exec_index_, exec_horizon_limit_, iteration_, waypoint[0], waypoint[1], waypoint[2], waypoint[3]);
    if (!current_waypoint_) {
        current_waypoint_ = std::make_unique<mrs_msgs::Reference>();
    }
    current_waypoint_->position.x = waypoint[0];
    current_waypoint_->position.y = waypoint[1];
    current_waypoint_->position.z = waypoint[2];
    current_waypoint_->heading   = waypoint[3];

    next_start = waypoint;

    previous_node = std::make_unique<rrt_star::Node>(prev_waypoint);
    previous_node->parent = nullptr;

    mrs_msgs::ReferenceStamped initial_reference;
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();
    initial_reference.reference.position.x = waypoint[0];
    initial_reference.reference.position.y = waypoint[1];
    initial_reference.reference.position.z = waypoint[2];
    initial_reference.reference.heading = waypoint[3];
    pub_reference.publish(initial_reference.reference);
    pub_initial_reference.publish(initial_reference);

    ros::Duration(1).sleep();
}

void NBVPlanner::initialize(mrs_msgs::ReferenceStamped initial_reference) {
    initial_reference.header.frame_id = ns + "/" + frame_id;
    initial_reference.header.stamp = ros::Time::now();

    ROS_INFO("[NBVPlanner]: Flying 3 meters up");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 3;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(3).sleep();

    ROS_INFO("[NBVPlanner]: Rotating 360 degrees");

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

    ROS_INFO("[NBVPlanner]: Flying 2 meters down");

    initial_reference.reference.position.x = pose[0];
    initial_reference.reference.position.y = pose[1];
    initial_reference.reference.position.z = pose[2] + 1;
    initial_reference.reference.heading = pose[3];
    pub_initial_reference.publish(initial_reference);
    // Max horizontal speed is 1 m/s so we wait 2 second between points
    ros::Duration(2).sleep();
}

void NBVPlanner::rotate() {
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

bool NBVPlanner::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
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

    changeState(STATE_INITIALIZE);

    res.success = true;
    res.message = "starting";
    return true;

}

bool NBVPlanner::callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
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
    changeState(STATE_STOPPED);

    std::stringstream ss;
    ss << "Stopping by request";

    res.success = true;
    res.message = ss.str();
    return true;

}

void NBVPlanner::callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[NBVPlanner]: getting ControlManager diagnostics");
    control_manager_diag = *msg;
}

void NBVPlanner::callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[NBVPlanner]: getting UavState diagnostics");
    geometry_msgs::Pose uav_state = msg->pose;
    double yaw = mrs_lib::getYaw(uav_state);
    pose = {uav_state.position.x, uav_state.position.y, uav_state.position.z, yaw};
}

void NBVPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequsities //{ */

    const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
    const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

    if (!got_control_manager_diag || !got_uav_state) {
        ROS_INFO_THROTTLE(1.0, "[NBVPlanner]: waiting for data: ControlManagerDiag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
        return;
    } else {
        ready_to_plan_ = true;
    }

    std_msgs::Bool starter;
    starter.data = true;
    pub_start.publish(starter);

    ROS_INFO_ONCE("[NBVPlanner]: main timer spinning");

    if (!set_variables) {
        GetTransformation();
        ROS_INFO("[NBVPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[NBVPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
        set_variables = true;
    }

    switch (state_) {
        case STATE_IDLE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[NBVPlanner]: tracker has goal");
            } else {
                ROS_INFO("[NBVPlanner]: waiting for command");
            }
            break;
        }
        case STATE_WAITING_INITIALIZE: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[NBVPlanner]: tracker has goal");
            } else {
                ROS_INFO("[NBVPlanner]: waiting for command");
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

            retreating_ = false;   // a boxed-in backtrack inside NBV() re-sets this

            {
                auto plan_t0 = std::chrono::high_resolution_clock::now();
                NBV();
                total_planning_ms_ += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - plan_t0).count();
            }
            clear_all_voxels();

            if (state_ != STATE_PLANNING) {
                break;
            }

            iteration_ += 1;

            // Boxed-in retreat (AEP-style): fly one step back along the executed path, bypassing horizon
            // execution, so the drone leaves the boxed-in spot instead of re-planning in place.
            if (retreating_ && !executed_path_.empty()) {
                retreat_node_ = std::make_unique<rrt_star::Node>(executed_path_.back());
                exec_horizon_limit_ = 1;
                exec_index_ = 1;
                commandWaypoint(retreat_node_->point, pose);
                changeState(STATE_MOVING);
                break;
            }

            exec_waypoints_ = best_branch;
            if (exec_waypoints_.size() < 2) {
                changeState(STATE_PLANNING);
                break;
            }
            exec_horizon_limit_ = execution_horizon_;
            if (exec_horizon_limit_ > (int)exec_waypoints_.size() - 1) {
                exec_horizon_limit_ = (int)exec_waypoints_.size() - 1;
            }
            exec_index_ = 1;

            // Record the executed (forward) path so a later backtrack can retreat along it.
            if (executed_path_.empty()) executed_path_.push_back(exec_waypoints_[0]);
            for (int i = 1; i <= exec_horizon_limit_; ++i) executed_path_.push_back(exec_waypoints_[i]);

            for (size_t i = 1; i <= (size_t)exec_horizon_limit_; ++i) {
                visualize_frustum(exec_waypoints_[i], (int)i);
                visualize_unknown_voxels(exec_waypoints_[i], (int)i * 100000);
            }

            commandWaypoint(exec_waypoints_[exec_index_], exec_waypoints_[exec_index_ - 1]);

            best_branch.clear();
            changeState(STATE_MOVING);
            break;

        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[NBVPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double dist = distance(current_waypoint_, current_pose);
                ROS_INFO("[NBVPlanner]: Distance to waypoint: %.2f", dist);
            } else {
                if (exec_index_ < (size_t)exec_horizon_limit_) {
                    exec_index_++;
                    commandWaypoint(exec_waypoints_[exec_index_], exec_waypoints_[exec_index_ - 1]);
                } else {
                    changeState(STATE_PLANNING);
                }
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[NBVPlanner]: Total Iterations: %d", iteration_);
            if (!stats_written_) {
                std::string log_dir;
                if (nh_private_.getParam("performance_log_dir", log_dir) && !log_dir.empty()) {
                    std::ofstream dl(log_dir + "/data_log.txt", std::ios::app);
                    if (dl.is_open())
                        dl << "total_planning_time_ms=" << total_planning_ms_ << " iterations=" << iteration_ << "\n";
                }
                stats_written_ = true;
            }
            ROS_INFO("[NBVPlanner]: Shutting down.");
            ros::shutdown();
            return;
        }
        default: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[NBVPlanner]: tracker has goal");
            } else {
                ROS_INFO("[NBVPlanner]: waiting for command");
            }
            break;
        }
    }
}

void NBVPlanner::changeState(const State_t new_state) {
    const State_t old_state = state_;

    if (old_state == STATE_STOPPED) {
        ROS_WARN("[NBVPlanner]: Planning interrupted, not changing state.");
        return;
    }

    ROS_INFO("[NBVPlanner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

    state_ = new_state;
}

void NBVPlanner::visualize_node(const Eigen::Vector4d& pos, const std::string& ns) {
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

void NBVPlanner::visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns) {
    visualization_msgs::Marker edges;
    edges.header.stamp = ros::Time::now();
    edges.header.frame_id = ns + "/" + frame_id;
    edges.ns = "tree_branches";
    edges.id = 0;
    edges.type = visualization_msgs::Marker::LINE_LIST;
    edges.action = visualization_msgs::Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.06;
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
            edges.points.push_back(pp);
            edges.points.push_back(p);
        }
    }
    pub_markers.publish(edges);
    pub_markers.publish(pts);
}

void NBVPlanner::visualize_edge(rrt_star::Node* node, const std::string& ns) {
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

void NBVPlanner::visualize_path(rrt_star::Node* node, const std::string& ns) {
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

void NBVPlanner::visualize_frustum(const Eigen::Vector4d& waypoint, int id) {
    Eigen::Vector4d trajectory_point_visualize = waypoint;

    visualization_msgs::Marker frustum;
    frustum.header.frame_id = ns + "/" + frame_id;
    frustum.header.stamp = ros::Time::now();
    frustum.ns = "camera_frustum";
    frustum.id = id;
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

void NBVPlanner::visualize_unknown_voxels(const Eigen::Vector4d& waypoint, int id_base) {
    Eigen::Vector4d trajectory_point_visualize = waypoint;

    voxblox::Pointcloud voxel_points;
    segment_evaluator.visualizeGain(trajectory_point_visualize, voxel_points);
    
    visualization_msgs::MarkerArray voxels_marker;
    for (size_t i = 0; i < voxel_points.size(); ++i) {
        visualization_msgs::Marker unknown_voxel;
        unknown_voxel.header.frame_id = ns + "/" + frame_id;
        unknown_voxel.header.stamp = ros::Time::now();
        unknown_voxel.ns = "unknown_voxels";
        unknown_voxel.id = id_base + (int)i;
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

void NBVPlanner::clear_node() {
    visualization_msgs::Marker clear_node;
    clear_node.header.stamp = ros::Time::now();
    clear_node.ns = "nodes";
    clear_node.id = node_id_counter_;
    clear_node.action = visualization_msgs::Marker::DELETE;
    node_id_counter_--;
    pub_markers.publish(clear_node);
}

void NBVPlanner::clear_all_voxels() {
    visualization_msgs::Marker clear_voxels;
    clear_voxels.header.stamp = ros::Time::now();
    clear_voxels.ns = "unknown_voxels";
    clear_voxels.action = visualization_msgs::Marker::DELETEALL;
    pub_voxels.publish(clear_voxels);
}

void NBVPlanner::clearMarkers() {
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
