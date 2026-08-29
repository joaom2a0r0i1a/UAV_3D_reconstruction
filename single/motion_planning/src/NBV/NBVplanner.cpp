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

    param_loader.loadParam("evaluation/marginal_gain", marginal_gain, false);
    param_loader.loadParam("evaluation/optimize_yaw", optimize_yaw, false);
    param_loader.loadParam("evaluation/compute", eval_compute, std::string("cpu"));
    param_loader.loadParam("evaluation/marginal_split", marginal_split, false);
    param_loader.loadParam("evaluation/objective", objective_, std::string("expdecay"));

    // Benchmark / X2 timing
    param_loader.loadParam("benchmark/enabled", benchmark_mode, false);
    param_loader.loadParam("benchmark/timing_after_s", timing_after_s_, 600.0);
    param_loader.loadParam("benchmark/x2_max", x2_capture_max_, 10);
    param_loader.loadParam("benchmark/suite", bench_suite_, std::string("x2"));

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

// Evaluate node gains per (marginal_gain, eval_compute), always at each node's fixed yaw.
void NBVPlanner::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    GainEvaluator::GainConfig cfg{marginal_gain, optimize_yaw, eval_compute, marginal_split, /*track_absolute=*/false};
    segment_evaluator.evaluateGains(nodes, flat_map_, cfg, last_marg_kernel_ms_, last_abs_kernel_ms_);
}

std::vector<rrt_star::Node*> NBVPlanner::collectTreeNodes() { return planner_helpers::collectTreeNodes(RRTStar); }

// Per-node score dump over the final tree (once), so multi-batch runs don't re-log each batch.
void NBVPlanner::logTreeNodes() { if (benchmark_mode) return; planner_helpers::logTreeNodes(RRTStar, lambda); }

// Dispatch the selected benchmark suite(s) on `nodes` (shared impl).
void NBVPlanner::benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase) {
    planner_helpers::runBenchSuite(segment_evaluator, nodes, flat_map_, bench_, bench_suite_,
                                   optimize_yaw, marginal_split, replan_count_, phase);
}

bool NBVPlanner::inBoundingBox(const Eigen::Vector4d& p) const { return planner_helpers::inBoundingBox(p, min_x, max_x, min_y, max_y, min_z, max_z); }

// Sample a point, steer from the nearest node, and add it to the tree; returns nullptr if it lands
// outside the box or the node/parent-edge collides (edge check catches walls between free endpoints).
rrt_star::Node* NBVPlanner::expandTreeNode(rrt_star::Node* root_ptr) {
    Eigen::Vector4d rand_point_yaw;
    RRTStar.computeSamplingDimensionsNBV(bounded_radius, rand_point_yaw);
    Eigen::Vector3d rand_point = rand_point_yaw.head(3) + root_ptr->point.head(3);

    rrt_star::Node* nearest_node = nullptr;
    RRTStar.findNearestKD(rand_point, nearest_node);
    std::unique_ptr<rrt_star::Node> new_node;
    RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node, fixed_step, min_edge_length_);

    if (!inBoundingBox(new_node->point)) return nullptr;

    std::vector<rrt_star::Node*> seg = {new_node.get()};
    if (!isPathCollisionFree(seg)) { collision_id_counter_++; return nullptr; }
    if (!isEdgeCollisionFree(nearest_node->point.head<3>(), new_node->point.head<3>())) { collision_id_counter_++; return nullptr; }

    new_node->point[3] = rand_point_yaw[3];   // sampled yaw; optimize_yaw may re-pick it in evaluateGains
    new_node->gain = 0.0; new_node->score = 0.0; new_node->cum_gain = 0.0;
    segment_evaluator.computeCost(new_node.get());
    return RRTStar.addKDTreeNode(std::move(new_node));
}

void NBVPlanner::NBV() {
    best_score_ = 0;
    ++replan_count_;

    // X2 timing: only benchmark once sim-time passes the threshold (early collision-heavy replans skew timings).
    double sim_now = ros::Time::now().toSec();
    bool x2_was_open = x2_timing_window_;
    x2_timing_window_ = (sim_now >= timing_after_s_);
    if (x2_timing_window_ && !x2_was_open)
        ROS_WARN("[X2] timing window OPEN at sim_t=%.1fs (threshold=%.1fs, replan=%d)", sim_now, timing_after_s_, replan_count_);
    bool x2_do_capture = benchmark_mode && x2_timing_window_ && (x2_capture_count_ < x2_capture_max_);

    if (benchmark_mode) bench_ = {};

    auto tree_t0 = std::chrono::high_resolution_clock::now();

    // Root = next executed pose (or the drone's current pose on the first plan).
    std::unique_ptr<rrt_star::Node> root;
    if (current_waypoint_)           root = std::make_unique<rrt_star::Node>(next_start);
    else if (best_branch.size() > 1) root = std::make_unique<rrt_star::Node>(prev_best_branch[1]);
    else                             root = std::make_unique<rrt_star::Node>(pose);
    root->cost = 0;

    RRTStar.clearKDTree();
    rrt_star::Node* root_ptr = RRTStar.addKDTreeNode(std::move(root));
    clearMarkers();

    flat_map_ = segment_evaluator.flattenMap(map_origin_, map_dim_);       // for the fixed-yaw GPU + CPU-flatmap eval
    segment_evaluator.cacheMapOnGPU(flat_map_, map_origin_, map_dim_);

    root_ptr->depth_buffer.clear();
    best_score_ = root_ptr->score;
    rrt_star::Node* best_node = root_ptr;

    int j = 1;

    // PHASE A: re-add the un-executed remainder of the previous best branch as a fixed chain. The drone executed
    // exec_horizon_limit_ steps, so the new root is prev_best_branch[exec_horizon_limit_]; re-add from the next node.
    const size_t reAddStart = (size_t)exec_horizon_limit_ + 1;
    if (prev_best_branch.size() > reAddStart) {
        std::vector<rrt_star::Node*> branch_candidates;
        for (size_t i = reAddStart; i < prev_best_branch.size(); ++i) {
            rrt_star::Node* nearest_node_best = nullptr;
            RRTStar.findNearestKD(prev_best_branch[i].head(3), nearest_node_best);
            auto new_node_best = std::make_unique<rrt_star::Node>(prev_best_branch[i]);
            new_node_best->parent = nearest_node_best;
            segment_evaluator.computeCost(new_node_best.get());
            branch_candidates.push_back(RRTStar.addKDTreeNode(std::move(new_node_best)));
        }
        if (!branch_candidates.empty()) {
            evaluateGains(branch_candidates);
            for (rrt_star::Node* node : branch_candidates) {
                segment_evaluator.computeScore(node, lambda);
                if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
            }
            j += (int)branch_candidates.size();
        }
    }
    prev_best_branch.clear();
    best_branch.clear();

    // PHASE B: batched RRT expansion with fixed-yaw gain.
    const int BATCH_SIZE = 2 * N_max;
    collision_id_counter_ = 0;
    bool terminated = false;
    double x2_tree_ms = 0.0, x2_eval_ms = 0.0, x2_score_ms = 0.0, x2_kernel_ms = 0.0;
    ros::WallTime plan_start_ = ros::WallTime::now();   // bounds the tree build so NBV() can never spin (single-threaded timer)

    while (j < N_max || best_score_ == 0.0) {

        // Wall-clock-bounded backtrack: boxed-in = tree still tiny past a short deadline; timed-out = hard cap.
        const double plan_elapsed = (ros::WallTime::now() - plan_start_).toSec();
        const bool boxed_in  = plan_elapsed > recovery_boxed_deadline_ && j < recovery_min_tree_;
        const bool timed_out = plan_elapsed > recovery_timeout_;
        if (recovery_enabled_ && (boxed_in || timed_out)) {
            if (!executed_path_.empty()) executed_path_.pop_back();   // drop the node we're on
            if (!executed_path_.empty()) {
                retreating_ = true;                                  // timerMain flies back to executed_path_.back()
                logTreeNodes();
                ROS_WARN("[NBVPlanner]: Backtracking (%s after %.1fs, tree=%d) -> executed node %zu",
                         boxed_in ? "boxed-in" : "timeout", plan_elapsed, j, executed_path_.size());
                best_branch.clear();
                prev_best_branch.clear();
                return;
            }
            ROS_INFO("[NBVPlanner]: Backtrack Rotation");
            rotate();
            plan_start_ = ros::WallTime::now();
            collision_id_counter_ = 0;
        }

        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int cap = std::min(BATCH_SIZE, nodes_needed);
        if (cap <= 0) break;

        std::vector<rrt_star::Node*> batch_nodes;
        auto x2_tree0 = std::chrono::high_resolution_clock::now();
        for (int k = 0; k < cap && j <= N_termination; ++k) {

            // Boxed-in guard: when every sample collides, k-- spins this inner loop and the outer check never runs.
            if (recovery_enabled_) {
                const double e = (ros::WallTime::now() - plan_start_).toSec();
                if ((e > recovery_boxed_deadline_ && j < recovery_min_tree_) || e > recovery_timeout_) break;
            }
            rrt_star::Node* added_node = expandTreeNode(root_ptr);
            if (!added_node) { k--; continue; }
            batch_nodes.push_back(added_node);
            j++;
        }
        x2_tree_ms += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - x2_tree0).count();

        if (batch_nodes.empty()) continue;

        // Fixed yaw is tree-independent, so RRT scores and re-raycasts only the new batch.
        std::vector<rrt_star::Node*> score_nodes = batch_nodes;
        std::vector<rrt_star::Node*> gain_nodes  = batch_nodes;

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

    if (benchmark_mode) planner_helpers::logBenchSummary(bench_);

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


void NBVPlanner::visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns) { planner_helpers::visualize_tree(pub_markers, frame_id, ns, nodes); }


void NBVPlanner::visualize_path(rrt_star::Node* node, const std::string& ns) { planner_helpers::visualize_path(pub_markers, frame_id, ns, node, path_id_counter_); }

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


void NBVPlanner::clear_all_voxels() { planner_helpers::clear_all_voxels(pub_voxels); }

void NBVPlanner::clearMarkers() { planner_helpers::clearMarkers(pub_markers, node_id_counter_, edge_id_counter_, path_id_counter_); }
