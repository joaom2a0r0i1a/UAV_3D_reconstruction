#include "motion_planning_real_world/NBVReal/NBVplannerReal.h"
#include <cstdlib>
#include <fstream>
#include <algorithm>

NBVPlanner::NBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), segment_evaluator(nh_private_), voxblox_server_(nh_, nh_private_) {

    /* Parameter loading (defaults = config/NBVplannerReal.yaml values) */

    // Namespace
    nh_private_.param("uav_namespace", ns, std::string("uav1"));

    // Frames, Coordinates and Dimensions
    nh_private_.param("frame_id", frame_id, std::string("map"));
    nh_private_.param("body/frame_id", body_frame_id, std::string("base_link"));
    nh_private_.param("pose_sanity/max_distance", pose_max_distance_, 500.0);
    nh_private_.param("pose_sanity/max_speed", pose_max_speed_, 20.0);
    nh_private_.param("camera/frame_id", camera_frame_id, std::string("camera_link"));

    // Bounded Box (relative to the start pose; shifted by captureOffset)
    nh_private_.param("bounded_box/min_x", min_x, -1.0f);
    nh_private_.param("bounded_box/max_x", max_x, 11.0f);
    nh_private_.param("bounded_box/min_y", min_y, -6.0f);
    nh_private_.param("bounded_box/max_y", max_y, 6.0f);
    nh_private_.param("bounded_box/min_z", min_z, 3.0f);
    nh_private_.param("bounded_box/max_z", max_z, 9.0f);
    base_min_x = min_x; base_max_x = max_x;
    base_min_y = min_y; base_max_y = max_y;
    base_min_z = min_z; base_max_z = max_z;

    // RRT Tree
    nh_private_.param("rrt/N_max", N_max, 20);
    nh_private_.param("rrt/N_termination", N_termination, 200);
    nh_private_.param("rrt/N_yaw_samples", num_yaw_samples, 10);
    nh_private_.param("rrt/radius", radius, 2.0);
    nh_private_.param("rrt/step_size", step_size, 2.0);
    nh_private_.param("rrt/min_edge_length", min_edge_length_, 0.2);
    nh_private_.param("rrt/fixed_step", fixed_step, false);
    nh_private_.param("rrt/tolerance", tolerance, 1.0);

    nh_private_.param("evaluation/marginal_gain", marginal_gain, true);
    nh_private_.param("evaluation/optimize_yaw", optimize_yaw, true);
    nh_private_.param("evaluation/compute", eval_compute, std::string("gpu"));
    nh_private_.param("evaluation/marginal_split", marginal_split, false);
    nh_private_.param("evaluation/objective", objective_, std::string("expdecay"));

    // Camera
    nh_private_.param("camera/h_fov", horizontal_fov, 1.51844);
    nh_private_.param("camera/width", resolution_x, 1080);
    nh_private_.param("camera/height", resolution_y, 720);
    nh_private_.param("camera/min_distance", min_distance, 0.2);
    nh_private_.param("camera/max_distance", max_distance, 5.0);

    // Planner
    nh_private_.param("path/uav_radius", uav_radius, 2.0);
    nh_private_.param("path/collision_check_resolution", collision_check_resolution_, 0.1);
    nh_private_.param("path/waypoint_reach_distance", waypoint_reach_distance_, 0.4);
    nh_private_.param("path/waypoint_reach_velocity", waypoint_reach_velocity_, 0.15);
    nh_private_.param("path/optimistic_iterations", optimistic_iterations_, 5);
    nh_private_.param("path/recovery_enabled", recovery_enabled_, true);
    nh_private_.param("path/recovery_boxed_deadline", recovery_boxed_deadline_, 4.0);
    nh_private_.param("path/recovery_min_tree", recovery_min_tree_, 10);
    nh_private_.param("path/recovery_timeout", recovery_timeout_, 12.0);
    nh_private_.param("rotation/step_deg", rotation_step_deg_, 45.0);
    nh_private_.param("rotation/settle", rotation_settle_, 1.0);
    nh_private_.param("path/lambda", lambda, 0.5);

    // Timer
    nh_private_.param("timer_main/rate", timer_main_rate, 10.0);

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

    // Setup TF listener (body -> camera extrinsics)
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

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
    pub_start = nh_private_.advertise<std_msgs::Bool>("simulation_ready", 1);
    pub_frustum = nh_private_.advertise<visualization_msgs::Marker>("frustum_out", 10);
    pub_voxels = nh_private_.advertise<visualization_msgs::MarkerArray>("unknown_voxels_out", 10);
    pub_setpoint = nh_private_.advertise<mavros_msgs::PositionTarget>("setpoint_out", 10);
    pub_offset = nh_private_.advertise<geometry_msgs::Point>("offset_out", 1, /*latch=*/true);

    /* Subscribers */
    sub_local_pose = nh_private_.subscribe("local_pose_in", 10, &NBVPlanner::callbackLocalPose, this);
    sub_velocity = nh_private_.subscribe("velocity_in", 10, &NBVPlanner::callbackVelocity, this);
    sub_state = nh_private_.subscribe("state_in", 10, &NBVPlanner::callbackState, this);

    /* Service Servers */
    ss_start = nh_private_.advertiseService("start_in", &NBVPlanner::callbackStart, this);
    ss_stop = nh_private_.advertiseService("stop_in", &NBVPlanner::callbackStop, this);
    ss_offset = nh_private_.advertiseService("offset_in", &NBVPlanner::callbackOffset, this);

    /* Timer */
    timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &NBVPlanner::timerMain, this);

    is_initialized = true;
}

double NBVPlanner::getMapDistance(const Eigen::Vector3d& position) const { return planner_helpers::getMapDistance(voxblox_server_, position); }

bool NBVPlanner::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const { return planner_helpers::isPathCollisionFree(voxblox_server_, path, uav_radius); }

bool NBVPlanner::isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const { return planner_helpers::isEdgeCollisionFree(voxblox_server_, from, to, uav_radius, collision_check_resolution_, optimistic_edges_); }

void NBVPlanner::GetTransformation() {
    // From Body Frame to Camera Frame; on success flips set_variables so timerMain stops retrying.
    geometry_msgs::TransformStamped msg;
    try {
        // T_C_B = body-in-camera (CameraModel::setBodyPose inverts it); tf2 lookupTransform(target,source) gives source-in-target so target=camera here (opposite of mrs getTransform).
        msg = tf_buffer_.lookupTransform(camera_frame_id, body_frame_id, ros::Time(0), ros::Duration(3.0));
    } catch (const tf2::TransformException& e) {
        ROS_ERROR_THROTTLE(1.0, "[NBVPlanner]: could not get transform from body frame to the camera frame: %s", e.what());
        return;
    }

    T_C_B_message = msg;

    // Transform into matrix
    tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
    segment_evaluator.setCameraExtrinsics(T_C_B);
    set_variables = true;
}

// Evaluate node gains per (marginal_gain, eval_compute), always at each node's fixed yaw.
void NBVPlanner::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    GainEvaluator::GainConfig cfg{marginal_gain, optimize_yaw, eval_compute, marginal_split, /*track_absolute=*/false};
    segment_evaluator.evaluateGains(nodes, flat_map_, cfg, last_marg_kernel_ms_, last_abs_kernel_ms_);
}

std::vector<rrt_star::Node*> NBVPlanner::collectTreeNodes() { return planner_helpers::collectTreeNodes(RRTStar); }

// Per-node score dump over the final tree (once), so multi-batch runs don't re-log each batch.
void NBVPlanner::logTreeNodes() { planner_helpers::logTreeNodes(RRTStar, lambda); }

bool NBVPlanner::inBoundingBox(const Eigen::Vector4d& p) const { return planner_helpers::inBoundingBox(p, min_x, max_x, min_y, max_y, min_z, max_z); }

// Sample+steer from the nearest node and add to the tree; nullptr if it lands outside the box or the node/parent-edge collides (walls between free endpoints).
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

    auto tree_t0 = std::chrono::high_resolution_clock::now();

    // Root = next executed pose (or the drone's current pose on the first plan).
    std::unique_ptr<rrt_star::Node> root;
    if (have_commanded_)             root = std::make_unique<rrt_star::Node>(next_start);
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

    // PHASE A: re-add the un-executed remainder of the previous best branch as a fixed chain (fixed horizon 1, so the new root is prev_best_branch[1]).
    const size_t reAddStart = 2;
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

        if (batch_nodes.empty()) continue;

        // Fixed yaw is tree-independent, so RRT scores and re-raycasts only the new batch.
        std::vector<rrt_star::Node*> score_nodes = batch_nodes;
        std::vector<rrt_star::Node*> gain_nodes  = batch_nodes;

        evaluateGains(gain_nodes);

        for (rrt_star::Node* node : score_nodes) {
            segment_evaluator.computeScore(node, lambda);
            if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
        }
        visualize_tree(collectTreeNodes());

        if (j >= N_termination) { terminated = true; break; }
    }

    logTreeNodes();

    if (terminated) {
        ROS_INFO("[NBVPlanner]: RH-NBVP Terminated");
        RRTStar.clearKDTree();
        best_branch.clear();
        clearMarkers();
        changeState(STATE_STOPPED);
        return;
    }

    if (best_node) {
        double tree_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - tree_t0).count();
        ROS_INFO("[NBVPlanner]: Chosen node [%.2f, %.2f, %.2f] score=%.3f gain=%.3f | tree computed in %.1f ms",
                 best_node->point[0], best_node->point[1], best_node->point[2], best_node->score, best_node->gain, tree_ms);

        next_best_node = best_node;
        RRTStar.backtrackPathNode(best_node, best_branch, next_best_node);
        visualize_path(best_node);
        prev_best_branch = best_branch;
    }
}

double NBVPlanner::distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b) { return planner_helpers::distance(a, b); }

void NBVPlanner::captureOffset() {
    initial_offset = pose.head<3>();
    // z from the ground reading latched at arming; the current pose is airborne.
    if (have_ground_z_) {
        initial_offset.z() = ground_z_;
    } else {
        initial_offset.z() = 0.0;
        ROS_WARN("[NBVPlanner]: never saw the disarmed to armed edge, using z offset 0. Start the "
                 "planner stack before arming to correct the barometric bias.");
    }

    // Shift the bounded box (base = yaml values, so a re-capture is idempotent).
    min_x = base_min_x + (float)initial_offset.x();  max_x = base_max_x + (float)initial_offset.x();
    min_y = base_min_y + (float)initial_offset.y();  max_y = base_max_y + (float)initial_offset.y();
    min_z = base_min_z + (float)initial_offset.z();  max_z = base_max_z + (float)initial_offset.z();

    // Shift the gain_evaluation box the same way (GPU flattenMap + CPU raycasts use it directly).
    segment_evaluator.setWorldOffset(initial_offset);

    geometry_msgs::Point offset_msg;
    offset_msg.x = initial_offset.x();
    offset_msg.y = initial_offset.y();
    offset_msg.z = initial_offset.z();
    pub_offset.publish(offset_msg);   // latched

    ROS_INFO("[NBVPlanner]: Start offset captured: [%.2f, %.2f, %.2f]", initial_offset.x(), initial_offset.y(), initial_offset.z());
    ROS_INFO("[NBVPlanner]: Bounded box shifted to x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f]", min_x, max_x, min_y, max_y, min_z, max_z);
}

mavros_msgs::PositionTarget NBVPlanner::makeSetpoint(const Eigen::Vector4d& waypoint) {
    mavros_msgs::PositionTarget sp;
    sp.header.frame_id = frame_id;
    sp.header.stamp = ros::Time::now();
    sp.coordinate_frame = 1;   // FRAME_LOCAL_NED (mavros local frame)
    sp.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY | mavros_msgs::PositionTarget::IGNORE_VZ
                 | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ
                 | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    sp.position.x = waypoint[0];
    sp.position.y = waypoint[1];
    sp.position.z = waypoint[2];
    sp.yaw = waypoint[3];
    return sp;
}

void NBVPlanner::commandWaypoint(const Eigen::Vector4d& waypoint, const Eigen::Vector4d& prev_waypoint) {
    ROS_INFO("[NBVPlanner]: step (plan %d) -> [%.2f, %.2f, %.2f] yaw=%.2f", iteration_, waypoint[0], waypoint[1], waypoint[2], waypoint[3]);

    next_start = waypoint;
    current_target_ = waypoint;
    have_commanded_ = true;

    previous_node = std::make_unique<rrt_star::Node>(prev_waypoint);
    previous_node->parent = nullptr;

    active_setpoint_ = makeSetpoint(waypoint);
    has_active_setpoint_ = true;
    pub_setpoint.publish(active_setpoint_);
}

void NBVPlanner::rotate() {
    // Rotate 360deg in place (recovery sweep). Keep the step well under 180deg or the FCU
    // takes the short way round. Defaults 45deg/1.0s = 8 steps, ~8s (was 30deg/2.0s = 24s).
    const int steps = std::max(3, (int)std::ceil(360.0 / rotation_step_deg_));
    const double step = 2.0 * M_PI / steps;
    for (int s = 1; s <= steps; ++s) {
        Eigen::Vector4d wp = pose;
        wp[3] = pose[3] + s * step;
        pub_setpoint.publish(makeSetpoint(wp));
        ros::Duration(rotation_settle_).sleep();
    }
}

bool NBVPlanner::callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[NBVPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    captureOffset();
    changeState(STATE_PLANNING);

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

bool NBVPlanner::callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    if (!is_initialized) {
        res.success = false;
        res.message = "not initialized";
        return true;
    }

    if (!ready_to_plan_ || !have_pose_) {
        std::stringstream ss;
        ss << "not ready to plan, missing data";

        ROS_ERROR_STREAM_THROTTLE(0.5, "[NBVPlanner]: " << ss.str());

        res.success = false;
        res.message = ss.str();
        return true;
    }

    captureOffset();   // manual re-capture; idempotent (boxes recompute from base values)

    std::stringstream ss;
    ss << "Start offset re-captured: [" << initial_offset.x() << ", " << initial_offset.y() << ", " << initial_offset.z() << "]";

    res.success = true;
    res.message = ss.str();
    return true;

}

void NBVPlanner::callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    ROS_INFO_ONCE("[NBVPlanner]: getting LocalPose diagnostics");
    uav_local_pose = msg->pose;

    const geometry_msgs::Quaternion& q = uav_local_pose.orientation;

    // Check for NaNs or zero-length quaternion
    if (std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w)) {
        ROS_ERROR("[NBVPlanner]: Invalid quaternion received (contains NaNs)");
        return;
    }

    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm < 0.1 || norm > 1.1) {
      ROS_WARN_THROTTLE(5, "[NBVPlanner] Invalid quaternion detected. Norm: %.3f. Skipping this pose.", norm);
      return;
    }

    // mavros emits occasional wild poses (up to 1e35 in the 2025 flights).
    const geometry_msgs::Point& p = uav_local_pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        std::abs(p.x) > pose_max_distance_ || std::abs(p.y) > pose_max_distance_ ||
        std::abs(p.z) > pose_max_distance_) {
        ROS_WARN_THROTTLE(5, "[%s]: implausible pose [%.3g, %.3g, %.3g], skipping.",
                          "NBVPlanner", p.x, p.y, p.z);
        return;
    }
    if (have_pose_) {
        const double dt = (ros::Time::now() - last_pose_time_).toSec();
        const double jump = std::sqrt(std::pow(p.x - pose[0], 2) + std::pow(p.y - pose[1], 2) +
                                      std::pow(p.z - pose[2], 2));
        if (dt > 1e-3 && jump / dt > pose_max_speed_) {
            ROS_WARN_THROTTLE(5, "[%s]: pose jumped %.2f m in %.3f s, skipping.",
                              "NBVPlanner", jump, dt);
            return;
        }
    }

    double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

    pose = {uav_local_pose.position.x, uav_local_pose.position.y, uav_local_pose.position.z, yaw};
    last_pose_time_ = ros::Time::now();
    have_pose_ = true;
}

// Latches the barometric z bias on the ground, which captureOffset removes.
void NBVPlanner::callbackState(const mavros_msgs::State::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    // first arming edge only; mavros reports spurious re-arms in flight.
    if (msg->armed && !prev_armed_ && have_pose_ && !have_ground_z_) {
        ground_z_ = pose.z();
        have_ground_z_ = true;
        ROS_INFO("[NBVPlanner]: armed on the ground, latching z = %.2f m as the takeoff reference.",
                 ground_z_);
    }
    prev_armed_ = msg->armed;
}

void NBVPlanner::callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr msg) {
    if (!is_initialized) {
        return;
    }
    const auto& v = msg->twist.linear;
    current_speed_ = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    last_vel_time_ = ros::Time::now();
    have_vel_ = true;
}

void NBVPlanner::timerMain(const ros::TimerEvent& event) {
    if (!is_initialized) {
        return;
    }

    /* prerequisites */

    const bool got_local_pose = have_pose_ && (ros::Time::now() - last_pose_time_).toSec() < 2.0;

    if (!got_local_pose) {
        ROS_INFO_THROTTLE(1.0, "[NBVPlanner]: waiting for data: LocalPose = FALSE");
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
        if (!set_variables) return;   // keep retrying until the body->camera transform exists
        ROS_INFO("[NBVPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
        ROS_INFO("[NBVPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
    }

    switch (state_) {
        case STATE_IDLE: {
            ROS_INFO_THROTTLE(5.0, "[NBVPlanner]: waiting for command");
            break;
        }
        case STATE_PLANNING: {
            // Optimistic edges (plan through unknown) only for the first optimistic_iterations_ replans to bootstrap from spawn; after that unknown is blocked so we don't drive into a pocket.
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

            // Boxed-in retreat (AEP-style): fly one step back along the executed path so the drone leaves the boxed-in spot instead of re-planning in place.
            if (retreating_ && !executed_path_.empty()) {
                retreat_node_ = std::make_unique<rrt_star::Node>(executed_path_.back());
                commandWaypoint(retreat_node_->point, pose);
                changeState(STATE_MOVING);
                break;
            }

            exec_waypoints_ = best_branch;
            if (exec_waypoints_.size() < 2) {
                changeState(STATE_PLANNING);
                break;
            }

            // Record the executed (forward) path so a later backtrack can retreat along it.
            if (executed_path_.empty()) executed_path_.push_back(exec_waypoints_[0]);
            executed_path_.push_back(exec_waypoints_[1]);

            visualize_frustum(exec_waypoints_[1], 1);
            visualize_unknown_voxels(exec_waypoints_[1], 100000);

            // Execution horizon fixed to 1 in the real world: fly only the first step, then replan.
            commandWaypoint(exec_waypoints_[1], exec_waypoints_[0]);

            best_branch.clear();
            changeState(STATE_MOVING);
            break;

        }
        case STATE_MOVING: {
            if (!has_active_setpoint_) {
                changeState(STATE_PLANNING);
                break;
            }

            pub_setpoint.publish(active_setpoint_);   // keep streaming the current setpoint

            double dist = distance(pose, current_target_);
            double yaw_difference = std::fabs(std::remainder(current_target_[3] - pose[3], 2.0 * M_PI));

            // Velocity gate: require a true stop at the waypoint, falling back to distance-only if velocity is stale/absent so a missing topic can't deadlock the mission.
            const bool vel_fresh = have_vel_ && (ros::Time::now() - last_vel_time_).toSec() < 1.0;
            const double speed = vel_fresh ? current_speed_ : 0.0;
            if (!vel_fresh) ROS_WARN_ONCE("[NBVPlanner]: no fresh velocity - waypoint gate is distance-only (check ~velocity_in)");
            ROS_INFO_THROTTLE(1.0, "[NBVPlanner]: Distance to waypoint: %.2f, yaw diff: %.2f, speed: %.2f", dist, yaw_difference, speed);

            if (dist < waypoint_reach_distance_ && yaw_difference < 0.4 && speed < waypoint_reach_velocity_) {
                ROS_INFO("[NBVPlanner]: Reached waypoint");
                has_active_setpoint_ = false;
                changeState(STATE_PLANNING);
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


void NBVPlanner::visualize_tree(const std::vector<rrt_star::Node*>& nodes) { planner_helpers::visualize_tree(pub_markers, frame_id, nodes); }


void NBVPlanner::visualize_path(rrt_star::Node* node) { planner_helpers::visualize_path(pub_markers, frame_id, node, path_id_counter_); }

void NBVPlanner::visualize_frustum(const Eigen::Vector4d& waypoint, int id) {
    Eigen::Vector4d trajectory_point_visualize = waypoint;

    visualization_msgs::Marker frustum;
    frustum.header.frame_id = frame_id;
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
        unknown_voxel.header.frame_id = frame_id;
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
