#include "motion_planning/NBV/NBVplanner.h"

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
    param_loader.loadParam("rrt/tolerance", tolerance);
    param_loader.loadParam("rrt/rrt_star", local_rrt_star, false);

    param_loader.loadParam("evaluation/marginal_gain", marginal_gain, false);
    param_loader.loadParam("evaluation/compute", eval_compute, std::string("cpu"));
    param_loader.loadParam("evaluation/marginal_split", marginal_split, false);
    param_loader.loadParam("evaluation/benchmark", benchmark_mode, false);

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
    transformer_ = std::make_unique<mrs_lib::Transformer>("NBVPlanner");
    transformer_->setDefaultFrame(frame_id);
    transformer_->setDefaultPrefix(ns);
    transformer_->retryLookupNewest(true);

    set_variables = false;

    // Setup Collision Avoidance
    voxblox_server_.setTraversabilityRadius(uav_radius);
    voxblox_server_.publishTraversable();

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

double NBVPlanner::getMapDistance(const Eigen::Vector3d& position) const {
    if (!voxblox_server_.getEsdfMapPtr()) {
        return 0.0;
    }
    double distance = 0.0;
    if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
        return 0.0;
    }
    return distance;
}

bool NBVPlanner::isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const {
    for (rrt_star::Node* node : path) {
        if (getMapDistance(node->point.head(3)) < uav_radius) {
            return false;
        }
    }
    return true;
}

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

std::vector<float> NBVPlanner::parentCamRows(float yaw) {
    const float pitch = 10.0f * M_PI / 180.0f;
    float cos_y = cosf(yaw), sin_y = sinf(yaw);
    float cos_p = cosf(pitch), sin_p = sinf(pitch);
    return { sin_y,          -cos_y,          0.0f,
             -sin_p * cos_y, -sin_p * sin_y, -cos_p,
              cos_p * cos_y,  cos_p * sin_y, -sin_p };
}

// Batched multi-ancestor marginal gain at each node's FIXED yaw (generation-ordered).
void NBVPlanner::evaluateMarginalGainsBatched(const std::vector<rrt_star::Node*>& nodes) {
    if (nodes.empty()) return;
    const int per = segment_evaluator.depthImagePixels();

    std::map<int, std::vector<size_t>> levels;
    for (size_t i = 0; i < nodes.size(); ++i) {
        int depth = 0;
        for (rrt_star::Node* a = nodes[i]->parent; a != nullptr; a = a->parent) ++depth;
        levels[depth].push_back(i);
    }

    for (const auto& level : levels) {
        const std::vector<size_t>& idxs = level.second;
        const size_t n = idxs.size();

        std::vector<float> cand_x_f(n), cand_y_f(n), cand_z_f(n), fixed_yaws(n);
        std::vector<int>   anc_offsets(1, 0);
        std::vector<float> anc_pos, anc_yaw, anc_R, anc_depth;
        for (size_t li = 0; li < n; ++li) {
            rrt_star::Node* nd = nodes[idxs[li]];
            cand_x_f[li] = (float)nd->point.x();
            cand_y_f[li] = (float)nd->point.y();
            cand_z_f[li] = (float)nd->point.z();
            fixed_yaws[li] = (float)nd->point[3];        // evaluate at the node's own random yaw
            for (rrt_star::Node* a = nd->parent; a != nullptr; a = a->parent) {
                std::vector<float> R_flat = parentCamRows((float)a->point[3]);
                anc_pos.push_back((float)a->point.x());
                anc_pos.push_back((float)a->point.y());
                anc_pos.push_back((float)a->point.z());
                anc_yaw.push_back((float)a->point[3]);
                anc_R.insert(anc_R.end(), R_flat.begin(), R_flat.end());
                if ((int)a->depth_buffer.size() == per)
                    anc_depth.insert(anc_depth.end(), a->depth_buffer.begin(), a->depth_buffer.end());
                else
                    anc_depth.insert(anc_depth.end(), per, -1.0f);   // root ancestor (never evaluated)
            }
            anc_offsets.push_back((int)(anc_pos.size() / 3));
        }

        std::vector<float> depth_out;
        float ms = 0.0f;
        auto results = segment_evaluator.computeMarginalGainBatchGPU(
            cand_x_f, cand_y_f, cand_z_f, anc_offsets, anc_pos, anc_yaw, anc_R, anc_depth,
            marginal_split, depth_out, ms, &fixed_yaws);

        for (size_t li = 0; li < n; ++li) {
            rrt_star::Node* node = nodes[idxs[li]];
            node->gain = results[li].first;              // yaw stays fixed
            node->depth_buffer.assign(depth_out.begin() + (size_t)li * per,
                                      depth_out.begin() + (size_t)(li + 1) * per);
        }
    }
}

// Evaluate node gains per (marginal_gain, eval_compute), always at each node's fixed yaw.
void NBVPlanner::evaluateGains(const std::vector<rrt_star::Node*>& nodes) {
    if (nodes.empty()) return;
    const bool gpu = (eval_compute == "gpu");

    if (marginal_gain && gpu) {
        evaluateMarginalGainsBatched(nodes);
    } else if (!marginal_gain && gpu) {
        std::vector<double> x(nodes.size()), y(nodes.size()), z(nodes.size());
        std::vector<float> fixed_yaws(nodes.size());
        for (size_t i = 0; i < nodes.size(); ++i) {
            x[i] = nodes[i]->point.x(); y[i] = nodes[i]->point.y(); z[i] = nodes[i]->point.z();
            fixed_yaws[i] = (float)nodes[i]->point[3];
        }
        auto res = segment_evaluator.computeGainBatchGPU(x, y, z, &fixed_yaws);
        for (size_t i = 0; i < nodes.size(); ++i) nodes[i]->gain = res[i].first;   // yaw fixed
    } else {
        for (rrt_star::Node* nd : nodes) {
            std::pair<double, double> r;
            if (marginal_gain) {
                if (nd->parent && nd->parent->parent) segment_evaluator.populateParentHistory(flat_map_, nd->parent);
                r = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, nd, nd->point[3]);
            } else {
                eth_mav_msgs::EigenTrajectoryPoint pose;
                pose.position_W = nd->point.head(3);
                r = segment_evaluator.computeGainCPU_FlatMap(flat_map_, pose, nd->point[3]);
            }
            nd->gain = r.first;   // yaw fixed
        }
    }
}

double NBVPlanner::computeV2SingleParent(rrt_star::Node* node) {
    const int per = segment_evaluator.depthImagePixels();
    rrt_star::Node* p = node->parent;
    double p_yaw = p ? p->point[3] : 0.0;
    std::vector<float> R = parentCamRows((float)p_yaw);
    Eigen::Vector3d p_pos = p ? p->point.head(3) : node->point.head(3);
    std::vector<float> p_depth;
    if (p && (int)p->depth_buffer.size() == per) p_depth = p->depth_buffer;
    else p_depth.assign((size_t)per, -1.0f);
    std::vector<float> out;
    auto r = segment_evaluator.computeMarginalGainGPU_v2(
        node->point.x(), node->point.y(), node->point.z(), p_pos, p_yaw, R, p_depth, out, node->point[3]);
    return r.first;
}

// Order nodes shallow-first so cumulative scoring sees each parent before its children.
void NBVPlanner::sortByDepth(std::vector<rrt_star::Node*>& nodes) {
    auto depth = [](rrt_star::Node* n) { int d = 0; for (auto* p = n->parent; p; p = p->parent) ++d; return d; };
    std::stable_sort(nodes.begin(), nodes.end(),
                     [&](rrt_star::Node* a, rrt_star::Node* b) { return depth(a) < depth(b); });
}

std::vector<rrt_star::Node*> NBVPlanner::collectTreeNodes() {
    std::vector<rrt_star::Node*> nodes;
    const auto& all = RRTStar.getNodes();
    nodes.reserve(all.size());
    for (const auto& up : all) if (up->parent) nodes.push_back(up.get());   // skip root
    return nodes;
}

// Per-node score dump over the final tree (once), so multi-batch runs don't re-log each batch.
void NBVPlanner::logTreeNodes() {
    if (benchmark_mode) return;
    for (const auto& up : RRTStar.getNodes())
        if (up->parent)
            ROS_INFO("[Node] gain=%.3f score_contribution=%.3f score=%.3f",
                     up->gain, up->gain * exp(-lambda * up->cost), up->score);
}

// Benchmark: time all methods + per-node v2-vs-cpuhash (all fixed-yaw) on the same nodes.
void NBVPlanner::benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase) {
    if (nodes.empty()) return;
    const size_t n = nodes.size();

    std::vector<double> save_gain(n);
    for (size_t i = 0; i < n; ++i) save_gain[i] = nodes[i]->gain;
    const bool save_marg = marginal_gain; const std::string save_comp = eval_compute; const bool save_split = marginal_split;

    auto timed = [&](bool marg, const std::string& comp, bool split) -> double {
        marginal_gain = marg; eval_compute = comp; marginal_split = split;
        auto t0 = std::chrono::high_resolution_clock::now();
        evaluateGains(nodes);
        auto t1 = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(t1 - t0).count();
    };

    double t_fused = timed(true,  "gpu", false); std::vector<double> g_fused(n); for (size_t i = 0; i < n; ++i) g_fused[i] = nodes[i]->gain;
    double t_split = timed(true,  "gpu", true);  std::vector<double> g_split(n); for (size_t i = 0; i < n; ++i) g_split[i] = nodes[i]->gain;
    double t_mcpu  = timed(true,  "cpu", false);
    double t_agpu  = timed(false, "gpu", false); std::vector<double> g_agpu(n);  for (size_t i = 0; i < n; ++i) g_agpu[i]  = nodes[i]->gain;
    double t_acpu  = timed(false, "cpu", false); std::vector<double> g_acpu(n);  for (size_t i = 0; i < n; ++i) g_acpu[i]  = nodes[i]->gain;

    bench_ms_fused += t_fused; bench_ms_split += t_split; bench_ms_mcpu += t_mcpu; bench_ms_agpu += t_agpu; bench_ms_acpu += t_acpu;
    bench_nodes += (int)n;

    marginal_gain = save_marg; eval_compute = save_comp; marginal_split = save_split;
    for (size_t i = 0; i < n; ++i) nodes[i]->gain = save_gain[i];

    // Per-node: GPU single-parent (v2) vs CPU single-parent hash (ground truth), both at the node's fixed yaw.
    for (size_t i = 0; i < n; ++i) {
        rrt_star::Node* nd = nodes[i];
        double v2 = computeV2SingleParent(nd);
        double sg = nd->gain;
        if (nd->parent && nd->parent->parent) segment_evaluator.populateParentHistory(flat_map_, nd->parent);
        double hash = segment_evaluator.computeMarginalGainCPU_HashMap(flat_map_, nd, nd->point[3]).first;
        nd->gain = sg;
        double err = std::fabs(v2 - hash);
        bench_v2_err_sum += err; if (err > bench_v2_err_max) bench_v2_err_max = err;
        ROS_INFO("[BENCH][%s] fused=%7.3f split=%7.3f | v2(gpu1p)=%7.3f hash(cpu1p)=%7.3f err=%.4f | absG=%7.3f absC=%7.3f",
                 phase, g_fused[i], g_split[i], v2, hash, err, g_agpu[i], g_acpu[i]);
    }
}

void NBVPlanner::NBV() {
    best_score_ = 0;
    rrt_star::Node* best_node = nullptr;
    if (benchmark_mode) {
        bench_ms_fused = bench_ms_split = bench_ms_mcpu = bench_ms_agpu = bench_ms_acpu = 0.0;
        bench_v2_err_sum = bench_v2_err_max = 0.0; bench_nodes = 0;
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

    // Root gain at its fixed yaw; NBVP scores the root by its own gain. Clear its depth buffer so
    // children don't subtract the root's view (matches AEP, where the root is never evaluated).
    { std::vector<rrt_star::Node*> only_root = {root_ptr}; evaluateGains(only_root); }
    root_ptr->depth_buffer.clear();
    root_ptr->score = root_ptr->gain;
    best_score_ = root_ptr->score;
    best_node = root_ptr;
    visualize_node(root_ptr->point, ns);

    int j = 1;

    // PHASE A: re-add the previous best branch as a fixed chain (each node keeps its cached yaw).
    if (prev_best_branch.size() > 2) {
        std::vector<rrt_star::Node*> branch_candidates;
        for (size_t i = 2; i < prev_best_branch.size(); ++i) {
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
            if (benchmark_mode) benchmarkGains(branch_candidates);
            for (rrt_star::Node* node : branch_candidates) {
                segment_evaluator.computeScore(node, lambda);
                if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
                visualize_node(node->point, ns); visualize_edge(node, ns);
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

    while (j < N_max || best_score_ == 0.0) {
        if (collision_id_counter_ > 10000 * j) {
            if (previous_node) {
                logTreeNodes();
                ROS_INFO("[NBVPlanner]: Backtracking to [%f, %f, %f]", previous_node->point[0], previous_node->point[1], previous_node->point[2]);
                next_best_node = previous_node.get();
                best_branch.clear();
                return;
            } else {
                ROS_INFO("[NBVPlanner]: Backtrack Rotation");
                rotate();
                collision_id_counter_ = 0;
            }
        }

        int nodes_needed = (j < N_max) ? (N_max - j) : (N_termination - j);
        int cap = std::min(BATCH_SIZE, nodes_needed);
        if (cap <= 0) break;

        std::vector<rrt_star::Node*> batch_nodes;
        for (int k = 0; k < cap && j <= N_termination; ++k) {
            Eigen::Vector4d rand_point_yaw;
            RRTStar.computeSamplingDimensionsNBV(bounded_radius, rand_point_yaw);
            Eigen::Vector3d rand_point = rand_point_yaw.head(3) + root_ptr->point.head(3);

            rrt_star::Node* nearest_node = nullptr;
            RRTStar.findNearestKD(rand_point, nearest_node);
            std::unique_ptr<rrt_star::Node> new_node;
            RRTStar.steer_parent(nearest_node, rand_point, step_size, new_node);

            if (new_node->point[0] > max_x || new_node->point[0] < min_x ||
                new_node->point[1] < min_y || new_node->point[1] > max_y ||
                new_node->point[2] < min_z || new_node->point[2] > max_z) { k--; continue; }

            std::vector<rrt_star::Node*> seg = {new_node.get()};
            if (!isPathCollisionFree(seg)) { collision_id_counter_++; k--; continue; }

            new_node->point[3] = rand_point_yaw[3];   // NBVP: fixed random yaw (never optimized)
            new_node->gain = 0.0; new_node->score = 0.0;

            rrt_star::Node* added_node;
            if (local_rrt_star) {
                std::vector<rrt_star::Node*> nearby;
                RRTStar.findNearbyKD(new_node.get(), radius, nearby);
                if (nearby.empty()) nearby.push_back(nearest_node);
                RRTStar.chooseParent(new_node.get(), nearby);
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
                RRTStar.rewire(added_node, nearby, radius);
            } else {
                segment_evaluator.computeCost(new_node.get());
                added_node = RRTStar.addKDTreeNode(std::move(new_node));
            }
            batch_nodes.push_back(added_node);
            j++;
        }

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

        evaluateGains(gain_nodes);
        if (benchmark_mode) benchmarkGains(gain_nodes);

        for (rrt_star::Node* node : score_nodes) {
            segment_evaluator.computeScore(node, lambda);
            if (node->score > best_score_) { best_score_ = node->score; best_node = node; }
            visualize_node(node->point, ns); visualize_edge(node, ns);
        }

        if (j >= N_termination) { terminated = true; break; }
    }

    logTreeNodes();

    if (benchmark_mode && bench_nodes > 0) {
        double bn = (double)bench_nodes;
        ROS_INFO("\n=== NBVP GAIN BENCHMARK (fixed yaw, %d nodes) ===\n"
                 "marginal-gpu-fused : %9.3f ms | %7.4f ms/node\n"
                 "marginal-gpu-split : %9.3f ms | %7.4f ms/node\n"
                 "marginal-cpu-hash  : %9.3f ms | %7.4f ms/node\n"
                 "absolute-gpu       : %9.3f ms | %7.4f ms/node\n"
                 "absolute-cpu       : %9.3f ms | %7.4f ms/node\n"
                 "v2(gpu 1-parent) vs cpu-hash: mean err %.4f | max err %.4f\n"
                 "==================================================",
                 bench_nodes,
                 bench_ms_fused, bench_ms_fused/bn, bench_ms_split, bench_ms_split/bn,
                 bench_ms_mcpu, bench_ms_mcpu/bn, bench_ms_agpu, bench_ms_agpu/bn,
                 bench_ms_acpu, bench_ms_acpu/bn, bench_v2_err_sum/bn, bench_v2_err_max);
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

double NBVPlanner::distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint->position.x, waypoint->position.y, waypoint->position.z),
                                 vec3_t(pose.position.x, pose.position.y, pose.position.z));
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
            NBV();
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
            current_waypoint_->heading   = next_best_node->point[3];

            next_start[0] = current_waypoint_->position.x;
            next_start[1] = current_waypoint_->position.y;
            next_start[2] = current_waypoint_->position.z;
            next_start[3] = current_waypoint_->heading;

            visualize_frustum(next_best_node);
            visualize_unknown_voxels(next_best_node);

            mrs_msgs::Reference reference;

            if (next_best_node && next_best_node->parent) {
                previous_node = std::make_unique<rrt_star::Node>(*next_best_node->parent);
            }

            mrs_msgs::ReferenceStamped initial_reference;
            initial_reference.header.frame_id = ns + "/" + frame_id;
            initial_reference.header.stamp = ros::Time::now();

            initial_reference.reference.position.x = next_best_node->point[0];
            initial_reference.reference.position.y = next_best_node->point[1];
            initial_reference.reference.position.z = next_best_node->point[2];
            initial_reference.reference.heading = next_best_node->point[3];
            pub_reference.publish(initial_reference.reference);
            pub_initial_reference.publish(initial_reference);

            best_branch.clear();
            ros::Duration(1).sleep();

            changeState(STATE_MOVING);
            break;

        }
        case STATE_MOVING: {
            if (control_manager_diag.tracker_status.have_goal) {
                ROS_INFO("[NBVPlanner]: tracker has goal");
                mrs_msgs::UavState::ConstPtr uav_state_here = sub_uav_state.getMsg();
                geometry_msgs::Pose current_pose = uav_state_here->pose;
                double current_yaw = mrs_lib::getYaw(current_pose);

                double dist = distance(current_waypoint_, current_pose);
                double yaw_difference = fabs(atan2(sin(current_waypoint_->heading - current_yaw), cos(current_waypoint_->heading - current_yaw)));
                ROS_INFO("[NBVPlanner]: Distance to waypoint: %.2f", dist);
            } else {
                ROS_INFO("[NBVPlanner]: waiting for command");
                changeState(STATE_PLANNING);
            }
            break;
        }
        case STATE_STOPPED: {
            ROS_INFO_ONCE("[NBVPlanner]: Total Iterations: %d", iteration_);
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

void NBVPlanner::visualize_frustum(rrt_star::Node* position) {
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

void NBVPlanner::visualize_unknown_voxels(rrt_star::Node* position) {
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
