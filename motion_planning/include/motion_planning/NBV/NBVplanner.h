#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/MpcPredictionFullState.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/Vec1.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/msg_extractor.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/utils/planning_utils.h>

#include <minkindr_conversions/kindr_msg.h>
#include <eth_mav_msgs/eigen_mav_msgs.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/gain_evaluator.h>

#include <map>
#include <chrono>
#include <algorithm>
#include <cmath>

typedef enum
{
  STATE_IDLE,
  STATE_INITIALIZE,
  STATE_WAITING_INITIALIZE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "INITIALIZE", "WAITING", "PLANNING", "MOVING", "REACHED"};

using vec3_t = mrs_lib::geometry::vec_t<3>;

class NBVPlanner {
public:
    NBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    // Sample the straight segment from->to at collision_check_resolution_ and require clearance >= uav_radius at each.
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void NBV();

    // Fixed-yaw gain evaluation (duplicated from AEP; NBVP keeps each node's random yaw).
    void evaluateMarginalGainsBatched(const std::vector<rrt_star::Node*>& nodes);
    void evaluateGains(const std::vector<rrt_star::Node*>& nodes);
    void benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase = "nbvp");
    std::vector<float> parentCamRows(float yaw);
    double computeSingleParentGainGPU(rrt_star::Node* node);
    std::vector<rrt_star::Node*> collectTreeNodes();
    void sortByDepth(std::vector<rrt_star::Node*>& nodes);
    void logTreeNodes();

    double distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose);
    void initialize(mrs_msgs::ReferenceStamped initial_reference);
    void rotate();

    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
    void callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);
    
    void changeState(const State_t new_state);

    void visualize_node(const Eigen::Vector4d& pos, const std::string& ns);
    void visualize_edge(rrt_star::Node* node, const std::string& ns);
    void visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns);
    void visualize_path(rrt_star::Node* node, const std::string& ns);
    void visualize_frustum(const Eigen::Vector4d& waypoint, int id);
    void visualize_unknown_voxels(const Eigen::Vector4d& waypoint, int id_base);

    void commandWaypoint(const Eigen::Vector4d& waypoint, const Eigen::Vector4d& prev_waypoint);
    void clear_node();
    void clear_all_voxels();
    void clearMarkers();

private:
    // Node Handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Gain Evaluator Instance
    GainEvaluator segment_evaluator;

    // Voxblox Map Server
    voxblox::EsdfServer voxblox_server_;

    // Shortcut to Maps
    voxblox::EsdfMap::Ptr esdf_map_;
    voxblox::TsdfMap::Ptr tsdf_map_;

    // Transformer
    std::unique_ptr<mrs_lib::Transformer> transformer_;
    bool set_variables;

    // Transformations
    geometry_msgs::TransformStamped T_C_B_message;
    voxblox::Transformation T_C_B;
    geometry_msgs::TransformStamped T_B_C_message;
    voxblox::Transformation T_B_C;

    // Parameters
    std::string frame_id;
    std::string body_frame_id;
    std::string camera_frame_id;
    std::string ns;
    double best_score_;

    // Bounded Box
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    double bounded_radius;

    // Tree Parameters
    int N_max;
    int N_termination;
    double radius;
    double step_size;
    double min_edge_length_;
    bool fixed_step;   // false = classic RRT (edge <= step_size); true = every edge exactly step_size
    double tolerance;
    int num_yaw_samples;
    bool local_rrt_star;   // false = RRT (nearest parent); true = RRT* (choose-parent + rewire)

    // Gain-evaluation options
    bool marginal_gain;
    bool optimize_yaw;          // false = keep each node's random sampled yaw; true = pick argmax yaw per node (like AEP)
    std::string eval_compute;   // "gpu" or "cpu"
    bool marginal_split;
    std::string objective_;
    bool benchmark_mode;

    // Benchmark accumulators (reset each NBV cycle)
    double bench_ms_gall_gpu, bench_ms_gall_split, bench_ms_g1p_cpu, bench_ms_abs_gpu, bench_ms_abs_cpu;
    double bench_g1p_err_sum, bench_g1p_err_max;
    int    bench_nodes;
    float  last_marg_kernel_ms_, last_abs_kernel_ms_;
    double bench_kernel_gall_gpu_, bench_kernel_abs_gpu_;

    // GPU map cache (for gpu / flat-map fixed-yaw eval)
    std::vector<uint8_t> flat_map_;
    Eigen::Vector3d map_origin_;
    Eigen::Vector3i map_dim_;

    int  replan_count_;
    double     timing_after_s_;
    bool       nbv_started_;
    bool       x2_timing_window_;
    ros::Time  nbv_start_time_;
    int        x2_capture_count_;
    int        x2_capture_max_;

    // Timer Parameters
    double timer_main_rate;

    // Camera Parameters
    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;

    // Planner Parameters
    double uav_radius;
    double collision_check_resolution_;   // [m] edge-sampling spacing for isEdgeCollisionFree (default 0.2)
    // --- optimistic-edges gate + IN-PLANNER backtrack (bounds the tree-build loop so NBV() can't spin) ---
    bool   optimistic_edges_ = true;      // set each replan: unknown=traversable only for the first replans
    int    optimistic_iterations_;        // # of initial replans allowed to plan through unknown space
    bool   recovery_enabled_ = true;      // master toggle for the in-planner backtrack (OFF for benchmark idle runs)
    double recovery_boxed_deadline_;      // [s] if the tree is still tiny after this, backtrack (boxed in)
    int    recovery_min_tree_;            // "tree still empty" node count for the boxed-in check
    double recovery_timeout_;             // [s] hard deadline: backtrack after this no matter what
    double lambda;

    // Tree variables
    std::vector<Eigen::Vector4d> path;
    std::vector<Eigen::Vector4d> prev_best_branch;
    std::vector<Eigen::Vector4d> best_branch;
    rrt_star::Node* next_best_node = nullptr;
    std::unique_ptr<rrt_star::Node> previous_node;  // owning copy, survives clearKDTree()
    std::vector<Eigen::Vector4d> executed_path_;    // flown poses (forward moves); boxed-in backtrack retreats along it
    bool retreating_ = false;                       // set only by a backtrack, cleared each STATE_PLANNING cycle
    std::unique_ptr<rrt_star::Node> retreat_node_;  // holds the current retreat pose
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point;
    Eigen::Vector4d next_start;

    int execution_horizon_;
    std::vector<Eigen::Vector4d> exec_waypoints_;
    size_t exec_index_ = 0;
    int exec_horizon_limit_ = 0;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    mrs_msgs::UavState uav_state;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;
    std::unique_ptr<mrs_msgs::Reference> current_waypoint_;

    // State variables
    std::atomic<State_t> state_;
    std::atomic<bool> ready_to_plan_  = false;

    // Visualization variables
    int node_id_counter_;
    int edge_id_counter_;
    int path_id_counter_;
    int collision_id_counter_;
    int iteration_;
    double total_planning_ms_ = 0.0;
    bool stats_written_ = false;

    // Instances
    rrt_star RRTStar;

    // Subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sub_control_manager_diag;
    mrs_lib::SubscribeHandler<mrs_msgs::UavState> sub_uav_state;

    // Publishers
    ros::Publisher pub_markers;
    ros::Publisher pub_reference;
    ros::Publisher pub_start;
    ros::Publisher pub_initial_reference;
    ros::Publisher pub_frustum;
    ros::Publisher pub_voxels;

    // Service servers
    ros::ServiceServer ss_start;
    ros::ServiceServer ss_stop;

    // Service clients
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv> sc_trajectory_generation;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference;

    // Timers
    ros::Timer timer_main;
};

#endif // PLANNER_H