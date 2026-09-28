#ifndef RH_NBVP_H
#define RH_NBVP_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/Reference.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/msg_extractor.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>

#include <minkindr_conversions/kindr_msg.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <gain_evaluation/gain_evaluator.h>
#include "motion_planning/planner_helpers.h"

#include <chrono>
#include <algorithm>
#include <cmath>

typedef enum {
    STATE_IDLE,
    STATE_INITIALIZE,
    STATE_WAITING_INITIALIZE,
    STATE_PLANNING,
    STATE_MOVING,
    STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "INITIALIZE", "WAITING", "PLANNING", "MOVING", "REACHED"};

class RH_NBVP {
  public:
    RH_NBVP(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void planStep();

    // Gain Evaluation
    void evaluateGains(const std::vector<rrt_star::Node*>& nodes);
    void benchmarkGains(const std::vector<rrt_star::Node*>& nodes, const char* phase = "nbvp");
    std::vector<rrt_star::Node*> collectTreeNodes();
    void logTreeNodes();

    bool inBoundingBox(const Eigen::Vector4d& p) const;
    rrt_star::Node* expandTreeNode(rrt_star::Node* root_ptr);

    double distance(const std::unique_ptr<mrs_msgs::Reference>& waypoint, const geometry_msgs::Pose& pose);
    void initialize(mrs_msgs::ReferenceStamped initial_reference);
    void rotate();

    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
    void callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);

    void changeState(const State_t new_state);

    void visualize_tree(const std::vector<rrt_star::Node*>& nodes, const std::string& ns);
    void visualize_path(rrt_star::Node* node, const std::string& ns);
    void visualize_frustum(const Eigen::Vector4d& waypoint, int id);
    void visualize_unknown_voxels(const Eigen::Vector4d& waypoint, int id_base);

    void commandWaypoint(const Eigen::Vector4d& waypoint, const Eigen::Vector4d& prev_waypoint);
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
    bool fixed_step;
    double tolerance;
    int num_yaw_samples;

    // Gain Evaluation Options
    bool marginal_gain;
    bool optimize_yaw;
    std::string eval_compute;
    bool marginal_split;
    std::string objective_;

    // Benchmark
    bool benchmark_mode;
    std::string bench_suite_ = "timing";
    planner_helpers::BenchAccum bench_;
    float last_marg_kernel_ms_, last_abs_kernel_ms_;

    // GPU Map Cache
    std::vector<uint8_t> flat_map_;
    Eigen::Vector3d map_origin_;
    Eigen::Vector3i map_dim_;

    // Timing Capture
    int replan_count_;
    double timing_after_s_;
    bool nbv_started_;
    bool timing_window_;
    int capture_count_;
    int capture_max_;

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
    double collision_check_resolution_;
    double lambda;

    // Optimistic Edges
    bool optimistic_edges_ = true;
    int optimistic_iterations_;

    // Recovery
    bool recovery_enabled_ = true;
    double recovery_boxed_deadline_;
    int recovery_min_tree_;
    double recovery_timeout_;

    // Tree variables
    std::vector<Eigen::Vector4d> path;
    std::vector<Eigen::Vector4d> prev_best_branch;
    std::vector<Eigen::Vector4d> best_branch;
    rrt_star::Node* next_best_node = nullptr;
    std::unique_ptr<rrt_star::Node> previous_node;
    Eigen::Vector4d trajectory_point;
    Eigen::Vector4d next_start;

    // Retreat Along Flown Path
    std::vector<Eigen::Vector4d> executed_path_;
    bool retreating_ = false;
    std::unique_ptr<rrt_star::Node> retreat_node_;

    // Execution Horizon
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
    std::atomic<bool> ready_to_plan_ = false;

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

    // Timers
    ros::Timer timer_main;
};

#endif  // RH_NBVP_H