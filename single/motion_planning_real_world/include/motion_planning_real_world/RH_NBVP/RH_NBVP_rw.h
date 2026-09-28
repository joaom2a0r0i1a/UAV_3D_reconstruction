#ifndef RH_NBVP_RW_H
#define RH_NBVP_RW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>

#include <minkindr_conversions/kindr_msg.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <gain_evaluation/gain_evaluator.h>
#include "motion_planning_real_world/planner_helpers_rw.h"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

typedef enum {
    STATE_IDLE,
    STATE_PLANNING,
    STATE_MOVING,
    STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "STOPPED"};

class RH_NBVP_rw {
  public:
    RH_NBVP_rw(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void planStep();

    // Gain Evaluation
    void evaluateGains(const std::vector<rrt_star::Node*>& nodes);
    std::vector<rrt_star::Node*> collectTreeNodes();
    void logTreeNodes();

    bool inBoundingBox(const Eigen::Vector4d& p) const;
    rrt_star::Node* expandTreeNode(rrt_star::Node* root_ptr);

    double distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b);
    void rotate();
    void explorationSweep();

    // Start Offset
    void captureOffset();
    mavros_msgs::PositionTarget makeSetpoint(const Eigen::Vector4d& waypoint);
    void commandWaypoint(const Eigen::Vector4d& waypoint, const Eigen::Vector4d& prev_waypoint);

    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg);
    void callbackVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
    void callbackState(const mavros_msgs::State::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);

    void changeState(const State_t new_state);

    void visualize_tree(const std::vector<rrt_star::Node*>& nodes);
    void visualize_path(rrt_star::Node* node);
    void visualize_frustum(const Eigen::Vector4d& waypoint, int id);
    void visualize_unknown_voxels(const Eigen::Vector4d& waypoint, int id_base);

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

    // Camera Extrinsics
    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    bool set_variables;

    // Transformations
    geometry_msgs::TransformStamped T_C_B_message;
    voxblox::Transformation T_C_B;

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
    float base_min_x, base_max_x, base_min_y, base_max_y, base_min_z, base_max_z;
    double bounded_radius;

    // Start Offset
    Eigen::Vector3d initial_offset{0.0, 0.0, 0.0};

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
    float last_marg_kernel_ms_, last_abs_kernel_ms_;

    // GPU Map Cache
    std::vector<uint8_t> flat_map_;
    Eigen::Vector3d map_origin_;
    Eigen::Vector3i map_dim_;

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
    double waypoint_reach_distance_;
    double waypoint_reach_velocity_;
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
    std::vector<Eigen::Vector4d> prev_best_branch;
    std::vector<Eigen::Vector4d> best_branch;
    rrt_star::Node* next_best_node = nullptr;
    std::unique_ptr<rrt_star::Node> previous_node;
    Eigen::Vector4d next_start;

    // Retreat Along Flown Path
    std::vector<Eigen::Vector4d> executed_path_;
    bool retreating_ = false;
    std::unique_ptr<rrt_star::Node> retreat_node_;

    // Single Step Execution
    std::vector<Eigen::Vector4d> exec_waypoints_;
    mavros_msgs::PositionTarget active_setpoint_;
    Eigen::Vector4d current_target_;
    bool has_active_setpoint_ = false;
    bool have_commanded_ = false;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    geometry_msgs::Pose uav_local_pose;
    ros::Time last_pose_time_;
    bool have_pose_ = false;
    bool prev_armed_ = false;
    double ground_z_ = 0.0;
    bool have_ground_z_ = false;
    double current_speed_ = 0.0;
    ros::Time last_vel_time_;
    bool have_vel_ = false;

    // Exploration Sweep
    bool exploration_initial_;
    double exploration_climb_;
    double exploration_settle_;
    bool exploration_return_;
    bool pending_exploration_ = false;
    double rotation_step_deg_;
    double rotation_settle_;

    // Pose Sanity Gates
    double pose_max_distance_;
    double pose_max_speed_;

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
    ros::Subscriber sub_local_pose;
    ros::Subscriber sub_velocity;
    ros::Subscriber sub_state;

    // Publishers
    ros::Publisher pub_markers;
    ros::Publisher pub_start;
    ros::Publisher pub_frustum;
    ros::Publisher pub_voxels;
    ros::Publisher pub_setpoint;
    ros::Publisher pub_offset;

    // Service servers
    ros::ServiceServer ss_start;
    ros::ServiceServer ss_stop;
    ros::ServiceServer ss_offset;

    // Timers
    ros::Timer timer_main;
};

#endif  // RH_NBVP_RW_H
