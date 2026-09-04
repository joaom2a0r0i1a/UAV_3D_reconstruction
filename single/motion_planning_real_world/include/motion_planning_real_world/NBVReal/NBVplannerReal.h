#ifndef NBV_PLANNER_REAL_H
#define NBV_PLANNER_REAL_H

// Real-world RH-NBVP: port of motion_planning/NBV/NBVplanner. Deltas vs sim:
// mavros PositionTarget instead of the MRS control stack, plain ROS/tf2 plumbing (no mrs_lib),
// start offset auto-captured and applied to the bounded box + gain box (setWorldOffset),
// no benchmark suites, execution horizon fixed to 1 (single step per replan).

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
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/utils/planning_utils.h>

#include <minkindr_conversions/kindr_msg.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <gain_evaluation/gain_evaluator.h>
#include "motion_planning_real_world/planner_helpers_real.h"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "STOPPED"};


class NBVPlanner {
public:
    NBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void NBV();

    // Fixed-yaw gain evaluation (duplicated from AEP; NBVP keeps each node's random yaw).
    void evaluateGains(const std::vector<rrt_star::Node*>& nodes);
    std::vector<rrt_star::Node*> collectTreeNodes();
    void logTreeNodes();

    bool inBoundingBox(const Eigen::Vector4d& p) const;
    rrt_star::Node* expandTreeNode(rrt_star::Node* root_ptr);

    double distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b);
    void rotate();
    void explorationSweep();

    // Snapshot the current pose as the start offset; shift bounded box + gain box; publish latched offset.
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

    // TF (body -> camera extrinsics)
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

    // Bounded Box (shifted by the start offset; base_* = pristine yaml values)
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    float base_min_x, base_max_x, base_min_y, base_max_y, base_min_z, base_max_z;
    double bounded_radius;

    // Start offset (captured on ~start; re-capturable via ~offset)
    Eigen::Vector3d initial_offset{0.0, 0.0, 0.0};

    // Tree Parameters
    int N_max;
    int N_termination;
    double radius;
    double step_size;
    double min_edge_length_;
    bool fixed_step;   // false = classic RRT (edge <= step_size); true = every edge exactly step_size
    double tolerance;
    int num_yaw_samples;

    // Gain-evaluation options
    bool marginal_gain;
    bool optimize_yaw;          // false = keep each node's random sampled yaw; true = pick argmax yaw per node (like AEP)
    std::string eval_compute;   // "gpu" or "cpu"
    bool marginal_split;
    std::string objective_;
    float last_marg_kernel_ms_, last_abs_kernel_ms_;

    // GPU map cache (for gpu / flat-map fixed-yaw eval)
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
    double collision_check_resolution_;   // [m] edge-sampling spacing for isEdgeCollisionFree
    double waypoint_reach_distance_;      // [m] a waypoint counts as reached within this dist (+ yaw < 0.4)
    double waypoint_reach_velocity_;      // [m/s] velocity gate: the target waypoint also requires speed below this (true stop-and-go)
    // --- optimistic-edges gate + IN-PLANNER backtrack (bounds the tree-build loop so NBV() can't spin) ---
    bool   optimistic_edges_ = true;      // set each replan: unknown=traversable only for the first replans
    int    optimistic_iterations_;        // # of initial replans allowed to plan through unknown space
    bool   recovery_enabled_ = true;      // master toggle for the in-planner backtrack
    double recovery_boxed_deadline_;      // [s] if the tree is still tiny after this, backtrack (boxed in)
    int    recovery_min_tree_;            // "tree still empty" node count for the boxed-in check
    double recovery_timeout_;             // [s] hard deadline: backtrack after this no matter what
    double lambda;

    // Tree variables
    std::vector<Eigen::Vector4d> prev_best_branch;
    std::vector<Eigen::Vector4d> best_branch;
    rrt_star::Node* next_best_node = nullptr;
    std::unique_ptr<rrt_star::Node> previous_node;  // owning copy, survives clearKDTree()
    std::vector<Eigen::Vector4d> executed_path_;    // flown poses (forward moves); boxed-in backtrack retreats along it
    bool retreating_ = false;                       // set only by a backtrack, cleared each STATE_PLANNING cycle
    std::unique_ptr<rrt_star::Node> retreat_node_;  // holds the current retreat pose
    Eigen::Vector4d next_start;

    // Single-step execution (execution horizon fixed to 1 in the real world)
    std::vector<Eigen::Vector4d> exec_waypoints_;
    mavros_msgs::PositionTarget active_setpoint_;
    Eigen::Vector4d current_target_;
    bool has_active_setpoint_ = false;
    bool have_commanded_ = false;   // true after the first commandWaypoint (root = next_start from then on)

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    geometry_msgs::Pose uav_local_pose;
    ros::Time last_pose_time_;
    bool have_pose_ = false;
    bool exploration_initial_;   // up-rotate-down before the first plan
    double exploration_climb_;
    double exploration_settle_;
    bool exploration_return_;
    bool pending_exploration_ = false;
    double rotation_step_deg_;   // recovery sweep step
    double rotation_settle_;     // wait per step
    bool prev_armed_ = false;    // for the disarmed to armed edge
    double ground_z_ = 0.0;      // local z while still on the ground
    bool have_ground_z_ = false;
    double pose_max_distance_;   // reject poses further than this from the origin
    double pose_max_speed_;      // reject poses implying a jump faster than this
    double current_speed_ = 0.0;
    ros::Time last_vel_time_;
    bool have_vel_ = false;

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

#endif // NBV_PLANNER_REAL_H
