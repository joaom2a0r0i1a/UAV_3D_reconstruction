#ifndef AEP_PLANNER_REAL_H
#define AEP_PLANNER_REAL_H

// Real-world AEP: port of motion_planning/AEP/AEPlanner. Deltas vs sim:
// mavros PositionTarget instead of the MRS control stack, plain ROS/tf2 plumbing (no mrs_lib),
// start offset auto-captured and applied to the bounded box + gain box (setWorldOffset),
// no benchmark suites, no auto-takeoff routine (user takes off, then calls ~start).

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
#include <sensor_msgs/PointCloud2.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/utils/planning_utils.h>

#include <cache_nodes/Node.h>
#include <cache_nodes/Query.h>
#include <cache_nodes/BestNode.h>
#include <cache_nodes/Reevaluate.h>

#include <minkindr_conversions/kindr_msg.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/kd_tree.h>
#include <gain_evaluation/gain_evaluator.h>
#include "motion_planning_real_world/planner_helpers_real.h"

#include <atomic>
#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <chrono>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "STOPPED"};


class AEPlanner {
public:
    AEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void AEP();
    void localPlannerGPU();
    void globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node*& best_global_node);

    void evaluateGains(const std::vector<rrt_star::Node*>& nodes);
    std::vector<rrt_star::Node*> collectTreeNodes();
    std::unordered_map<rrt_star::Node*, double> pathUnion(rrt_star::Node* root_ptr, bool use_marginal);
    void cacheHighGainNodes();
    void logTreeNodes();

    bool inBoundingBox(const Eigen::Vector4d& p) const;
    rrt_star::Node* expandTreeNode(rrt_star::Node* root_ptr);

    void getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers);
    bool getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, rrt_star::Node* node);
    void getBestGlobalPath(const std::vector<rrt_star::Node*>& global_goals, rrt_star::Node*& best_global_node);

    void cacheNode(rrt_star::Node* Node, double gain, double yaw);
    double distance(const Eigen::Vector4d& a, const Eigen::Vector4d& b);
    void rotate();

    // Snapshot the current pose as the start offset; shift bounded box + gain box; publish latched offset.
    void captureOffset();
    mavros_msgs::PositionTarget makeSetpoint(const Eigen::Vector4d& waypoint);

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
    void visualize_frustum(rrt_star::Node* position);
    void visualize_unknown_voxels(rrt_star::Node* position);

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
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;

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

    // RRT Parameters
    int N_max;
    int N_termination;
    double radius;
    double step_size;
    double min_edge_length_;
    double tolerance;
    int num_yaw_samples;
    double g_zero;

    // RRT* Parameters
    int N_min_nodes;
    bool goto_global_planning;
    std::string global_selection;   // goal pick: "cost" (nearest), "gain" (most info), or "score" (gain vs distance)

    // Gain-evaluation options (shared by local + global planner)
    bool marginal_gain;             // true: marginal gain (path sum in global); false: absolute gain
    std::string eval_compute;       // "gpu" (batched) or "cpu" (sequential)
    bool marginal_split;            // marginal+gpu: false = fused kernel, true = split kernel
    std::string objective_;

    // Timer Parameters
    double timer_main_rate;

    // Camera Parameters
    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;
    double camera_pitch_deg;
    double camera_pitch;   // downward camera pitch [rad], loaded from camera/pitch [deg]

    // Planner Parameters
    double uav_radius;
    double collision_check_resolution_;   // [m] edge-sampling spacing for isEdgeCollisionFree
    double waypoint_reach_distance_;      // [m] advance to next waypoint within this dist (+ yaw < 0.4)
    double waypoint_reach_velocity_;      // [m/s] velocity gate: the FINAL waypoint also requires speed below this (true stop-and-go)
    // --- optimistic-edges gate + IN-PLANNER backtrack (bounds the tree-build loop so AEP() can't spin) ---
    bool   optimistic_edges_ = true;      // set each replan: unknown=traversable only for the first replans
    int    optimistic_iterations_;        // # of initial replans allowed to plan through unknown space
    bool   recovery_enabled_ = true;      // master toggle for the in-planner backtrack
    double recovery_boxed_deadline_;      // [s] if the tree is still tiny after this, backtrack (boxed in)
    int    recovery_min_tree_;            // "tree still empty" node count for the boxed-in check
    double recovery_timeout_;             // [s] hard deadline: backtrack after this no matter what
    double lambda;
    double global_lambda;

    // GPU Optimization - Flatten Map
    Eigen::Vector3d map_origin_;
    Eigen::Vector3i map_dim_;
    std::vector<uint8_t> flat_map_;
    float last_marg_kernel_ms_ = 0.0f, last_abs_kernel_ms_ = 0.0f;

    // Backtrack
    bool backtrack = false;

    // Waypoint chain of the chosen branch, flown via mavros setpoints
    std::vector<Eigen::Vector4d> waypoints_;
    size_t waypoint_index_ = 0;
    bool have_commanded_ = false;   // true after the first executed plan (root = next_start from then on)

    // Local Planner variables. best_branch owns its nodes (outlives clearKDTree()); next_best_node is non-owning.
    std::vector<std::unique_ptr<rrt_star::Node>> best_branch;
    rrt_star::Node* next_best_node = nullptr;

    std::vector<Eigen::Vector4d> executed_path_;      // flown poses (forward moves); boxed-in backtrack retreats along it
    bool retreating_ = false;                         // set only by a backtrack, cleared each STATE_PLANNING cycle
    std::unique_ptr<rrt_star::Node> retreat_node_;    // holds the current retreat pose (Node has no default ctor)
    Eigen::Vector4d trajectory_point;
    Eigen::Vector4d next_start;

    // Global Planner variables
    rrt_star::Node* best_global_node = nullptr;
    std::vector<Eigen::Vector3d> GlobalFrontiers;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    geometry_msgs::Pose uav_local_pose;
    ros::Time last_pose_time_;
    bool have_pose_ = false;
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
    kd_tree goals_tree;

    // Subscribers
    ros::Subscriber sub_local_pose;
    ros::Subscriber sub_velocity;
    ros::Subscriber sub_state;

    // Publishers
    ros::Publisher pub_markers;
    ros::Publisher pub_start;
    ros::Publisher pub_node;
    ros::Publisher pub_frustum;
    ros::Publisher pub_voxels;
    ros::Publisher pub_gpu_debug;
    ros::Publisher pub_setpoint;
    ros::Publisher pub_offset;

    // Service servers
    ros::ServiceServer ss_start;
    ros::ServiceServer ss_stop;
    ros::ServiceServer ss_offset;

    // Service clients
    ros::ServiceClient sc_best_node;

    // Timers
    ros::Timer timer_main;
};

#endif // AEP_PLANNER_REAL_H
