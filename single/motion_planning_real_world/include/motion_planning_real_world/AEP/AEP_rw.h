#ifndef AEP_RW_H
#define AEP_RW_H

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
#include <voxblox_ros/esdf_server.h>

#include <cache_nodes/Node.h>
#include <cache_nodes/BestNode.h>

#include <minkindr_conversions/kindr_msg.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/kd_tree.h>
#include <gain_evaluation/gain_evaluator.h>
#include "motion_planning_real_world/planner_helpers_rw.h"

#include <atomic>
#include <fstream>
#include <string>
#include <sstream>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>

typedef enum {
    STATE_IDLE,
    STATE_PLANNING,
    STATE_MOVING,
    STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "STOPPED"};

class AEP_rw {
  public:
    AEP_rw(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<rrt_star::Node*>& path) const;
    bool isEdgeCollisionFree(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const;
    void GetTransformation();

    void planStep();
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
    void explorationSweep();

    // Start Offset
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
    std::string global_selection;

    // Gain Evaluation Options
    bool marginal_gain;
    std::string eval_compute;
    bool marginal_split;
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
    double camera_pitch;

    // Planner Parameters
    double uav_radius;
    double collision_check_resolution_;
    double waypoint_reach_distance_;
    double waypoint_reach_velocity_;
    double lambda;
    double global_lambda;

    // Optimistic Edges
    bool optimistic_edges_ = true;
    int optimistic_iterations_;

    // Recovery
    bool recovery_enabled_ = true;
    double recovery_boxed_deadline_;
    int recovery_min_tree_;
    double recovery_timeout_;

    // GPU Map Cache
    Eigen::Vector3d map_origin_;
    Eigen::Vector3i map_dim_;
    std::vector<uint8_t> flat_map_;
    float last_marg_kernel_ms_ = 0.0f, last_abs_kernel_ms_ = 0.0f;

    // Backtrack
    bool backtrack = false;

    // Waypoint Chain
    std::vector<Eigen::Vector4d> waypoints_;
    size_t waypoint_index_ = 0;
    bool have_commanded_ = false;

    // Local Planner variables
    std::vector<std::unique_ptr<rrt_star::Node>> best_branch;
    rrt_star::Node* next_best_node = nullptr;
    Eigen::Vector4d trajectory_point;
    Eigen::Vector4d next_start;

    // Retreat Along Flown Path
    std::vector<Eigen::Vector4d> executed_path_;
    bool retreating_ = false;
    std::unique_ptr<rrt_star::Node> retreat_node_;

    // Global Planner variables
    rrt_star::Node* best_global_node = nullptr;
    std::vector<Eigen::Vector3d> GlobalFrontiers;

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

#endif  // AEP_RW_H
