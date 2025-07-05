#ifndef NBV_PLANNER_REAL_H
#define NBV_PLANNER_REAL_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/Reference.h>
#include <mrs_msgs/Vec1.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/PositionTarget.h>

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

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "REACHED"};

using vec3_t = mrs_lib::geometry::vec_t<3>;

class NBVPlanner {
public:
    NBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<std::shared_ptr<rrt_star::Node>>& path) const;
    void GetTransformation();

    void NBV();

    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);
    
    void changeState(const State_t new_state);
    double distance(Eigen::Vector4d current_pose, Eigen::Vector3d waypoint);

    void visualize_node(const Eigen::Vector4d& pos, const std::string& ns);
    void visualize_edge(const std::shared_ptr<rrt_star::Node> node, const std::string& ns);
    void visualize_path(const std::shared_ptr<rrt_star::Node> node, const std::string& ns);
    void visualize_frustum(std::shared_ptr<rrt_star::Node> position);
    void visualize_unknown_voxels(std::shared_ptr<rrt_star::Node> position);
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
    geometry_msgs::TransformStamped T_C_W_message;
    voxblox::Transformation T_C_W;
    geometry_msgs::TransformStamped T_W_C_message;
    voxblox::Transformation T_W_C;

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
    double tolerance;
    int num_yaw_samples;

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
    double lambda;

    // Tree variables
    //std::vector<std::shared_ptr<rrt_star::Node>> tree;
    std::vector<Eigen::Vector4d> path;
    std::vector<Eigen::Vector4d> prev_best_branch;
    std::vector<Eigen::Vector4d> best_branch;
    std::shared_ptr<rrt_star::Node> next_best_node;
    std::shared_ptr<rrt_star::Node> previous_root;
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    Eigen::Vector3d initial_offset;
    geometry_msgs::Pose uav_local_pose;
    mrs_msgs::Reference current_waypoint_;
    
    // Waypoint Following
    std::vector<mavros_msgs::PositionTarget> active_waypoints;
    size_t current_wp_idx = 0;
    bool has_active_path = false;
    Eigen::Vector3d current_waypoint;

    // State variables
    std::atomic<State_t> state_;
    std::atomic<bool> ready_to_plan_  = false;

    // Visualization variables
    int node_id_counter_;
    int edge_id_counter_;
    int path_id_counter_;
    int collision_id_counter_;
    int iteration_;

    // Instances
    rrt_star RRTStar;

    // Subscribers
    mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sub_local_pose_diag;

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
