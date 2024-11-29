#ifndef KINO_NBV_PLANNER_REAL_H
#define KINO_NBV_PLANNER_REAL_H

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

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/attitude_converter.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/utils/planning_utils.h>

#include <minkindr_conversions/kindr_msg.h>
#include <eth_mav_msgs/eigen_mav_msgs.h>

#include <Eigen/Core>
#include <motion_planning_python/RRT/kino_rrt_star_kd.h>
#include <motion_planning_python/gain_evaluator.h>
#include <motion_planning_python/llaToenu.hpp>

#include <GeographicLib/Geoid.hpp>

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

class KinoNBVPlanner {
public:
    KinoNBVPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isTrajectoryCollisionFree(const std::shared_ptr<kino_rrt_star::Trajectory>& trajectory) const;
    void GetTransformation();

    void KinoNBV();
    
    double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
    void initialize(geographic_msgs::GeoPoseStamped initial_reference);
    void rotate();

    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackGlobalState(const sensor_msgs::NavSatFix::ConstPtr msg);
    void callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg);
    void callbackLocalVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
    void callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);
    
    void changeState(const State_t new_state);

    void visualize_node(const Eigen::Vector4d& pos, double size, const std::string& ns);
    void visualize_trajectory(const std::shared_ptr<kino_rrt_star::Trajectory> trajectory, const std::string& ns);
    void visualize_best_trajectory(const std::shared_ptr<kino_rrt_star::Trajectory> trajectory, const std::string& ns);
    void visualize_frustum(std::shared_ptr<kino_rrt_star::Node> position);
    void visualize_unknown_voxels(std::shared_ptr<kino_rrt_star::Node> position);
    void clear_node();
    void clear_all_voxels();
    void clearMarkers();

private:
    // Node Handles
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

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
    float planner_range;
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
    double lambda2;
    int max_accel_iterations;
    bool reset_velocity;
    std::atomic<int> replanning_counter_ = 0;

    // Tree variables
    std::vector<std::shared_ptr<kino_rrt_star::Trajectory>> best_branch;
    std::shared_ptr<kino_rrt_star::Trajectory> previous_trajectory;
    std::shared_ptr<kino_rrt_star::Trajectory> next_best_trajectory;
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point;

    // GPS Coordinates
    double latitude_ref, longitude_ref, altitude_ref;
    double latitude, longitude, altitude;
    double x_gps, y_gps, z_gps;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector4d pose;
    Eigen::Vector3d velocity;
    mrs_msgs::UavState uav_state;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;
    geometry_msgs::Pose uav_local_pose;
    mrs_msgs::Reference current_waypoint_;

    // State variables
    std::atomic<State_t> state_;
    std::atomic<bool>    interrupted_ = false;
    std::atomic<bool> ready_to_plan_  = false;

    // Visualization variables
    int node_id_counter_;
    int trajectory_id_counter_;
    int best_trajectory_id_counter_;
    int collision_id_counter_;
    int iteration_;

    // Instances
    GainEvaluator segment_evaluator;
    kino_rrt_star KinoRRTStar;

    // Subscribers
    mrs_lib::SubscribeHandler<sensor_msgs::NavSatFix> sub_global_diag;
    mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sub_local_pose_diag;
    mrs_lib::SubscribeHandler<geometry_msgs::TwistStamped> sub_local_velocity_diag;
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sub_control_manager_diag;

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

#endif // KINO_NBV_PLANNER_REAL_H