#ifndef AEP_PLANNER_REAL_H
#define AEP_PLANNER_REAL_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/Reference.h>
#include <mrs_msgs/Vec1.h>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/scope_timer.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/msg_extractor.h>
#include <mrs_lib/geometry/misc.h>

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
#include <eth_mav_msgs/eigen_mav_msgs.h>

#include <Eigen/Core>
#include <rrt_construction/rrt_star_kd.h>
#include <rrt_construction/kd_tree.h>
#include <rrt_construction/gain_evaluator.h>

#include <fstream>
#include <string>
#include <ctime>
#include <sstream>
#include <chrono>

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING", "REACHED"};

using vec3_t = mrs_lib::geometry::vec_t<3>;

class AEPlanner {
public:
    AEPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    double getMapDistance(const Eigen::Vector3d& position) const;
    bool isPathCollisionFree(const std::vector<std::shared_ptr<rrt_star::Node>>& path) const;
    void GetTransformation();

    void AEP();
    void localPlanner();
    void globalPlanner(const std::vector<Eigen::Vector3d>& GlobalFrontiers, std::shared_ptr<rrt_star::Node>& best_global_node);

    void getGlobalFrontiers(std::vector<Eigen::Vector3d>& GlobalFrontiers);
    bool getGlobalGoal(const std::vector<Eigen::Vector3d>& GlobalFrontiers, const std::shared_ptr<rrt_star::Node>& node);
    void getBestGlobalPath(const std::vector<std::shared_ptr<rrt_star::Node>>& global_goals, std::shared_ptr<rrt_star::Node>& best_global_node);

    void cacheNode(std::shared_ptr<rrt_star::Node> Node);
    double distance(Eigen::Vector4d current_pose, Eigen::Vector3d waypoint);
    
    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackOffset(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    void callbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr msg);
    void timerMain(const ros::TimerEvent& event);
    
    void changeState(const State_t new_state);

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
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;

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

    // RRT Parameters
    int N_max;
    int N_termination;
    double radius;
    double step_size;
    double tolerance;
    int num_yaw_samples;
    double g_zero;

    // RRT* Parameters
    int N_min_nodes;
    bool goto_global_planning;

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
    double global_lambda;
    std::atomic<int> replanning_counter_ = 0;

    // Local Planner variables
    //std::vector<std::shared_ptr<rrt_star::Node>> tree;
    std::vector<std::shared_ptr<rrt_star::Node>> best_branch;
    std::shared_ptr<rrt_star::Node> previous_root;
    std::shared_ptr<rrt_star::Node> next_best_node;
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point;

    // Global Planner variables
    std::shared_ptr<rrt_star::Node> best_global_node;
    std::vector<Eigen::Vector3d> GlobalFrontiers;

    // UAV variables
    bool is_initialized = false;
    double distance_;
    Eigen::Vector4d pose;
    Eigen::Vector3d initial_offset;
    geometry_msgs::Pose uav_local_pose;
    mrs_msgs::Reference current_waypoint_;
    
    // Waypoint Following
    std::vector<mavros_msgs::PositionTarget> active_waypoints;
    size_t current_wp_idx = 0;
    bool has_active_path = false;

    // State variables
    std::atomic<State_t> state_;
    std::atomic<bool>    interrupted_ = false;
    std::atomic<bool> ready_to_plan_  = false;

    // Visualization variables
    int node_id_counter_;
    int edge_id_counter_;
    int path_id_counter_;
    int collision_id_counter_;
    int iteration_;
    
    bool go_terminate = false;

    // Instances
    rrt_star RRTStar;
    kd_tree goals_tree;

    // Subscribers
    mrs_lib::SubscribeHandler<geometry_msgs::PoseStamped> sub_local_pose_diag;

    // Publishers
    ros::Publisher pub_markers;
    ros::Publisher pub_start;
    ros::Publisher pub_node;
    ros::Publisher pub_setpoint;
    ros::Publisher pub_frustum;
    ros::Publisher pub_voxels;
    ros::Publisher pub_offset;

    // Service servers
    ros::ServiceServer ss_start;
    ros::ServiceServer ss_stop;
    ros::ServiceServer ss_offset;

    // Service clients
    mrs_lib::ServiceClientHandler<cache_nodes::BestNode> sc_best_node;

    // Timers
    ros::Timer timer_main;
};

#endif // AEP_PLANNER_REAL_H
