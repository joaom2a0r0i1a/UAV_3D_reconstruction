#ifndef AEP_PLANNER_H
#define AEP_PLANNER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/DynamicsConstraints.h>
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
  STATE_INITIALIZE,
  STATE_WAITING_INITIALIZE,
  STATE_PLANNING,
  STATE_MOVING,
  STATE_STOPPED,
} State_t;

const std::string _state_names_[] = {"IDLE", "INITIALIZE", "WAITING", "PLANNING", "MOVING", "REACHED"};

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
    double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
    void initialize(mrs_msgs::ReferenceStamped initial_reference);
    void rotate();
    
    bool callbackStart(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool callbackStop(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    //bool callbackReevaluate(cache_nodes::Reevaluate::Request& req, cache_nodes::Reevaluate::Response& res);
    void callbackControlManagerDiag(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);
    void callbackUavState(const mrs_msgs::UavState::ConstPtr msg);
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
    double bounded_radius;

    // RRT Parameters
    int N_max;
    int N_termination;
    double radius;
    double step_size;
    double tolerance;
    int num_yaw_samples;
    double g_zero;

    // Log files
    int num_nodes_count;
    std::ofstream outfile;

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

    mrs_msgs::Reference current_waypoint_;

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
    Eigen::Vector4d pose;
    double distance_;
    mrs_msgs::UavState uav_state;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;

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

    // Instances
    GainEvaluator segment_evaluator;
    rrt_star RRTStar;
    kd_tree goals_tree;

    // Subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sub_control_manager_diag;
    mrs_lib::SubscribeHandler<mrs_msgs::UavState> sub_uav_state;

    // Publishers
    ros::Publisher pub_markers;
    ros::Publisher pub_start;
    ros::Publisher pub_reference;
    ros::Publisher pub_node;
    ros::Publisher pub_initial_reference;
    ros::Publisher pub_frustum;
    ros::Publisher pub_voxels;

    // Service servers
    ros::ServiceServer ss_start;
    ros::ServiceServer ss_stop;

    // Service clients
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv> sc_trajectory_generation;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference;
    mrs_lib::ServiceClientHandler<cache_nodes::BestNode> sc_best_node;

    // Timers
    ros::Timer timer_main;
};

#endif // AEP_PLANNER_H