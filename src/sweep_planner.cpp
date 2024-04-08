#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/TrackerCommand.h>
#include <mrs_msgs/DynamicsConstraints.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/GetPathSrv.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/Vec1.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/service_client_handler.h>

#include <Eigen/Core>
#include "rrt_star_2d_yaw.h"

class Planner {
public:
    Planner() {
        ros::NodeHandle nh("~");

        ns = "uav1/";

        /* Parameter loading */
        mrs_lib::ParamLoader param_loader(nh, "planner");

        param_loader.loadParam("frame_id", frame_id);
        param_loader.loadParam("center/x", center_x);
        param_loader.loadParam("center/y", center_y);
        param_loader.loadParam("center/z", center_z);
        param_loader.loadParam("dimensions/x", dimensions_x);
        param_loader.loadParam("dimensions/y", dimensions_y);
        param_loader.loadParam("rrt/num_nodes", num_nodes);
        param_loader.loadParam("rrt/radius", radius);
        param_loader.loadParam("rrt/step_size", step_size);
        param_loader.loadParam("rrt/tolerance", tolerance);
        param_loader.loadParam("timer_main/rate", timer_main_rate);

        /* Publishers */
        pub_markers = nh.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);

        /* Subscribers */
        sub_uav_state = nh.subscribe("uav_state_in", 10, &Planner::callbackUavState, this);
        sub_control_manager_diag = nh.subscribe("control_manager_diag_in", 10, &Planner::callbackControlManagerDiagnostics, this);

        /* Service Servers */
        ss_start = nh.advertiseService("start_in", &Planner::callbackStart, this);

        /* Service Clients */
        sc_trajectory_generation = nh.serviceClient<mrs_msgs::GetPathSrv>("trajectory_generation_out");
        sc_trajectory_reference = nh.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("trajectory_reference_out");

        /* Timer */
        timer_main = nh.createTimer(ros::Duration(1.0 / timer_main_rate), &Planner::timerMain, this);

        is_initialized = true;
    }

    bool callbackStart(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {
        if (!is_initialized) {
            res.success = false;
            res.message = "not initialized";
            return true;
        }

        Eigen::Vector3d goal = {25, 27, 0};

        mrs_msgs::GetPathSrv srv_get_path;

        srv_get_path.request.path.header.frame_id = "uav1/" + frame_id;
        srv_get_path.request.path.header.stamp = ros::Time::now();
        srv_get_path.request.path.fly_now = false;
        srv_get_path.request.path.use_heading = true;

        mrs_msgs::Reference reference;

        for (int i = 0; i < 2; i++){
            for (int j = 10; j < 27; j++){
                reference.position.x = 20 + i;
                reference.position.y = j;
                reference.position.z = center_z;
                reference.heading = 0; 
                srv_get_path.request.path.points.push_back(reference);
            }
        }

        bool success = sc_trajectory_generation.call(srv_get_path);

        if (!success) {
            ROS_ERROR("[planner]: service call for trajectory failed");
            res.success = false;
            res.message = "Failed to call path service";
            return false;
        } else {
            if (!srv_get_path.response.success) {
                ROS_ERROR("[planner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
                res.success = false;
                res.message = "Path service returned failure";
                return false;
            }
        }

        mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
        srv_trajectory_reference.request.trajectory = srv_get_path.response.trajectory;
        srv_trajectory_reference.request.trajectory.fly_now = true;

        bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

        if (!success_trajectory) {
            ROS_ERROR("[planner]: service call for trajectory reference failed");
            res.success = false;
            res.message = "Failed to call trajectory service";
            return false;
        } else {
            if (!srv_trajectory_reference.response.success) {
                ROS_ERROR("[planner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                res.success = false;
                res.message = "Trajectory service returned failure";
                return false;
            }
        }

        ros::Duration(timer_main_rate).sleep();

        res.success = true;
        res.message = "starting";
        return true;

    }

    void callbackControlManagerDiagnostics(const mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {
        if (!is_initialized) {
            return;
        }
        ROS_INFO_ONCE("[planner]: getting ControlManager diagnostics");
        control_manager_diag = *msg;
    }

    void callbackUavState(const mrs_msgs::UavState::ConstPtr msg) {
        if (!is_initialized) {
            return;
        }
        ROS_INFO_ONCE("[planner]: getting UavState diagnostics");
        uav_state = *msg;
        pose = {uav_state.pose.position.x, uav_state.pose.position.y, 0.0};
    }

    void timerMain(const ros::TimerEvent& event) {
        if (!is_initialized) {
            return;
        }

        ROS_INFO_ONCE("[planner]: main timer spinning");

        if (control_manager_diag.tracker_status.have_goal) {
            ROS_INFO("[planner]: tracker has goal");
        } else {
            ROS_INFO("[planner]: waiting for command");
        }
    }

    void visualize_node(const Eigen::Vector3d& pos, int id, const std::string& ns) {
        visualization_msgs::Marker n;
        n.header.stamp = ros::Time::now();
        n.header.seq = id;
        n.header.frame_id = ns + frame_id;
        n.id = id;
        n.ns = "nodes";
        n.type = visualization_msgs::Marker::SPHERE;
        n.action = visualization_msgs::Marker::ADD;
        n.pose.position.x = pos[0];
        n.pose.position.y = pos[1];
        n.pose.position.z = center_z;

        n.pose.orientation.x = 1;
        n.pose.orientation.y = 0;
        n.pose.orientation.z = 0;
        n.pose.orientation.w = 0;

        n.scale.x = 0.2;
        n.scale.y = 0.2;
        n.scale.z = 0.2;

        n.color.r = 0.4;
        n.color.g = 0.7;
        n.color.b = 0.2;
        n.color.a = 1;

        n.lifetime = ros::Duration(5.0);
        n.frame_locked = false;
        pub_markers.publish(n);
    }

    void visualize_edge(const rrt_star::Node* node, int id, const std::string& ns) {
        visualization_msgs::Marker e;
        e.header.stamp = ros::Time::now();
        e.header.seq = id;
        e.header.frame_id = ns + frame_id;
        e.id = id;
        e.ns = "tree_branches";
        e.type = visualization_msgs::Marker::ARROW;
        e.action = visualization_msgs::Marker::ADD;
        e.pose.position.x = node->parent->point[0];
        e.pose.position.y = node->parent->point[1];
        e.pose.position.z = center_z;

        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        Eigen::Vector3d dir(node->point[0] - node->parent->point[0],
                            node->point[1] - node->parent->point[1],
                            node->point[2] - node->parent->point[2]);
        q.setFromTwoVectors(init, dir);
        q.normalize();

        e.pose.orientation.x = q.x();
        e.pose.orientation.y = q.y();
        e.pose.orientation.z = q.z();
        e.pose.orientation.w = q.w();

        e.scale.x = dir.norm();
        e.scale.y = 0.05;
        e.scale.z = 0.05;

        e.color.r = 1.0;
        e.color.g = 0.3;
        e.color.b = 0.7;
        e.color.a = 1.0;

        e.lifetime = ros::Duration(5.0);
        e.frame_locked = false;
        pub_markers.publish(e);
    }

    void visualize_path(const rrt_star::Node* node, const std::string& ns) {
        int id = 150;
        while (node->parent) {
            visualization_msgs::Marker p;
            p.header.stamp = ros::Time::now();
            p.header.seq = id;
            p.header.frame_id = ns + frame_id;
            p.id = id;
            p.ns = "path";
            p.type = visualization_msgs::Marker::ARROW;
            p.action = visualization_msgs::Marker::ADD;
            p.pose.position.x = node->parent->point[0];
            p.pose.position.y = node->parent->point[1];
            p.pose.position.z = node->parent->point[2];

            Eigen::Quaternion<double> q;
            Eigen::Vector3d init(1.0, 0.0, 0.0);
            Eigen::Vector3d dir(node->point[0] - node->parent->point[0],
                                node->point[1] - node->parent->point[1],
                                node->point[2] - node->parent->point[2]);
            q.setFromTwoVectors(init, dir);
            q.normalize();
            p.pose.orientation.x = q.x();
            p.pose.orientation.y = q.y();
            p.pose.orientation.z = q.z();
            p.pose.orientation.w = q.w();

            p.scale.x = dir.norm();
            p.scale.y = 0.07;
            p.scale.z = 0.07;

            p.color.r = 0.7;
            p.color.g = 0.7;
            p.color.b = 0.3;
            p.color.a = 1.0;

            p.lifetime = ros::Duration(100.0);
            p.frame_locked = false;
            pub_markers.publish(p);

            node = node->parent;
            id += 1;
        }
    }

private:
    // Parameters
    std::string frame_id;
    std::string ns;
    double center_x;
    double center_y;
    double center_z;
    double dimensions_x;
    double dimensions_y;
    int num_nodes;
    double radius;
    double step_size;
    double tolerance;
    double timer_main_rate;

    // State variables
    bool is_initialized = false;
    Eigen::Vector3d pose;
    mrs_msgs::UavState uav_state;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;
    mrs_msgs::ControlManagerDiagnostics::ConstPtr last_control_msg;

    // Subscribers
    ros::Subscriber sub_control_manager_diag;
    ros::Subscriber sub_uav_state;

    // Publishers
    ros::Publisher pub_markers;

    // Service servers
    ros::ServiceServer ss_start;

    // Service clients
    ros::ServiceClient sc_trajectory_generation;
    ros::ServiceClient sc_trajectory_reference;

    // Timers
    ros::Timer timer_main;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    Planner planner;
    ros::spin();
    return 0;
}
