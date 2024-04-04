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

        //mrs_lib::ParamLoader param_loader(nh, "planner");

        // Load parameters
        nh.getParam("frame_id", frame_id);
        nh.getParam("center/x", center_x);
        nh.getParam("center/y", center_y);
        nh.getParam("center/z", center_z);
        nh.getParam("dimensions/x", dimensions_x);
        nh.getParam("dimensions/y", dimensions_y);
        nh.getParam("num_nodes", num_nodes);
        nh.getParam("rrt/radius", radius);
        nh.getParam("rrt/step_size", step_size);
        nh.getParam("rrt/tolerance", tolerance);
        nh.getParam("timer_main/rate", timer_main_rate);

        pose << 20, 0, 0;


        /*Initial Call*/
        // subscribers
        mrs_lib::SubscribeHandler<mrs_msgs::UavState>                  sub_uav_state;
        mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sub_control_manager_diag;

        // publishers
        ros::Publisher pub_markers;

        // service servers
        ros::ServiceServer ss_start;

        // service clients
        mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>             sc_trajectory_generation;
        mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference;


        /*Initialization*/
        // Initialize publishers
        pub_markers = nh.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);

        // Initialize subscribers
        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh                 = nh;
        shopts.node_name          = "planner";
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe         = true;
        shopts.autostart          = true;
        shopts.queue_size         = 10;
        shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

        sub_uav_state            = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &Planner::callbackUavState, this);
        sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &Planner::callbackControlManagerDiagnostics, this);

        // Initialize service servers
        ss_start = nh.advertiseService("start_in", &Planner::callbackStart, this);

        // Initialize service clients
        sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh, "trajectory_generation_out");
        sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh, "trajectory_reference_out");

        // Initialize timer
        timer_main = nh.createTimer(ros::Duration(1.0 / timer_main_rate), &Planner::timerMain, this);

        // Initialize other variables
        is_initialized = true;

        // Spin
        ros::spin();
    }

    std::tuple<mrs_msgs::GetPathSrv, std::vector<Eigen::Vector3d>> rh(std::vector<Eigen::Vector3d>& prev_best_branch, const Eigen::Vector3d& goal) {
        ROS_INFO("[planner]: executing path");
        int N_max = num_nodes;
        double dim_x = dimensions_x;
        double dim_y = dimensions_y;
        double radius = this->radius;
        double step_size = this->step_size;
        rrt_star::Node root(pose);
        std::vector<rrt_star::Node*> tree;
        tree.push_back(&root);
        std::vector<Eigen::Vector3d> best_branch;
        rrt_star::Node* next_best_node;
        if (prev_best_branch.size() > 0) {
            for (size_t i = 2; i < prev_best_branch.size(); ++i) {
                rrt_star::Node* nearest_node = rrt_star::findNearest(tree, {prev_best_branch[i].x(), prev_best_branch[i].y()});
                rrt_star::Node* new_node = rrt_star::steer(nearest_node, {prev_best_branch[i].x(), prev_best_branch[i].y()}, step_size);
                visualize_node(new_node->point.head(2), 50 + i, ns);
                std::vector<rrt_star::Node*> nearby_nodes = rrt_star::findNearby(tree, new_node, radius);
                new_node = rrt_star::chooseParent(new_node, nearby_nodes);
                tree.push_back(new_node);
                visualize_edge(new_node, 100 + i, ns);
                rrt_star::rewire(tree, new_node, nearby_nodes, radius);
            }
        }
        for (int j = 0; j < N_max; ++j) {
            Eigen::Vector2d rand_point = rrt_star::sampleSpace(dim_x, dim_y);
            rrt_star::Node* nearest_node = rrt_star::findNearest(tree, rand_point);
            rrt_star::Node* new_node = rrt_star::steer(nearest_node, rand_point, step_size);
            visualize_node(new_node->point.head(2), j, ns);
            std::vector<rrt_star::Node*> nearby_nodes = rrt_star::findNearby(tree, new_node, radius);
            new_node = rrt_star::chooseParent(new_node, nearby_nodes);
            tree.push_back(new_node);
            visualize_edge(new_node, j, ns);
            rrt_star::rewire(tree, new_node, nearby_nodes, radius);
            //printf("Vector: [%.2f, %.2f, %.2f]\n", new_node->point.x(), new_node->point.y(), new_node->point.z());
            /*if (j == N_max - 1) {
                std::vector<double> distances;
                distances.reserve(tree.size());
                for (const auto& node : tree) {
                    distances.push_back((goal.head(2) - node->point.head(2)).norm());
                }
                auto min_it = std::min_element(distances.begin(), distances.end());
                int min_index = std::distance(distances.begin(), min_it);
                rrt_star::Node* min_cost_node = tree[min_index];
                std::tie(best_branch, next_best_node) = rrt_star::backtrackPathNode(min_cost_node);
                visualize_path(next_best_node, ns);
            }*/
        }

        std::vector<double> distances;
        distances.reserve(tree.size());
        for (const auto& node : tree) {
            distances.push_back((goal.head(2) - node->point.head(2)).norm());
        }
        auto min_it = std::min_element(distances.begin(), distances.end());
        int min_index = std::distance(distances.begin(), min_it);
        rrt_star::Node* min_cost_node = tree[min_index];
        std::tie(best_branch, next_best_node) = rrt_star::backtrackPathNode(min_cost_node);
        visualize_path(next_best_node, ns);

        tree.clear();

        printf("Vector: [%.2f, %.2f, %.2f]\n", next_best_node->point.x(), next_best_node->point.y(), next_best_node->point.z());


        mrs_msgs::GetPathSrv path_msg;
        convertPathToROSMessage(path_msg, next_best_node);
        return std::make_tuple(path_msg, best_branch);
    }

    void convertPathToROSMessage(mrs_msgs::GetPathSrv& path_msg, rrt_star::Node* next_best_node) {
        path_msg.request.path.header.frame_id = "uav1/" + frame_id;
        path_msg.request.path.header.stamp = ros::Time::now();
        path_msg.request.path.fly_now = true;
        path_msg.request.path.use_heading = true;

        mrs_msgs::Reference reference;
        reference.position.x = next_best_node->point[0];
        reference.position.y = next_best_node->point[1];
        reference.position.z = center_z;
        reference.heading = 0; 

        path_msg.request.path.points.push_back(reference);
    }


    bool callbackStart(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {
        if (!is_initialized) {
            res.success = false;
            res.message = "not initialized";
            return true;
        }

        Eigen::Vector3d goal = {25, 27, 0};

        while ((goal.head(2) - pose.head(2)).norm() > tolerance) {
            if (control_manager_diag.tracker_status.have_goal) {
                ros::Duration(2).sleep();
            } else {
                std::vector<Eigen::Vector3d> prev_best_branch;
                mrs_msgs::GetPathSrv srv_get_path;
                std::tie(srv_get_path, prev_best_branch) = rh(prev_best_branch, goal);

                bool success = sc_trajectory_generation.call(srv_get_path);

                if (!success) {
                    ROS_ERROR("[planner]: service call for trajectory failed");
                    break;
                } else {
                    if (!srv_get_path.response.success) {
                        ROS_ERROR("[planner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
                        break;
                    }
                }

                mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
                srv_trajectory_reference.request.trajectory = srv_get_path.response.trajectory;
                srv_trajectory_reference.request.trajectory.fly_now = true;

                bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

                if (!success_trajectory) {
                    ROS_ERROR("[planner]: service call for trajectory reference failed");
                    break;
                } else {
                    if (!srv_trajectory_reference.response.success) {
                        ROS_ERROR("[planner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                        break;
                    }
                }

                ros::Duration(timer_main_rate).sleep();
            }
        }

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
        pose << uav_state.pose.position.x, uav_state.pose.position.y, 0.0;
    }

    void timerMain(const ros::TimerEvent& event) {
        if (!is_initialized) {
            return;
        }

        ROS_INFO_ONCE("[planner]: main timer spinning");

        /*if (!sh_control_manager_diag_.hasMsg()) {
            std::stringstream ss;
            ss << "missing control manager diagnostics";
            ROS_ERROR_STREAM_THROTTLE(1.0, "[TrajectoryGeneration]: " << ss.str());
            return;
        }*/

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
    bool is_initialized;
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
    ros::Publisher pub_markers;
    ros::Subscriber sub_control_manager_diag;
    ros::Subscriber sub_uav_state;
    ros::ServiceServer ss_start;
    ros::ServiceClient sc_trajectory_generation;
    ros::ServiceClient sc_trajectory_reference;
    ros::Timer timer_main;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;
    mrs_msgs::UavState uav_state;
    Eigen::Vector3d pose;

    mrs_msgs::GetPathSrv path_msg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    Planner planner;
    return 0;
}
