#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

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

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/ros_params.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>
#include <voxblox/utils/planning_utils.h>

#include <minkindr_conversions/kindr_msg.h>
#include <eth_mav_msgs/eigen_mav_msgs.h>

#include <Eigen/Core>
#include <rrt_star_2d_yaw.h>
#include <gain_evaluator.h>

typedef enum
{
  STATE_IDLE,
  STATE_PLANNING,
  STATE_MOVING,
} State_t;

const std::string _state_names_[] = {"IDLE", "PLANNING", "MOVING"};

class Planner {
public:
    Planner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_), lower_bound_(Eigen::Vector3d::Zero()),
      upper_bound_(Eigen::Vector3d::Zero()) {

        ns = "uav1";

        /* Parameter loading */
        mrs_lib::ParamLoader param_loader(nh_private_, "planner");

        // Frames, Coordinates and Dimensions
        param_loader.loadParam("frame_id", frame_id);
        param_loader.loadParam("body/frame_id", body_frame_id);
        param_loader.loadParam("camera/frame_id", camera_frame_id);
        param_loader.loadParam("center/x", center_x);
        param_loader.loadParam("center/y", center_y);
        param_loader.loadParam("center/z", center_z);
        param_loader.loadParam("dimensions/x", dimensions_x);
        param_loader.loadParam("dimensions/y", dimensions_y);

        // RRT Tree
        param_loader.loadParam("rrt/N_max", num_nodes);
        param_loader.loadParam("rrt/N_termination", N_termination);
        param_loader.loadParam("rrt/N_yaw_samples", num_yaw_samples);
        param_loader.loadParam("rrt/radius", radius);
        param_loader.loadParam("rrt/step_size", step_size);
        param_loader.loadParam("rrt/tolerance", tolerance);

        // Camera
        param_loader.loadParam("camera/h_fov", horizontal_fov);
        param_loader.loadParam("camera/width", resolution_x);
        param_loader.loadParam("camera/height", resolution_y);
        param_loader.loadParam("camera/min_distance", min_distance);
        param_loader.loadParam("camera/max_distance", max_distance);

        // Planner
        param_loader.loadParam("path/uav_radius", uav_radius);

        // Timer
        param_loader.loadParam("timer_main/rate", timer_main_rate);

        // Set Goal and yaw samples
        goal = {24, 5, 0};

        // Get vertical FoV and setup camera
        vertical_fov = gain_evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
        gain_evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);

        // Setup Voxblox
        tsdf_map_ = voxblox_server_.getTsdfMapPtr();
        esdf_map_ = voxblox_server_.getEsdfMapPtr();
        gain_evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());
                
        // Setup Tf Transformer
        transformer_ = std::make_unique<mrs_lib::Transformer>("planner");
        transformer_->setDefaultFrame(frame_id);
        transformer_->setDefaultPrefix(ns);
        transformer_->retryLookupNewest(true);

        get_T_C_B = false;

        // Setup Collision Avoidance
        voxblox_server_.setTraversabilityRadius(uav_radius);
        voxblox_server_.publishTraversable();
        
        /* Publishers */
        pub_markers = nh_private_.advertise<visualization_msgs::Marker>("visualization_marker_out", 50);

        /* Subscribers */
        mrs_lib::SubscribeHandlerOptions shopts;
        shopts.nh                 = nh_private_;
        shopts.node_name          = "planner";
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe         = true;
        shopts.autostart          = true;
        shopts.queue_size         = 10;
        shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

        sub_uav_state = mrs_lib::SubscribeHandler<mrs_msgs::UavState>(shopts, "uav_state_in", &Planner::callbackUavState, this);
        sub_control_manager_diag = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diag_in", &Planner::callbackControlManagerDiagnostics, this);
        //sub_tracker_cmd = mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand>(shopts, "tracker_cmd_in", &Planner::callbackTrackerCmd, this);
        sub_constraints = mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints>(shopts, "constraints_in");

        /* Service Servers */
        ss_start = nh_private_.advertiseService("start_in", &Planner::callbackStart, this);

        /* Service Clients */
        sc_trajectory_generation = mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv>(nh_private_, "trajectory_generation_out");
        sc_trajectory_reference = mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv>(nh_private_, "trajectory_reference_out");

        /* Timer */
        timer_main = nh_private_.createTimer(ros::Duration(1.0 / timer_main_rate), &Planner::timerMain, this);

        is_initialized = true;
    }

    double getMapDistance(const Eigen::Vector3d& position) const {
        if (!voxblox_server_.getEsdfMapPtr()) {
            return 0.0;
        }
        double distance = 0.0;
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
            return 0.0;
        }
        return distance;
    }

    bool isPathCollisionFree(const eth_mav_msgs::EigenTrajectoryPointVector& path) const {
        for (const eth_mav_msgs::EigenTrajectoryPoint& point : path) {
            if (getMapDistance(point.position_W) < uav_radius) {
                return false;
            }
        }
        return true;
    }

    /*bool isPathFeasible(const eth_mav_msgs::EigenTrajectoryPointVector& path) const {
    const double kExcessMargin = 0.01;
        for (const eth_mav_msgs::EigenTrajectoryPoint& point : path) {
            if (point.acceleration_W.norm() > constraints_.a_max + kExcessMargin) {
                return false;
            }
            if (point.velocity_W.norm() > constraints_.v_max + kExcessMargin) {
                return false;
            }
        }
        return true;
    }*/

    void GetTransformation() {
        // C stand for Camera, B stand for Body, W stand for World

        // From body to camera
        auto Message_C_B = transformer_->getTransform(body_frame_id, camera_frame_id, ros::Time(0));
        if (!Message_C_B) {
            ROS_ERROR_THROTTLE(1.0, "[planner]: could not get transform from body frame to the camera frame!");
            return;
        }

        T_C_B_message = Message_C_B.value();
        T_B_C_message = transformer_->inverse(T_C_B_message);
        //T_C_B Translation: [-0.011500, -0.089000, -0.155000]
        //T_C_B Rotation: [0.500000, -0.500000, 0.500000, 0.500000]

        /*T_C_B_message.transform.translation.x = -0.011500;
        T_C_B_message.transform.translation.y = -0.089000;
        T_C_B_message.transform.translation.z = -0.155000;

        T_C_B_message.transform.rotation.x = 0.500000;
        T_C_B_message.transform.rotation.y = -0.500000;
        T_C_B_message.transform.rotation.z = 0.500000;
        T_C_B_message.transform.rotation.w = 0.500000;*/

        // Transform into matrix for Voxblox
        tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
        tf::transformMsgToKindr(T_B_C_message.transform, &T_B_C);

        // From world to camera
        auto Message_C_W = transformer_->getTransform(frame_id, camera_frame_id, ros::Time(0));
        if (!Message_C_W) {
            ROS_ERROR_THROTTLE(1.0, "[planner]: could not get transform from world frame to the camera frame!");
            return;
        }

        T_C_W_message = Message_C_W.value();
        T_W_C_message = transformer_->inverse(T_C_W_message);

        // Transform into matrix for Voxblox
        tf::transformMsgToKindr(T_C_W_message.transform, &T_C_W);
        tf::transformMsgToKindr(T_W_C_message.transform, &T_W_C);

        //gain_evaluator.setCameraExtrinsics(T_C_B);
    }

    void computeMapBounds(Eigen::Vector3d* lower_bound, Eigen::Vector3d* upper_bound) const {
        if (esdf_map_) {
            voxblox::utils::computeMapBoundsFromLayer(*esdf_map_->getEsdfLayerPtr(),
                                                    lower_bound, upper_bound);
        } else if (tsdf_map_) {
            voxblox::utils::computeMapBoundsFromLayer(*tsdf_map_->getTsdfLayerPtr(),
                                                    lower_bound, upper_bound);
        }
    }

    void computeBoundingBox(const Eigen::Vector3d& lower_bound, const Eigen::Vector3d& upper_bound,
                        double& min_x, double& min_y, double& min_z,
                        double& max_x, double& max_y, double& max_z, double& radius) const {
        min_x = lower_bound.x();
        min_y = lower_bound.y();
        min_z = lower_bound.z();
        
        max_x = upper_bound.x();
        max_y = upper_bound.y();
        max_z = upper_bound.z();

        radius = std::sqrt(std::pow(min_x - max_x, 2) + std::pow(min_y - max_y, 2) + std::pow(min_z - max_z, 2));
    }

    void rh(const Eigen::Vector3d& goal) {
        int N_max = num_nodes;
        double dim_x = dimensions_x;
        double dim_y = dimensions_y;

        rrt_star::Node root(pose);
        tree.push_back(&root);
        clearMarkers();

        bool isFirstIteration = true;

        computeMapBounds(&lower_bound_, &upper_bound_);

        ROS_INFO_STREAM("Map bounds: " << lower_bound_.transpose() << " to "
                                        << upper_bound_.transpose() << " size: "
                                        << (upper_bound_ - lower_bound_).transpose());

        double min_x, min_y, min_z, max_x, max_y, max_z, radius; 
        computeBoundingBox(lower_bound_, upper_bound_, min_x, min_y, min_z, max_x, max_y, max_z, radius);
              
        // Push the nodes from the previous best branch into the tree
        for (size_t i = 1; i < prev_best_branch.size(); ++i) {
            const auto& node = prev_best_branch[i];
            rrt_star::Node* nearest_node = rrt_star::findNearest(tree, {prev_best_branch[i][0], prev_best_branch[i][1]});
            //rrt_star::Node* new_node = rrt_star::steer(nearest_node, {prev_best_branch[i][0], prev_best_branch[i][1]}, step_size);
            rrt_star::Node* new_node = new rrt_star::Node(node);
            visualize_node(new_node->point.head(2), ns);

            std::vector<rrt_star::Node*> nearby_nodes = rrt_star::findNearby(tree, new_node, radius);
            new_node = rrt_star::chooseParent(new_node, nearby_nodes);

            //computeGainFromsampledYaw(new_node, num_yaw_samples, trajectory_point);

            double best_gain = 0;
            double gain;
            double best_yaw;
            for (int k = 0; k < num_yaw_samples; ++k) {
                double yaw = k * 2 * M_PI / num_yaw_samples;
                trajectory_point.position_W = new_node->point.head(2);
                trajectory_point.setFromYaw(yaw);
                gain = gain_evaluator.evaluateExplorationGainWithRaycasting(trajectory_point);
                if (gain > best_gain) {
                    best_gain = gain;
                    best_yaw = yaw;
                }
            }

            
            if (isFirstIteration) {
                isFirstIteration = false;
                continue; // Skip first iteration
            }
            
            /*trajectory_point.position_W = new_node->point.head(2);
            trajectory_point.setFromYaw(new_node->point[2]);
            new_node->gain = gain_evaluator.computeGain(trajectory_point);*/
            
            tree.push_back(new_node);
            visualize_edge(new_node, ns);
            rrt_star::rewire(tree, new_node, nearby_nodes, radius);
        }

        /*for (size_t i = 0; i < tree.size(); ++i) {
            std::cout << "Random Point: " << tree[i]->point << std::endl;
        }*/
        
        prev_best_branch.clear();

        int j = 0;
        best_gain_ = 0;
        while (j < N_max || best_gain_ == 0) {
            Eigen::Vector2d rand_point = rrt_star::sampleSpace(dim_x, dim_y);
            //std::cout << "Random Point: " << rand_point << std::endl;

            rrt_star::Node* nearest_node = rrt_star::findNearest(tree, rand_point);
            rrt_star::Node* new_node = rrt_star::steer(nearest_node, rand_point, step_size);
            //std::cout << "New Point: " << new_node->point << std::endl;

            visualize_node(new_node->point.head(2), ns);
            std::vector<rrt_star::Node*> nearby_nodes = rrt_star::findNearby(tree, new_node, radius);
            new_node = rrt_star::chooseParent(new_node, nearby_nodes);

            eth_mav_msgs::EigenTrajectoryPoint::Vector trajectory_segment;

            trajectory_point.position_W.head(2) = new_node->parent->point.head(2);
            trajectory_point.position_W.z() = center_z;
            trajectory_point.setFromYaw(new_node->parent->point[2]);
            trajectory_segment.push_back(trajectory_point);

            trajectory_point.position_W.head(2) = new_node->point.head(2);
            trajectory_point.position_W.z() = center_z;
            trajectory_point.setFromYaw(new_node->point[2]);
            trajectory_segment.push_back(trajectory_point);

            bool success_collision = false;
            success_collision = isPathCollisionFree(trajectory_segment);

            /*if (!success_collision) {
                clear_node();
                trajectory_segment.clear();
                continue;
            }*/

            //trajectory_segment.clear();

            //computeGainFromsampledYaw(new_node, num_yaw_samples, trajectory_point);

            double best_gain = 0;
            double gain;
            double best_yaw;
            for (int k = 0; k < num_yaw_samples; ++k) {
                double yaw = k * 2 * M_PI / num_yaw_samples;
                trajectory_point.position_W = new_node->point.head(2);
                trajectory_point.setFromYaw(yaw);
                gain = gain_evaluator.evaluateExplorationGainWithRaycasting(trajectory_point);
                if (gain > best_gain) {
                    best_gain = gain;
                    best_yaw = yaw;
                }
            }
            new_node->gain = best_gain;
            new_node->point[2] = best_yaw;
            
            if (new_node->gain > best_gain_) {
                best_gain_ = new_node->gain;
            }

            ROS_INFO("[planner]: Best Gain: %f", new_node->gain);

            tree.push_back(new_node);
            visualize_edge(new_node, ns);
            rrt_star::rewire(tree, new_node, nearby_nodes, radius);

            if (j == N_max - 1) {
                double max_gain = -std::numeric_limits<double>::infinity();
                rrt_star::Node* best_node = nullptr;

                // Iterate over all nodes in the tree to find the node with the highest gain.
                for (const auto& node : tree) {
                    if (node->gain > max_gain) {
                        max_gain = node->gain;
                        best_node = node;
                    }
                }
                
                // If a node with the highest gain is found, backtrack to get the best branch.
                if (best_node) {
                    std::tie(best_branch, next_best_node) = rrt_star::backtrackPathNode(best_node);
                    visualize_path(best_node, ns);
                    prev_best_branch = best_branch;
                }
            }

            if (j > N_termination) {
                changeState(STATE_IDLE);
            }

            ++j;

        }

    }

    void computeGainFromsampledYaw(rrt_star::Node* node, int yaw_samples, eth_mav_msgs::EigenTrajectoryPoint& trajectory_point) {
        double best_gain = 0;
        double gain;
        double best_yaw;
        for (int k = 0; k < yaw_samples; ++k) {
            double yaw = k * 2 * M_PI / yaw_samples;
            trajectory_point.position_W = node->point.head(2);
            trajectory_point.setFromYaw(yaw);
            gain = gain_evaluator.computeGain(trajectory_point);
            if (gain > best_gain) {
                best_gain = gain;
                best_yaw = yaw;
            }
        }
        node->gain = best_gain;
        node->point[2] = best_yaw;
    }

    bool callbackStart(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {
        if (!is_initialized) {
            res.success = false;
            res.message = "not initialized";
            return true;
        }

        if (!ready_to_plan_) {
            std::stringstream ss;
            ss << "not ready to plan, missing data";

            ROS_ERROR_STREAM_THROTTLE(0.5, "[planner]: " << ss.str());

            res.success = false;
            res.message = ss.str();
            return true;
        }

        interrupted_ = false;
        changeState(STATE_PLANNING);

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
        pose = {uav_state.pose.position.x, uav_state.pose.position.y, -3.0};
    }

    /*void callbackTrackerCmd(const mrs_msgs::TrackerCommand::ConstPtr msg) {
        if (!is_initialized) {
            return;
        }
        ROS_INFO_ONCE("[planner]: getting tracker cmd");
        tracker_cmd = *msg;
    }*/

    void timerMain(const ros::TimerEvent& event) {
        if (!is_initialized) {
            return;
        }

        /* prerequsities //{ */

        const bool got_control_manager_diag = sub_control_manager_diag.hasMsg() && (ros::Time::now() - sub_control_manager_diag.lastMsgTime()).toSec() < 2.0;
        const bool got_uav_state = sub_uav_state.hasMsg() && (ros::Time::now() - sub_uav_state.lastMsgTime()).toSec() < 2.0;

        if (!got_control_manager_diag || !got_uav_state) {
            ROS_INFO_THROTTLE(1.0, "[planner]: waiting for data: ControlManager diag = %s, UavState = %s", got_control_manager_diag ? "TRUE" : "FALSE", got_uav_state ? "TRUE" : "FALSE");
            return;
        } else {
            ready_to_plan_ = true;
        }

        ROS_INFO_ONCE("[planner]: main timer spinning");

        if (!get_T_C_B) {
            GetTransformation();
            ROS_INFO("[planner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
            ROS_INFO("[planner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
            gain_evaluator.setCameraExtrinsics(T_C_B);
            get_T_C_B = true;
        }
        
        switch (state_) {
            case STATE_IDLE: {break;}
            case STATE_PLANNING: {
                if ((goal.head(2) - pose.head(2)).norm() < tolerance) {
                    ROS_INFO("[planner]: Goal Reached.");
                    changeState(STATE_IDLE);
                    break;
                }

                rh(goal);

                mrs_msgs::GetPathSrv srv_get_path;

                srv_get_path.request.path.header.frame_id = "uav1/" + frame_id;
                srv_get_path.request.path.header.stamp = ros::Time::now();
                srv_get_path.request.path.fly_now = false;
                srv_get_path.request.path.use_heading = true;

                mrs_msgs::Reference reference;

                reference.position.x = next_best_node->point[0];
                reference.position.y = next_best_node->point[1];
                reference.position.z = center_z;
                reference.heading = next_best_node->point[2];
                srv_get_path.request.path.points.push_back(reference);

                bool success = sc_trajectory_generation.call(srv_get_path);

                if (!success) {
                    ROS_ERROR("[planner]: service call for trajectory failed");
                    changeState(STATE_IDLE);
                    return;
                } else {
                    if (!srv_get_path.response.success) {
                        ROS_ERROR("[planner]: service call for trajectory failed: '%s'", srv_get_path.response.message.c_str());
                        changeState(STATE_IDLE);
                        return;
                    }
                }

                mrs_msgs::TrajectoryReferenceSrv srv_trajectory_reference;
                srv_trajectory_reference.request.trajectory = srv_get_path.response.trajectory;
                srv_trajectory_reference.request.trajectory.fly_now = true;

                bool success_trajectory = sc_trajectory_reference.call(srv_trajectory_reference);

                if (!success_trajectory) {
                    ROS_ERROR("[planner]: service call for trajectory reference failed");
                    changeState(STATE_IDLE);
                    return;
                } else {
                    if (!srv_trajectory_reference.response.success) {
                        ROS_ERROR("[planner]: service call for trajectory reference failed: '%s'", srv_trajectory_reference.response.message.c_str());
                        changeState(STATE_IDLE);
                        return;
                    }
                }

                tree.clear();
                best_branch.clear();
                ros::Duration(1.5).sleep();

                changeState(STATE_MOVING);
                break;
                
            }
            case STATE_MOVING: {
                if (control_manager_diag.tracker_status.have_goal) {
                    ROS_INFO("[planner]: tracker has goal");
                } else {
                    ROS_INFO("[planner]: waiting for command");
                    changeState(STATE_PLANNING);
                }
                break;
            }

            default: {
                if (control_manager_diag.tracker_status.have_goal) {
                    ROS_INFO("[planner]: tracker has goal");
                } else {
                    ROS_INFO("[planner]: waiting for command");
                }
                break;
            }
        }
    }

    void changeState(const State_t new_state) {
        const State_t old_state = state_;

        if (interrupted_ && old_state == STATE_IDLE) {
            ROS_WARN("[planner]: Planning interrupted, not changing state.");
            return;
        }

        switch (new_state) {

            case STATE_PLANNING: {

                if (old_state == STATE_IDLE) {
                    replanning_counter_ = 0;
                }
            }

            default: {break;}
        }

        ROS_INFO("[planner]: changing state '%s' -> '%s'", _state_names_[old_state].c_str(), _state_names_[new_state].c_str());

        state_ = new_state;
    }

    void visualize_node(const Eigen::Vector3d& pos, const std::string& ns) {
        visualization_msgs::Marker n;
        n.header.stamp = ros::Time::now();
        n.header.seq = node_id_counter_;
        n.header.frame_id = ns + "/" + frame_id;
        n.id = node_id_counter_;
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

        node_id_counter_++;

        n.lifetime = ros::Duration(30.0);
        n.frame_locked = false;
        pub_markers.publish(n);
    }

    void visualize_edge(const rrt_star::Node* node, const std::string& ns) {
        visualization_msgs::Marker e;
        e.header.stamp = ros::Time::now();
        e.header.seq = edge_id_counter_;
        e.header.frame_id = ns + "/" + frame_id;
        e.id = edge_id_counter_;
        e.ns = "tree_branches";
        e.type = visualization_msgs::Marker::ARROW;
        e.action = visualization_msgs::Marker::ADD;
        e.pose.position.x = node->parent->point[0];
        e.pose.position.y = node->parent->point[1];
        e.pose.position.z = center_z;

        Eigen::Quaternion<double> q;
        Eigen::Vector3d init(1.0, 0.0, 0.0);
        Eigen::Vector3d dir(node->point[0] - node->parent->point[0],
                            node->point[1] - node->parent->point[1], 0);
                            //node->point[2] - node->parent->point[2]);
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

        edge_id_counter_++;

        e.lifetime = ros::Duration(30.0);
        e.frame_locked = false;
        pub_markers.publish(e);
    }

    void visualize_path(const rrt_star::Node* node, const std::string& ns) {
        while (node->parent) {
            visualization_msgs::Marker p;
            p.header.stamp = ros::Time::now();
            p.header.seq = path_id_counter_;
            p.header.frame_id = ns + "/" + frame_id;
            p.id = path_id_counter_;
            p.ns = "path";
            p.type = visualization_msgs::Marker::ARROW;
            p.action = visualization_msgs::Marker::ADD;
            p.pose.position.x = node->parent->point[0];
            p.pose.position.y = node->parent->point[1];
            p.pose.position.z = center_z;//node->parent->point[2];

            Eigen::Quaternion<double> q;
            Eigen::Vector3d init(1.0, 0.0, 0.0);
            Eigen::Vector3d dir(node->point[0] - node->parent->point[0],
                                node->point[1] - node->parent->point[1], 0);
                                //node->point[2] - node->parent->point[2]);
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
            path_id_counter_++;
        }
    }

    void clear_node() {
        visualization_msgs::Marker clear_node;
        clear_node.header.stamp = ros::Time::now();
        clear_node.ns = "nodes";
        clear_node.id = node_id_counter_;
        clear_node.action = visualization_msgs::Marker::DELETE;
        node_id_counter_--;
        pub_markers.publish(clear_node);
    }

    void clearMarkers() {
        // Clear nodes
        visualization_msgs::Marker clear_nodes;
        clear_nodes.header.stamp = ros::Time::now();
        clear_nodes.ns = "nodes";
        clear_nodes.action = visualization_msgs::Marker::DELETEALL;
        pub_markers.publish(clear_nodes);

        // Clear edges
        visualization_msgs::Marker clear_edges;
        clear_edges.header.stamp = ros::Time::now();
        clear_edges.ns = "tree_branches";
        clear_edges.action = visualization_msgs::Marker::DELETEALL;
        pub_markers.publish(clear_edges);

        // Clear path
        visualization_msgs::Marker clear_path;
        clear_path.header.stamp = ros::Time::now();
        clear_path.ns = "path";
        clear_path.action = visualization_msgs::Marker::DELETEALL;
        pub_markers.publish(clear_path);

        // Reset marker ID counters
        node_id_counter_ = 0;
        edge_id_counter_ = 0;
        path_id_counter_ = 0;
    }

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
    bool get_T_C_B;
    
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
    double center_x;
    double center_y;
    double center_z;
    double dimensions_x;
    double dimensions_y;
    Eigen::Vector3d goal;
    double best_gain_;

    // Tree Parameters
    int num_nodes;
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
    std::atomic<int> replanning_counter_ = 0;

    // Bounds Parameters
    // Bounds on the size of the map.
    Eigen::Vector3d lower_bound_;
    Eigen::Vector3d upper_bound_;

    // Tree variables
    std::vector<rrt_star::Node*> tree;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector3d> prev_best_branch;
    std::vector<Eigen::Vector3d> best_branch;
    rrt_star::Node* next_best_node;
    eth_mav_msgs::EigenTrajectoryPoint trajectory_point;

    // UAV variables
    bool is_initialized = false;
    Eigen::Vector3d pose;
    mrs_msgs::UavState uav_state;
    mrs_msgs::ControlManagerDiagnostics control_manager_diag;
    //mrs_msgs::TrackerCommand tracker_cmd;

    // State variables
    std::atomic<State_t> state_;
    std::atomic<bool>    interrupted_ = false;
    std::atomic<bool> ready_to_plan_  = false;

    // Visualization variables
    int node_id_counter_;
    int edge_id_counter_;
    int path_id_counter_;

    // Instances
    GainEvaluator gain_evaluator;

    // Subscribers
    mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sub_control_manager_diag;
    mrs_lib::SubscribeHandler<mrs_msgs::UavState> sub_uav_state;
    //mrs_lib::SubscribeHandler<mrs_msgs::TrackerCommand> sub_tracker_cmd;
    mrs_lib::SubscribeHandler<mrs_msgs::DynamicsConstraints> sub_constraints;

    // Publishers
    ros::Publisher pub_markers;

    // Service servers
    ros::ServiceServer ss_start;

    // Service clients
    mrs_lib::ServiceClientHandler<mrs_msgs::GetPathSrv> sc_trajectory_generation;
    mrs_lib::ServiceClientHandler<mrs_msgs::TrajectoryReferenceSrv> sc_trajectory_reference;

    // Timers
    ros::Timer timer_main;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Planner planner(nh, nh_private);
    ros::spin();
    return 0;
}
