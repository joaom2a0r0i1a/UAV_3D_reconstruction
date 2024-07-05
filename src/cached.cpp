#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrs_msgs/UavState.h>

#include <cache_nodes/Node.h>
#include <cache_nodes/Query.h>
#include <cache_nodes/QueryResponse.h>
#include <cache_nodes/BestNode.h>
#include <cache_nodes/BestNodeResponse.h>
#include <cache_nodes/Reevaluate.h>

#include <mrs_lib/param_loader.h>

#include <voxblox/core/tsdf_map.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_ros/tsdf_server.h>

#include <minkindr_conversions/kindr_msg.h>

#include <rrt_kd.h>
#include <evaluator.h>
#include <RTree.h>

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <functional>
#include <geometry_msgs/Point.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<float, 3, bg::cs::cartesian> Point;
typedef std::pair<Point, cache_nodes::Node> RTreeValue;

/*struct HyperParam {
    double l;
    double sigma_f;
    double sigma_n;
};*/

class Cached {
public:
    Cached(ros::NodeHandle& nh, const ros::NodeHandle& nh_private) : nh_(nh), nh_private_(nh_private), voxblox_server_(nh_, nh_private_) {
        //ss_query = nh_private_.advertiseService("gp_query_in", &Cached::callbackQuery, this);
        //ss_reevaluate = nh_private_.advertiseService("reevaluate_in", &Cached::callbackReevaluate, this);
        ss_best_node = nh_private_.advertiseService("best_node_in", &Cached::callbackBestNode, this);
        //sc_reevaluate = nh_private_.serviceClient<cache_nodes::Reevaluate>("reevaluate_out");

         ns = "uav1";

        /* Parameter loading */
        mrs_lib::ParamLoader param_loader(nh_private_, "cached");

        // Frames, Coordinates and Dimensions
        param_loader.loadParam("frame_id", frame_id);
        param_loader.loadParam("body/frame_id", body_frame_id);

        param_loader.loadParam("visualize/mean", visualize_mean, false);
        param_loader.loadParam("visualize/sigma", visualize_sigma, false);
        param_loader.loadParam("visualize/pts", visualize_pts, false);
        param_loader.loadParam("visualize/resolution", resolution, 1);
        param_loader.loadParam("local_planning/g_zero", g_zero, 2.0);
        param_loader.loadParam("local_planning/yaw_samples", num_yaw_samples, 10);

        // Camera
        param_loader.loadParam("camera/h_fov", horizontal_fov);
        param_loader.loadParam("camera/width", resolution_x);
        param_loader.loadParam("camera/height", resolution_y);
        param_loader.loadParam("camera/min_distance", min_distance);
        param_loader.loadParam("camera/max_distance", max_distance);
        param_loader.loadParam("camera/frame_id", camera_frame_id);

        sub_gain = nh_private_.subscribe("tree_node_in", 10, &Cached::callbackGain, this);
        sub_uav_state = nh_private_.subscribe("uav_state_in", 10, &Cached::callbackUavState, this);
        //pub_marker = nh_private_.advertise<visualization_msgs::MarkerArray>("gain_markers", 10);
        //pub_mean = nh_private_.advertise<visualization_msgs::MarkerArray>("mean_markers", 10);
        //pub_sigma = nh_private_.advertise<visualization_msgs::MarkerArray>("sigma_markers", 10);

        tsdf_map_ = voxblox_server_.getTsdfMapPtr();
        esdf_map_ = voxblox_server_.getEsdfMapPtr();
        evaluator.setTsdfLayer(tsdf_map_->getTsdfLayerPtr());

        // Get vertical FoV and setup camera
        vertical_fov = evaluator.getVerticalFoV(horizontal_fov, resolution_x, resolution_y);
        evaluator.setCameraModelParametersFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
        GetTransformation();

        /*if (visualize_pts) {
            rviz_timer = nh_private_.createTimer(ros::Duration(1), &Cached::rvizCallback, this);
        }
        if (visualize_mean || visualize_sigma) {
            evaluate_timer = nh_private_.createTimer(ros::Duration(5), &Cached::evaluate, this);
        }*/

        ROS_WARN("Defaulting to (-25, -25, 0.5), (25,  25, 15)...");
        min = {-25., -20., 0.5};
        max = {25.,  15., 20.};

        ROS_WARN("Range max parameter not specified");
        ROS_WARN("Defaulting to 5 m...");
        range = 5.0;

        bbx_min[0] = min[0];
        bbx_min[1] = min[1];
        bbx_min[2] = min[2];
        bbx_max[0] = max[0];
        bbx_max[1] = max[1];
        bbx_max[2] = max[2];

        //hyperparam.l = 1;
        //hyperparam.sigma_f = 1;
        //hyperparam.sigma_n = 0.1;

        //rtree = std::make_unique<RTree<cache_nodes::Node, double, 3>>();

        reevaluate_timer = nh.createTimer(ros::Duration(4), &Cached::timerReevaluate, this);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // Voxblox Map Server
    voxblox::EsdfServer voxblox_server_;

    // Shortcut to Maps
    std::shared_ptr<voxblox::TsdfMap> tsdf_map_;
    std::shared_ptr<voxblox::EsdfMap> esdf_map_;

    std::string frame_id;
    std::string body_frame_id;
    std::string camera_frame_id;
    std::string ns;

    // Transformations
    geometry_msgs::TransformStamped T_C_B_message;
    voxblox::Transformation T_C_B;

    //ros::ServiceServer ss_query;
    ros::ServiceServer ss_best_node;
    //ros::ServiceServer ss_reevaluate;
    //ros::ServiceClient sc_reevaluate;
    ros::Subscriber sub_gain;
    ros::Subscriber sub_uav_state;
    //ros::Publisher pub_marker;
    //ros::Publisher pub_mean;
    //ros::Publisher pub_sigma;
    //ros::Timer rviz_timer;
    //ros::Timer evaluate_timer;
    ros::Timer reevaluate_timer;

    bool visualize_mean;
    bool visualize_sigma;
    bool visualize_pts;
    int resolution;
    double g_zero;
    double range;
    int num_yaw_samples;
    //HyperParam hyperparam;
    Evaluator evaluator;

    // Camera Parameters
    double horizontal_fov;
    double vertical_fov;
    int resolution_x;
    int resolution_y;
    double min_distance;
    double max_distance;

    double bbx_min[3];
    double bbx_max[3];
    //std::unique_ptr<RTree<cache_nodes::Node, double, 3>> rtree;
    bgi::rtree<RTreeValue, bgi::quadratic<16>> rtree;
    //int id;
    std::vector<double> min;
    std::vector<double> max;
    double x;
    double y;
    double z;

    void GetTransformation() {
        T_C_B_message.transform.translation.x = -0.173024;
        T_C_B_message.transform.translation.y = 0.011500;
        T_C_B_message.transform.translation.z = 0.059864;

        T_C_B_message.transform.rotation.x = 0.000000;
        T_C_B_message.transform.rotation.y = -0.087156;
        T_C_B_message.transform.rotation.z = 0.000000;
        T_C_B_message.transform.rotation.w = 0.996195;

        // Transform into matrix
        tf::transformMsgToKindr(T_C_B_message.transform, &T_C_B);
        evaluator.setCameraExtrinsics(T_C_B);
    }

    /*Eigen::MatrixXd sqexpkernel(const Eigen::MatrixXd& x1, const Eigen::MatrixXd& x2) {
        int n1 = x1.rows();
        int n2 = x2.rows();
        Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n1, n2);

        for (int i = 0; i < n2; ++i) {
            Eigen::VectorXd l = (x1.rowwise() - x2.row(i)).rowwise().norm();
            K.col(i) = hyperparam.sigma_f * hyperparam.sigma_f * (-0.5 * l.array().square() / (hyperparam.l * hyperparam.l)).exp();
        }

        return K;
    }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> gp(const Eigen::VectorXd& y, const Eigen::MatrixXd& x, const Eigen::MatrixXd& xstar) {
        if (y.size() == 0 || x.rows() == 0) {
            return {Eigen::VectorXd::Zero(0), Eigen::VectorXd::Zero(0)};
        }

        Eigen::MatrixXd K = sqexpkernel(x, x);
        Eigen::MatrixXd Kstar = sqexpkernel(x, xstar);
        Eigen::MatrixXd Kss = sqexpkernel(xstar, xstar);

        Eigen::MatrixXd L = (K + hyperparam.sigma_n * hyperparam.sigma_n * Eigen::MatrixXd::Identity(K.rows(), K.cols())).llt().matrixL();
        Eigen::VectorXd alpha = L.transpose().triangularView<Eigen::Upper>().solve(L.triangularView<Eigen::Lower>().solve(y));
        Eigen::VectorXd posterior_mean = Kstar.transpose() * alpha;

        Eigen::MatrixXd v = L.triangularView<Eigen::Lower>().solve(Kstar);
        Eigen::VectorXd posterior_variance = Kss.diagonal() - (v.transpose() * v).diagonal();

        return {posterior_mean, posterior_variance};
    }*/

    void callbackUavState(const mrs_msgs::UavState::ConstPtr& msg) {
        x = msg->pose.position.x;
        y = msg->pose.position.y;
        z = msg->pose.position.z;
    }

    /*bool callbackReevaluate(cache_nodes::Reevaluate::Request& req, cache_nodes::Reevaluate::Response& res) {
        ROS_DEBUG_STREAM("Reevaluation Start!");

        for (std::vector<geometry_msgs::Point>::iterator iter = req.point.begin(); iter != req.point.end(); ++iter) {
            Eigen::Vector4d pos(iter->x, iter->y, iter->z, 0);
            std::shared_ptr<rrt_star::Node> node = std::make_shared<rrt_star::Node>(pos);
            eth_mav_msgs::EigenTrajectoryPoint traj_point;
            evaluator.computeGainFromsampledYaw(node, num_yaw_samples, traj_point);
            res.gain.push_back(node->gain);
            res.yaw.push_back(node->point[3]);
        }

        ROS_DEBUG_STREAM("Reevaluation Finish!");
        return true;
    }*/

    void timerReevaluate(const ros::TimerEvent&) {
        ROS_INFO("Reevaluate Start");

        std::vector<RTreeValue> result_s;
        rtree.query(bgi::satisfies([this](RTreeValue const& v) {
            Point current_point(x, y, z);
            Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
            //bool within_range = bg::distance(current_point, node_point) <= range;
            //bool above_g_zero = v.second.gain > g_zero;
            return(v.second.gain > g_zero && bg::distance(current_point, node_point) <= range);
        }), std::back_inserter(result_s));

        //rtree.query(bgi::satisfies([](RTreeValue const& v) { return true; }), std::back_inserter(result_s));

        for (const auto& node : result_s) {
            Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
            eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
            trajectory_point_gain.position_W = pos;
            trajectory_point_gain.setFromYaw(node.second.yaw);
            std::pair<double, double> result = evaluator.computeGainAEP(trajectory_point_gain);

            ROS_INFO("[Cached]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
            ROS_INFO("[Cached]: Old Point gain: %f", node.second.gain);
            ROS_INFO("[Cached]: New Point gain: %f", result.first);

            cache_nodes::Node updated_node = node.second;
            updated_node.gain = result.first;
            updated_node.yaw = result.second;

            rtree.remove(node);
            rtree.insert(std::make_pair(node.first, updated_node));
        }

        size_t numNodes = rtree.size();
        ROS_INFO("[Cached]: Nodes in the RTree: %lu", numNodes);
        ROS_INFO("[Cached]: List Size: %lu", result_s.size());

        /*if (!set_variables) {
            GetTransformation();
            ROS_INFO("[AEPlanner]: T_C_B Translation: [%f, %f, %f]", T_C_B_message.transform.translation.x, T_C_B_message.transform.translation.y, T_C_B_message.transform.translation.z);
            ROS_INFO("[AEPlanner]: T_C_B Rotation: [%f, %f, %f, %f]", T_C_B_message.transform.rotation.x, T_C_B_message.transform.rotation.y, T_C_B_message.transform.rotation.z, T_C_B_message.transform.rotation.w);
            set_variables = true;
        }*/

        /*if (!x || !y || !z)
        {
            ROS_WARN("No position received yet...");
            ROS_WARN("Make sure that 'pose' has been correctly mapped and that it is being published");
            return;
        }

        //ROS_INFO("I AM AT THE START");

        std::vector<cache_nodes::Node> hits_full;
        hits_full.clear();
        rtree->Search(bbx_min, bbx_max, [this, &hits_full](cache_nodes::Node pNode) {
            hits_full.push_back(pNode);
            return true;
        });

        double bbx_min_range[3] = {x - range, y - range, z - range};
        double bbx_max_range[3] = {x + range, y + range, z + range};
        std::vector<cache_nodes::Node> hits;
        hits.clear();
        rtree->Search(bbx_min_range, bbx_max_range, [this, &hits](cache_nodes::Node pNode) {
            hits.push_back(pNode);
            return true;
        });

        //ROS_INFO("I AM AT THE BOUNDING BOX");

        std::vector<geometry_msgs::Point> reevaluate_position_list;
        std::vector<cache_nodes::Node> reevaluate_list;
        //reevaluate_position_list.clear();
        //reevaluate_list.clear();
        for (auto item : hits) {
            if (item.gain > g_zero) {
                reevaluate_position_list.push_back(item.position);
                reevaluate_list.push_back(item);
            } //else {
                //rtree->Remove(bbx_min, bbx_max, item);
            //}
        }

        //ROS_INFO("I AM AT THE REEVALUATE POSITIONS");
        
        std::vector<double> gain_vector;
        std::vector<double> yaw_vector;
        gain_vector.clear();
        yaw_vector.clear();
        for (std::vector<geometry_msgs::Point>::iterator iter = reevaluate_position_list.begin(); iter != reevaluate_position_list.end(); ++iter) {
            Eigen::Vector4d pos(iter->x, iter->y, iter->z, 0);
            std::shared_ptr<rrt_star::Node> node = std::make_shared<rrt_star::Node>(pos);
            eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
            trajectory_point_gain.position_W = node->point.head(3);
            trajectory_point_gain.setFromYaw(node->point[3]);
            std::pair<double, double> result = evaluator.computeGainAEP(trajectory_point_gain);
            node->gain = result.first;
            node->point[3] = result.second;
            //eth_mav_msgs::EigenTrajectoryPoint traj_point;
            //evaluator.computeGainFromsampledYaw(node, num_yaw_samples, traj_point);
            //std::cout << "Position Computation: " << node->point << std::endl;
            //std::cout << "Gain Computation: " << node->gain << std::endl;
            gain_vector.push_back(node->gain);
            yaw_vector.push_back(node->point[3]);
        }

        //ROS_INFO("I AM AT THE REEVALUATION");

        for (size_t i = 0; i < reevaluate_list.size(); ++i) {
            ROS_INFO("[Cached]: Old Point position: [%f, %f, %f]", reevaluate_list[i].position.x, reevaluate_list[i].position.y, reevaluate_list[i].position.z);
            ROS_INFO("[Cached]: Old Point gain: %f", reevaluate_list[i].gain);
            //std::cout << "Old Point position: " << reevaluate_list[i].position << std::endl;
            //std::cout << "Old Point gain: " << reevaluate_list[i].gain << std::endl;
            reevaluate_list[i].gain = gain_vector[i];
            reevaluate_list[i].yaw = yaw_vector[i];
            ROS_INFO("[Cached]: New Point position: [%f, %f, %f]", reevaluate_list[i].position.x, reevaluate_list[i].position.y, reevaluate_list[i].position.z);
            ROS_INFO("[Cached]: New Point gain: %f", reevaluate_list[i].gain);
            //std::cout << "New Point position: " << reevaluate_list[i].position << std::endl;
            //std::cout << "New Point gain: " << reevaluate_list[i].gain << std::endl;
            rtree->Remove(bbx_min, bbx_max, reevaluate_list[i]);
            if (reevaluate_list[i].gain > g_zero) {
                rtree->Insert(bbx_min, bbx_max, reevaluate_list[i]);
            }
            //rtree->Insert(bbx_min, bbx_max, reevaluate_list[i]);
        }

        //ROS_INFO("I AM AT THE END");

        /*cache_nodes::Reevaluate srv;
        srv.request.point = reevaluate_position_list;
        if (sc_reevaluate.call(srv)) {
            for (size_t i = 0; i < hits.size(); ++i) {
                hits[i].gain = srv.response.gain[i];
                hits[i].yaw = srv.response.yaw[i];
                rtree->Remove(bbx_min_range, bbx_max_range, hits[i]);
                rtree->Insert(bbx_min_range, bbx_max_range, hits[i]);
            }
        } else {
            ROS_ERROR("Calling reevaluate service failed");
        }*/

       /*ROS_INFO("[Cached]: Full BBX List Size: %lu", hits_full.size());
       ROS_INFO("[Cached]: Full List Size: %lu", hits.size());
       ROS_INFO("[Cached]: List Size: %lu", reevaluate_list.size());*/
       //std::cout << "List Size: " << reevaluate_list.size() << std::endl;

        ROS_INFO("Reevaluate Done");
    }

    void callbackGain(const cache_nodes::Node::ConstPtr& msg) {
        Point point(msg->position.x, msg->position.y, msg->position.z);
        rtree.insert(std::make_pair(point, *msg));
        //rtree->Insert(bbx_min, bbx_max, *msg);
        //std::cout << "Added Point gain: " << msg->gain << std::endl;
        //++id;
    }

    /*bool callbackQuery(cache_nodes::Query::Request& req, cache_nodes::Query::Response& res) {
        double bbx_min_gp[3] = {x - 1, y - 1, z - 1};
        double bbx_max_gp[3] = {x + 1, y + 1, z + 1};
        std::vector<cache_nodes::Node> hits;
        rtree->Search(bbx_min_gp, bbx_max_gp, [this, &hits](const cache_nodes::Node& pNode) {
            hits.push_back(pNode);
            return true;
        });

        Eigen::VectorXd y(hits.size());
        Eigen::MatrixXd x(hits.size(), 3);

        for (size_t i = 0; i < hits.size(); ++i) {
            y(i) = hits[i].gain;
            x(i, 0) = hits[i].position.x;
            x(i, 1) = hits[i].position.y;
            x(i, 2) = hits[i].position.z;
        }

        double yaw = 0;
        for (auto item : hits) {
            yaw = item.yaw;
            break;
        }

        if (y.size() == 0) {
            res.mu = 0;
            res.sigma = 1;
            return true;
        }

        Eigen::MatrixXd xstar(1, 3);
        xstar << req.point.x, req.point.y, req.point.z;

        auto [mean, sigma] = gp(y, x, xstar);

        res.mu = mean(0);
        res.sigma = sigma(0);
        res.yaw = yaw;

        return true;
    }*/

    bool callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
        std::vector<RTreeValue> result_n;
        //rtree.query(bgi::nearest(Point(req.pos.x, req.pos.y, req.pos.z), 5), std::back_inserter(result_n));

        // Retrieve all nodes in the RTree
        rtree.query(bgi::satisfies([](RTreeValue const& v) { return true; }), std::back_inserter(result_n));

        for (const auto& value : result_n) {
            //double gain = evaluator.computeGainAEP(value.second.gain); // Assuming computeGainAEP takes the current gain
            if (value.second.gain > req.threshold) {
                res.best_node.push_back(value.second.position);
                res.gain.push_back(value.second.gain);
            }
        }

        return true;

        /*std::vector<cache_nodes::Node> hits_best;
        rtree->Search(bbx_min, bbx_max, [this, &hits_best](cache_nodes::Node pNode) {
            hits_best.push_back(pNode);
            return true;
        });

        double best_gain = 0.0;
        for (const auto& item : hits_best) {
            if (item.gain > req.threshold) {
                res.best_node.push_back(item.position);
                res.gain.push_back(item.gain);
            }
            //if (item.gain > best_gain) {
            //    best_gain = item.gain;
            //}
        }

        //res.gain = best_gain;
        return true;*/
    }

    /*void evaluate(const ros::TimerEvent&) {
        std::vector<cache_nodes::Node> hits_eval;
        rtree->Search(bbx_min, bbx_max, [this, &hits_eval](const cache_nodes::Node& pNode) {
            hits_eval.push_back(pNode);
            return true;
        });

        Eigen::VectorXd y(hits_eval.size());
        Eigen::MatrixXd x(hits_eval.size(), 3);

        for (size_t i = 0; i < hits_eval.size(); ++i) {
            y(i) = hits_eval[i].gain;
            x(i, 0) = hits_eval[i].position.x;
            x(i, 1) = hits_eval[i].position.y;
            x(i, 2) = hits_eval[i].position.z;
        }

        std::vector<double> xt, yt, zt = {1};
        for (double i = min[0]; i <= max[0]; i += resolution) {
            xt.push_back(i);
        }
        for (double i = min[1]; i <= max[1]; i += resolution) {
            yt.push_back(i);
        }

        Eigen::MatrixXd xstar(xt.size() * yt.size() * zt.size(), 3);
        int idx = 0;
        for (auto xx : xt) {
            for (auto yy : yt) {
                for (auto zz : zt) {
                    xstar.row(idx++) << xx, yy, zz;
                }
            }
        }

        auto [mean, sigma] = gp(y, x, xstar);

        visualization_msgs::MarkerArray mean_markers;
        visualization_msgs::MarkerArray sigma_markers;
        for (int id = 0; id < xstar.rows(); ++id) {
            mean_markers.markers.push_back(npArrayToMarker(id, xstar.row(id), mean(id), std::max(1 - sigma(id), 0.0)));
            // sigma_markers.markers.push_back(npArrayToMarker(id, xstar.row(id), sigma(id) * 2));
        }

        pub_mean.publish(mean_markers);
        pub_sigma.publish(sigma_markers);
    }

    void rvizCallback(const ros::TimerEvent&) {
        visualization_msgs::MarkerArray markers;
        std::vector<cache_nodes::Node> hits_rviz;
        rtree->Search(bbx_min, bbx_max, [this, &hits_rviz](const cache_nodes::Node& pNode) {
            hits_rviz.push_back(pNode);
            return true;
        });

        for (auto item : hits_rviz) {
            markers.markers.push_back(nodeToMarker(id, item));
        }

        pub_marker.publish(markers);
    }

    visualization_msgs::Marker npArrayToMarker(int id, const Eigen::Vector3d& p, double v = 0, double a = 0) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = ns + "/" + frame_id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = id;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.scale.z = 0.1;
        marker.color.r = v / 72.0;
        marker.color.g = 0;
        marker.color.b = 0.5;
        marker.color.a = a;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = p(0);
        marker.pose.position.y = p(1);
        marker.pose.position.z = p(2);
        marker.lifetime = ros::Duration(10);

        return marker;
    }

    visualization_msgs::Marker nodeToMarker(int id, const cache_nodes::Node& node) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = ns + "/" + frame_id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.id = id;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.r = node.gain / 72.0;
        marker.color.g = 0.0;
        marker.color.b = 0.5;
        marker.color.a = 1.0;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = node.position.x;
        marker.pose.position.y = node.position.y;
        marker.pose.position.z = node.position.z;
        marker.lifetime = ros::Duration(1.2);

        return marker;
    }*/
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Cached cached(nh, nh_private);
    ros::spin();
    return 0;
}
