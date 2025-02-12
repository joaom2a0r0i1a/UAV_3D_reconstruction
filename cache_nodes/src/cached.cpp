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
        range = 10.0;

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

        reevaluate_timer = nh.createTimer(ros::Duration(3), &Cached::timerReevaluate, this);
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

    void callbackUavState(const mrs_msgs::UavState::ConstPtr& msg) {
        x = msg->pose.position.x;
        y = msg->pose.position.y;
        z = msg->pose.position.z;
    }

    void timerReevaluate(const ros::TimerEvent&) {
        ROS_INFO("Reevaluate Start");

        std::vector<RTreeValue> result_s;
        rtree.query(bgi::satisfies([this](RTreeValue const& v) {
            Point current_point(x, y, z);
            Point node_point(v.second.position.x, v.second.position.y, v.second.position.z);
            //bool within_range = bg::distance(current_point, node_point) <= range;
            //bool above_g_zero = v.second.gain > g_zero;
            return(bg::distance(current_point, node_point) <= range);
        }), std::back_inserter(result_s));

        //rtree.query(bgi::satisfies([](RTreeValue const& v) { return true; }), std::back_inserter(result_s));

        size_t numNodesBefore = rtree.size();

        for (const auto& node : result_s) {
            Eigen::Vector3d pos(node.second.position.x, node.second.position.y, node.second.position.z);
            eth_mav_msgs::EigenTrajectoryPoint trajectory_point_gain;
            trajectory_point_gain.position_W = pos;
            trajectory_point_gain.setFromYaw(node.second.yaw);
            std::pair<double, double> result = evaluator.computeGainOptimizedAEP(trajectory_point_gain);

            ROS_INFO("[Cached]: Point position: [%f, %f, %f]", node.second.position.x, node.second.position.y, node.second.position.z);
            ROS_INFO("[Cached]: Old Point gain: %f", node.second.gain);
            ROS_INFO("[Cached]: New Point gain: %f", result.first);

            cache_nodes::Node updated_node = node.second;
            updated_node.gain = result.first;
            updated_node.yaw = result.second;

            rtree.remove(node);
            if (updated_node.gain > g_zero) {
                rtree.insert(std::make_pair(node.first, updated_node));
            }
        }

        size_t numNodesAfter = rtree.size();
        
        ROS_INFO("[Cached]: Nodes in the RTree Before: %lu", numNodesBefore);
        ROS_INFO("[Cached]: Search List Size: %lu", result_s.size());
        ROS_INFO("[Cached]: Nodes in the RTree After: %lu", numNodesAfter);

        ROS_INFO("Reevaluate Done");
    }

    void callbackGain(const cache_nodes::Node::ConstPtr& msg) {
        Point point(msg->position.x, msg->position.y, msg->position.z);
        rtree.insert(std::make_pair(point, *msg));
        //std::cout << "Added Point gain: " << msg->gain << std::endl;
        //++id;
    }

    bool callbackBestNode(cache_nodes::BestNode::Request& req, cache_nodes::BestNode::Response& res) {
        std::vector<RTreeValue> result_n;
        //rtree.query(bgi::nearest(Point(req.pos.x, req.pos.y, req.pos.z), 5), std::back_inserter(result_n));

        // Retrieve all nodes in the RTree
        //rtree.query(bgi::satisfies([](RTreeValue const& v) { return true; }), std::back_inserter(result_n));

        rtree.query(bgi::satisfies([this](RTreeValue const& v) {
            bool above_g_zero = v.second.gain > g_zero;
            return(above_g_zero);
        }), std::back_inserter(result_n));

        for (const auto& value : result_n) {
            //double gain = evaluator.computeGainOptimizedAEP(value.second.gain); // Assuming computeGainOptimizedAEP takes the current gain
            if (value.second.gain > req.threshold) {
                res.best_node.push_back(value.second.position);
                res.gain.push_back(value.second.gain);
            }
        }

        return true;
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cached");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    Cached cached(nh, nh_private);
    ros::spin();
    return 0;
}
