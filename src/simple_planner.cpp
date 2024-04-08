#include <ros/ros.h>
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Reference.h>
#include <mrs_msgs/Vec1.h>

class Goto {
public:
    Goto() {
        ros::NodeHandle nh;

        // Initialize service clients
        sc_path = nh.serviceClient<mrs_msgs::PathSrv>("/uav1/trajectory_generation/path");

        // Advertise the service
        service = nh.advertiseService("/uav1/planner/start", &Goto::callbackStart, this);
    }

    bool callbackStart(mrs_msgs::Vec1::Request& req, mrs_msgs::Vec1::Response& res) {
        mrs_msgs::PathSrv path_msg;
        path_msg.request.path.header.frame_id = "";
        path_msg.request.path.header.stamp = ros::Time::now();
        path_msg.request.path.use_heading = true;
        path_msg.request.path.fly_now = true;
        path_msg.request.path.max_execution_time = 5.0;
        path_msg.request.path.max_deviation_from_path = 0.0;
        path_msg.request.path.dont_prepend_current_state = false;

        double sign = 1.0;
        for (int i = 0; i < 10; ++i) {
            mrs_msgs::Reference point;
            point.position.x = i * 2;
            point.position.y = sign * 0.5;
            point.position.z = 5;
            point.heading = 0;
            sign *= -1;
            path_msg.request.path.points.push_back(point);
        }

        bool success = sc_path.call(path_msg);

        if (!success) {
            ROS_ERROR("[planner]: service call for trajectory failed");
            res.success = false;
            res.message = "Failed to call path service";
            return false;
        } else {
            if (!path_msg.response.success) {
                ROS_ERROR("[planner]: service call for trajectory failed: '%s'", path_msg.response.message.c_str());
                res.success = false;
                res.message = "Path service returned failure";
                return false;
            }
        }

        res.success = true;
        res.message = "starting";
        return true;
    }

private:
    ros::ServiceClient sc_path;
    ros::ServiceServer service;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "planner");
    Goto goto_instance;
    ros::spin();
    return 0;
}
