#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_position_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    ros::Rate rate(20.0); // Publishing rate of 20 Hz
    
    ROS_INFO("Setpoint Position publisher node started. Publishing position commands...");

    while (ros::ok()) {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "map";  // Adjust if necessary

        // Set desired position (x, y, z) in meters
        pose_msg.pose.position.x = 2.0;
        pose_msg.pose.position.y = 5.0;
        pose_msg.pose.position.z = 3.5;

        // Set desired yaw (converted to quaternion)
        double yaw = 1.57; // 90 degrees in radians
        pose_msg.pose.orientation.w = cos(yaw / 2);
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = sin(yaw / 2);

	ROS_INFO("Publishing Position: [x: %.2f, y: %.2f, z: %.2f] | Yaw: [yaw: %.2f]",
                 pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z,
                 yaw);
                 
        setpoint_pub.publish(pose_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
