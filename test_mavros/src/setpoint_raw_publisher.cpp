#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_raw_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher setpoint_pub = nh.advertise<mavros_msgs::PositionTarget>(
        "/mavros/setpoint_raw/local", 10);

    ros::Rate rate(20.0); // Publishing rate of 20 Hz
    
    ROS_INFO("Setpoint Raw publisher node started. Publishing commands...");

    while (ros::ok()) {
        mavros_msgs::PositionTarget target_msg;
        target_msg.header.stamp = ros::Time::now();
        target_msg.header.frame_id = "map";

        target_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        // Enable only position control (ignore velocity and acceleration)
        target_msg.type_mask =
            mavros_msgs::PositionTarget::IGNORE_VX |
            mavros_msgs::PositionTarget::IGNORE_VY |
            mavros_msgs::PositionTarget::IGNORE_VZ |
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        // Set desired position (x, y, z) in meters
        target_msg.position.x = 2.0;
        target_msg.position.y = 5.0;
        target_msg.position.z = 3.5;

        // Set desired yaw in radians
        target_msg.yaw = 1.57; // 0 degrees
        
        ROS_INFO("Publishing Position: [x: %.2f, y: %.2f, z: %.2f] | Yaw: [yaw: %.2f]",
                 target_msg.position.x, target_msg.position.y, target_msg.position.z,
                 target_msg.yaw);

        setpoint_pub.publish(target_msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

