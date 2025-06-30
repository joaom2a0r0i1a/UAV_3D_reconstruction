#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "setpoint_velocity_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    ros::Rate rate(20.0); // 20 Hz publishing rate

    ROS_INFO("Velocity publisher node started. Publishing velocity commands...");

    while (ros::ok()) {
        geometry_msgs::TwistStamped velocity_msg;
        velocity_msg.header.stamp = ros::Time::now();

        // Set desired linear velocity (vx, vy, vz) in m/s
        velocity_msg.twist.linear.x = 0.0;  // Move forward
        velocity_msg.twist.linear.y = 1.0;  // No sideways movement
        velocity_msg.twist.linear.z = 0.0;  // Move upward

        // Set desired angular velocity (yaw rate ωz) in rad/s
        velocity_msg.twist.angular.z = 0.3; // Slow yaw rotation

        // Log the velocity being published
        ROS_INFO("Publishing Velocity -> Linear: [x: %.2f, y: %.2f, z: %.2f] | Angular: [yaw: %.2f]",
                 velocity_msg.twist.linear.x, velocity_msg.twist.linear.y, velocity_msg.twist.linear.z,
                 velocity_msg.twist.angular.z);

        velocity_pub.publish(velocity_msg);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_WARN("Velocity publisher node shutting down.");
    return 0;
}

