#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def velocity_publisher():
    rospy.init_node('setpoint_velocity_publisher', anonymous=True)
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("Velocity publisher node started. Publishing velocity commands...")

    while not rospy.is_shutdown():
        velocity_msg = Twist()

        # Set desired linear velocity (vx, vy, vz) in m/s
        velocity_msg.linear.x = 0.0  # Move forward
        velocity_msg.linear.y = 1.0  # No sideways movement
        velocity_msg.linear.z = 0.0  # Move upward

        # Set desired angular velocity (yaw rate ωz) in rad/s
        velocity_msg.angular.z = 0.3  # Slow yaw rotation

        # Log the velocity being published
        rospy.loginfo(f"Publishing Velocity -> Linear: [x: {velocity_msg.linear.x}, y: {velocity_msg.linear.y}, z: {velocity_msg.linear.z}] | Angular: [yaw: {velocity_msg.angular.z}]")

        velocity_pub.publish(velocity_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        velocity_publisher()
    except rospy.ROSInterruptException:
        rospy.logwarn("Velocity publisher node interrupted.")

