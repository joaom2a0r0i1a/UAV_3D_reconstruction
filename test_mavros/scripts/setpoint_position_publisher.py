#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tf_trans

def publish_setpoint_position():
    rospy.init_node('setpoint_position_publisher', anonymous=True)
    position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Publishing position and yaw setpoints to /mavros/setpoint_position/local")

    while not rospy.is_shutdown():
        position_msg = PoseStamped()
        position_msg.header.stamp = rospy.Time.now()

        # Set position (meters)
        position_msg.pose.position.x = 2.0
        position_msg.pose.position.y = 5.0
        position_msg.pose.position.z = 3.5

        # Set yaw (convert from Euler to Quaternion)
        yaw = 1.57  # 90 degrees in radians
        quaternion = tf_trans.quaternion_from_euler(0, 0, yaw)
        position_msg.pose.orientation.x = quaternion[0]
        position_msg.pose.orientation.y = quaternion[1]
        position_msg.pose.orientation.z = quaternion[2]
        position_msg.pose.orientation.w = quaternion[3]

        position_pub.publish(position_msg)
        rospy.loginfo(f"Publishing -> Position: ({position_msg.pose.position.x}, {position_msg.pose.position.y}, {position_msg.pose.position.z}), "
                      f"Yaw: {yaw:.2f} rad")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoint_position()
    except rospy.ROSInterruptException:
        rospy.logwarn("Setpoint Position publisher interrupted.")

