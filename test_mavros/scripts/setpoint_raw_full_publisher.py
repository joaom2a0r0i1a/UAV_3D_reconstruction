#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget

def publish_setpoint_raw():
    rospy.init_node('setpoint_raw_full_publisher', anonymous=True)
    raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Publishing full control setpoints to /mavros/setpoint_raw/local")

    while not rospy.is_shutdown():
        raw_msg = PositionTarget()
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask: We do NOT ignore anything (everything is set)
        raw_msg.type_mask = PositionTarget.IGNORE_YAW_RATE  # We are setting yaw directly, not yaw rate

        # Set position (meters)
        raw_msg.position.x = 0.0
        raw_msg.position.y = 5.0
        raw_msg.position.z = 6.0

        # Set velocity (m/s)
        raw_msg.velocity.x = 0.0
        raw_msg.velocity.y = 1.0
        raw_msg.velocity.z = 0.0

        # Set acceleration (m/s²)
        raw_msg.acceleration_or_force.x = 0.2
        raw_msg.acceleration_or_force.y = 0.1
        raw_msg.acceleration_or_force.z = 0.0

        # Set yaw (radians)
        raw_msg.yaw = 1.57  # 90 degrees

        raw_pub.publish(raw_msg)
        rospy.loginfo(f"Publishing -> Position: ({raw_msg.position.x}, {raw_msg.position.y}, {raw_msg.position.z}), "
                      f"Velocity: ({raw_msg.velocity.x}, {raw_msg.velocity.y}, {raw_msg.velocity.z}), "
                      f"Acceleration: ({raw_msg.acceleration_or_force.x}, {raw_msg.acceleration_or_force.y}, {raw_msg.acceleration_or_force.z}), "
                      f"Yaw: {raw_msg.yaw:.2f} rad")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoint_raw()
    except rospy.ROSInterruptException:
        rospy.logwarn("Setpoint Raw Full publisher interrupted.")

