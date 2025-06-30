#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import PositionTarget

def publish_setpoint_raw():
    rospy.init_node('setpoint_raw_position_publisher', anonymous=True)
    raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    rospy.loginfo("Publishing position setpoints to /mavros/setpoint_raw/local")

    while not rospy.is_shutdown():
        raw_msg = PositionTarget()
        raw_msg.header.stamp = rospy.Time.now()
        raw_msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Type mask to ignore everything except position
        raw_msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |  # Ignore velocity
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |  # Ignore acceleration
            PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE  # Ignore yaw and yaw rate
        )

        # Set position (meters)
        raw_msg.position.x = 2.0
        raw_msg.position.y = 5.0
        raw_msg.position.z = 3.5

        raw_pub.publish(raw_msg)
        rospy.loginfo(f"Publishing -> Position: ({raw_msg.position.x}, {raw_msg.position.y}, {raw_msg.position.z})")

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_setpoint_raw()
    except rospy.ROSInterruptException:
        rospy.logwarn("Setpoint Raw Position publisher interrupted.")

