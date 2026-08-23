#!/usr/bin/env python

import rospy
from mrs_msgs.msg import UavState
from std_msgs.msg import Float32
import numpy as np

class VelocityMagnitude:
    def __init__(self):
        rospy.init_node('rqt_plot_velocity', anonymous=True)
        self.state_topic = '/uav1/estimation_manager/uav_state'
        self.pub_velocity_magnitude = rospy.Publisher('/uav1/velocity_magnitude', Float32, queue_size=10)
        self.sub_velocity = rospy.Subscriber(self.state_topic, UavState, self.uav_state_callback)
        rospy.spin()

    def uav_state_callback(self, msg):
        velocity_x = msg.velocity.linear.x
        velocity_y = msg.velocity.linear.y
        velocity_z = msg.velocity.linear.z

        velocity_magnitude = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
        self.pub_velocity_magnitude.publish(velocity_magnitude)

if __name__ == "__main__":
    try:
        velocity_magnitude_publisher = VelocityMagnitude()
    except rospy.ROSInterruptException:
        pass