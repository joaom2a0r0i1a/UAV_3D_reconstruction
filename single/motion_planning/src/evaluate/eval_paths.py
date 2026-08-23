#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from mrs_msgs.msg import UavState, Reference
from threading import Lock

class PathExtractor:
    def __init__(self, pose_topic, reference_topic, clock_topic):
        self.path = Path()
        self.path.header.frame_id = "uav1/world_origin"
        self.reference = Path()
        self.reference.header.frame_id = "uav1/world_origin"
        self.current_time = rospy.Time()

        self.path_pub = rospy.Publisher('/uav1/finished_path', Path, queue_size=10)
        self.reference_pub = rospy.Publisher('/uav1/reference_path', Path, queue_size=10)

        rospy.Subscriber(clock_topic, Clock, self.clock_callback)
        rospy.Subscriber(pose_topic, UavState, self.pose_callback)
        rospy.Subscriber(reference_topic, Reference, self.reference_callback)

    def clock_callback(self, msg):
        self.current_time = msg.clock

    def pose_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.current_time
        pose_stamped.header.frame_id = "uav1/world_origin"
        pose_stamped.pose = msg.pose

        self.path.poses.append(pose_stamped)

    def reference_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.current_time
        pose_stamped.header.frame_id = "uav1/world_origin"
        pose_stamped.pose.position = msg.position

        self.reference.poses.append(pose_stamped)

    def publish(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.path.header.stamp = rospy.Time.now()
            self.reference.header.stamp = rospy.Time.now()
            self.path_pub.publish(self.path)
            self.reference_pub.publish(self.reference)
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('path_publisher', anonymous=True)
        rospy.set_param('/use_sim_time', True)
        pose_topic = '/uav1/estimation_manager/uav_state'
        reference_topic = '/uav1/reference_out'
        clock_topic = '/clock'
        extractor = PathExtractor(pose_topic, reference_topic, clock_topic)
        extractor.publish()
    except rospy.ROSInterruptException:
        pass
