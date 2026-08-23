#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_bounding_box():
    rospy.init_node('bbx_publisher')

    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(10)

    # Define the bounding box parameters
    frame_id = "map"  # Change this if you want to attach it to another TF frame
    box_center = [4.5, 0.0, 3.0]  # [x, y, z]
    box_size = [5.0, 4.0, 6.0]    # [length, width, height]

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "bounding_box"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Set pose
    marker.pose.position.x = box_center[0]
    marker.pose.position.y = box_center[1]
    marker.pose.position.z = box_center[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Set size
    marker.scale.x = box_size[0]
    marker.scale.y = box_size[1]
    marker.scale.z = box_size[2]

    # Set color (RGBA)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5  # Transparency

    marker.lifetime = rospy.Duration()  # 0 means forever

    rospy.loginfo("Publishing bounding box marker in RViz...")

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        create_bounding_box()
    except rospy.ROSInterruptException:
        pass
