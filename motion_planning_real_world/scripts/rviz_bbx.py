#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def create_bounding_box_marker():
    marker = Marker()
    marker.header.frame_id = "map"  # Use your reference frame
    marker.header.stamp = rospy.Time.now()
    marker.ns = "bounding_box"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    # Center of the box: z is halfway between min (0) and max (5) → 2.5
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 2.5

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # Dimensions
    marker.scale.x = 8.0 # From -4 to 4
    marker.scale.y = 8.0
    marker.scale.z = 5.0  # From 0 to 5

    # Color (RGBA)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.3  # Transparent green

    marker.lifetime = rospy.Duration(0)  # Persistent

    return marker

def main():
    rospy.init_node('bounding_box_marker_node')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
    rate = rospy.Rate(1)  # Publish at 1 Hz

    while not rospy.is_shutdown():
        marker = create_bounding_box_marker()
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    main()
