#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RealSenseImageDecompressor:
    def __init__(self):
        rospy.init_node('realsense_image_decompressor', anonymous=True)
        self.bridge = CvBridge()

        # Publishers for decompressed images
        self.color_pub = rospy.Publisher(
            "/camera/color/image_raw_decompressed", Image, queue_size=1)
        self.depth_pub = rospy.Publisher(
            "/camera/aligned_depth_to_color/image_raw_decompressed", Image, queue_size=1)

        # Subscribers for compressed topics
        rospy.Subscriber("/camera/color/image_raw/compressed",
                         CompressedImage, self.color_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw/compressed",
                         CompressedImage, self.depth_callback)

        rospy.loginfo("Decompressor node initialized. Listening to RealSense compressed topics.")

    def color_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            ros_img.header = msg.header
            self.color_pub.publish(ros_img)
        except Exception as e:
            rospy.logerr(f"Failed to decompress color image: {e}")

    def depth_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # Expecting 16UC1
            ros_img = self.bridge.cv2_to_imgmsg(cv_img, encoding="8UC1")
            ros_img.header = msg.header
            self.depth_pub.publish(ros_img)
        except Exception as e:
            rospy.logerr(f"Failed to decompress depth image: {e}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = RealSenseImageDecompressor()
        node.run()
    except rospy.ROSInterruptException:
        pass

