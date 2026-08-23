#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
from struct import pack, unpack


class GazeboSensorModel:
    def __init__(self):
        rospy.init_node('gazebo_sensor_model', anonymous=True)

        # === Parameters ===
        self.bridge = CvBridge()
        self.model = rospy.get_param('~model_type', 'ground_truth')
        self.max_range = rospy.get_param('~maximum_distance', 0.0)  # 0 = no limit
        self.flatten_distance = rospy.get_param('~flatten_distance', 0.0)
        self.publish_inf_depth = rospy.get_param('~publish_inf_depth', False)
        out_topic   = rospy.get_param('~pointcloud_out', '~pointcloud')
        self.frame_id = rospy.get_param('~frame_id', 'camera')

        # Model dependent params
        if self.model == 'gaussian_depth_noise':
            # coefficients for polynomial, f(z) = k0 + k1z + k2z^2 + k3z^3
            self.coefficients = np.array([0.0]*8)
            for i in range(4):
                self.coefficients[i] = rospy.get_param('~k_mu_%i' % i, 0.0)
                self.coefficients[4 + i] = rospy.get_param('~k_sigma_%i' % i, 0.05)

        # Topics
        color_topic = rospy.get_param('~color_topic', '/camera/color/image_raw')
        depth_topic = rospy.get_param('~depth_topic', '/camera/depth/image_raw')
        info_topic = rospy.get_param('~info_topic', '/camera/camera_info')

        # Subscriptions
        self.sub_info = rospy.Subscriber(info_topic, CameraInfo, self.info_callback, queue_size=1)
        self.sub_depth = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1)
        self.sub_color = rospy.Subscriber(color_topic, Image, self.color_callback, queue_size=1)

        # Publishers
        self.pub_pc = rospy.Publisher(out_topic, PointCloud2, queue_size=1)
        if self.publish_inf_depth:
            self.pub_inf_depth = rospy.Publisher('~depth_inf', Image, queue_size=1)

        self.color_img = None
        self.camera_params = None  # [width, height, focal_length]

        rospy.loginfo("Gazebo sensor model initialized and running...")

    # === Callbacks ===
    def info_callback(self, msg):
        # Derive approximate focal length from intrinsics
        fx = msg.K[0]
        self.camera_params = [msg.width, msg.height, fx]

    def color_callback(self, msg):
        self.color_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def depth_callback(self, msg):
        if self.camera_params is None:
            rospy.logwarn_throttle(5, "Waiting for camera info...")
            return
        if self.color_img is None:
            rospy.logwarn_throttle(5, "Waiting for color image...")
            return

        # Convert depth to meters
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        if msg.encoding == "16UC1":
            depth = depth.astype(np.float32) / 1000.0
        

        # Replace zeros or NaNs with infinity (no hit)
        depth[np.isnan(depth)] = np.inf
        depth[depth <= 0.0] = np.inf

        if self.flatten_distance > 0:
            depth = np.clip(depth, 0, self.flatten_distance)

        # Project to 3D
        x, y, z = self.depth_to_3d(depth)

        # Convert color image to packed float
        rgb = self.rgb_to_float(self.color_img)

        if self.model == 'gaussian_depth_noise':
            z = self.process_gaussian_depth_noise(z)

        # Publish PointCloud2
        msg_out = PointCloud2()
        msg_out.header.stamp = msg.header.stamp
        msg_out.header.frame_id = self.frame_id
        msg_out.height = depth.shape[0]
        msg_out.width = depth.shape[1]
        msg_out.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1)
        ]
        msg_out.is_bigendian = False
        #msg_out.point_step = 12
        msg_out.point_step = 16
        msg_out.row_step = msg_out.point_step * msg_out.width
        msg_out.is_dense = True

        #cloud = np.zeros((msg_out.height, msg_out.width, 3), dtype=np.float32)
        cloud = np.zeros((msg_out.height, msg_out.width, 4), dtype=np.float32)
        cloud[:, :, 0] = x.reshape(depth.shape)
        cloud[:, :, 1] = y.reshape(depth.shape)
        cloud[:, :, 2] = z.reshape(depth.shape)
        cloud[:, :, 3] = rgb.reshape(depth.shape)
        msg_out.data = np.float32(cloud).tobytes()

        self.pub_pc.publish(msg_out)

    # === Core computation ===
    def depth_to_3d(self, depth):
        width, height, f = self.camera_params
        cx = width / 2.0
        cy = height / 2.0

        u, v = np.meshgrid(np.arange(width), np.arange(height))
        z = depth.flatten()  # X forward
        x = z
        y = -(u.flatten() - cx) * z / f  # Y left
        z_coord = -(v.flatten() - cy) * z / f  # Z up

        return x, y, z_coord



    @staticmethod
    def rgb_to_float(img):
        """Convert RGB uint8 image to packed float format for PointCloud2."""
        r = np.ravel(img[:, :, 0]).astype(np.uint32)
        g = np.ravel(img[:, :, 1]).astype(np.uint32)
        b = np.ravel(img[:, :, 2]).astype(np.uint32)
        rgb = (r << 16) | (g << 8) | b
        return np.array(unpack('%df' % len(rgb), pack('%dI' % len(rgb), *rgb)))

    def process_gaussian_depth_noise(self, z_in):
        # Add a depth dependent guassian error term to the perceived depth. Mean and stddev can be specified as up to
        # deg3 polynomials.
        mu = np.ones(np.shape(z_in)) * self.coefficients[0]
        sigma = np.ones(np.shape(z_in)) * self.coefficients[4]
        for i in range(1, 4):
            if self.coefficients[i] != 0:
                mu = np.abs(mu + np.power(z_in, i) * self.coefficients[i])
            if self.coefficients[4 + i] != 0:
                sigma = np.abs(sigma + np.power(z_in, i) * self.coefficients[4 + i])
        return z_in + np.random.normal(mu, sigma)


if __name__ == '__main__':
    GazeboSensorModel()
    rospy.spin()
