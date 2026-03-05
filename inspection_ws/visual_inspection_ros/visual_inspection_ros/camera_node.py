#!/usr/bin/env python3
"""
camera_node.py — ROS2 Node: Publish Insta360 + Logitech camera feeds as topics.

Topics published:
  /visual_inspection/insta360/image_raw  (sensor_msgs/Image)
  /visual_inspection/logitech/image_raw  (sensor_msgs/Image)

Test:
  ros2 topic list
  ros2 topic hz /visual_inspection/insta360/image_raw
  ros2 run rqt_image_view rqt_image_view
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os
import glob as glob_module


# ── Device Detection (same as ibvs_pipeline.py) ──────────────────────────────

def find_camera_index(udev_path):
    """Open camera by udev symlink path, return OpenCV index."""
    if os.path.exists(udev_path):
        cap = cv2.VideoCapture(udev_path)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                real = os.path.realpath(udev_path)
                try:
                    return int(real.replace('/dev/video', ''))
                except ValueError:
                    pass
    return -1


# ── ROS2 Node ────────────────────────────────────────────────────────────────

class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # --- Detect cameras ---
        self.get_logger().info('🔍 Detecting cameras...')

        insta_idx = find_camera_index('/dev/insta360')
        logi_idx  = find_camera_index('/dev/logitech')

        self.cap_insta = None
        self.cap_logi  = None

        if insta_idx < 0:
            self.get_logger().warn('⚠️  Insta360 not found at /dev/insta360 — check udev rules and camera connection')
        else:
            self.get_logger().info(f'✅ Insta360: /dev/video{insta_idx}')
            self.cap_insta = cv2.VideoCapture(insta_idx)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
            self.cap_insta.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if logi_idx < 0:
            self.get_logger().warn('⚠️  Logitech not found at /dev/logitech — check udev rules and camera connection')
        else:
            self.get_logger().info(f'✅ Logitech:  /dev/video{logi_idx}')
            self.cap_logi = cv2.VideoCapture(logi_idx)
            self.cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap_logi.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- Publishers ---
        self.pub_insta = self.create_publisher(
            Image, '/visual_inspection/insta360/image_raw', 10)
        self.pub_logi = self.create_publisher(
            Image, '/visual_inspection/logitech/image_raw', 10)

        # --- Timer: publish at ~30 Hz ---
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info('📷 Camera publisher started at 30 Hz')

    def timer_callback(self):
        # Insta360
        if self.cap_insta is not None:
            ret_i, frame_i = self.cap_insta.read()
            if ret_i and frame_i is not None:
                msg = self.bridge.cv2_to_imgmsg(frame_i, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'insta360_frame'
                self.pub_insta.publish(msg)

        # Logitech
        if self.cap_logi is not None:
            ret_l, frame_l = self.cap_logi.read()
            if ret_l and frame_l is not None:
                msg = self.bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'logitech_frame'
                self.pub_logi.publish(msg)

    def destroy_node(self):
        self.cap_insta.release()
        self.cap_logi.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
