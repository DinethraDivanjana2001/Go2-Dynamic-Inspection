#!/usr/bin/env python3
"""
camera_node.py — ROS2 Node: Publish Insta360 + Logitech camera feeds as topics.

Based on working kalibr_ws implementations (webcam_publisher + insta360_publisher).
Uses direct device path opens — no frame-verification that was causing false negatives.

Topics published:
  /visual_inspection/insta360/image_raw  (sensor_msgs/Image)
  /visual_inspection/logitech/image_raw  (sensor_msgs/Image)

Test:
  ros2 topic list | grep visual_inspection
  ros2 topic hz /visual_inspection/insta360/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os


class CameraPublisherNode(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        self.bridge = CvBridge()

        # --- Parameters (can override from launch file or CLI) ---
        self.declare_parameter('insta_device', '/dev/insta360')
        self.declare_parameter('logi_device',  '/dev/logitech')
        self.declare_parameter('fps', 30)

        insta_dev = self.get_parameter('insta_device').value
        logi_dev  = self.get_parameter('logi_device').value
        fps       = self.get_parameter('fps').value

        self.get_logger().info(f'Opening Insta360 at: {insta_dev}')
        self.get_logger().info(f'Opening Logitech  at: {logi_dev}')

        # --- Open cameras directly by path (same as working ibvs_pipeline.py) ---
        self.cap_insta = None
        self.cap_logi  = None

        # Insta360
        if os.path.exists(insta_dev):
            self.cap_insta = cv2.VideoCapture(insta_dev)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_insta.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
            self.cap_insta.set(cv2.CAP_PROP_FPS, fps)
            self.cap_insta.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.cap_insta.isOpened():
                self.get_logger().info('✅ Insta360 opened')
            else:
                self.get_logger().warn('⚠️  Insta360 device found but failed to open')
                self.cap_insta = None
        else:
            self.get_logger().warn(f'⚠️  Insta360 not found at {insta_dev} — is it plugged in?')

        # Logitech
        if os.path.exists(logi_dev):
            self.cap_logi = cv2.VideoCapture(logi_dev)
            self.cap_logi.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
            self.cap_logi.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
            self.cap_logi.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap_logi.set(cv2.CAP_PROP_FPS, fps)
            self.cap_logi.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.cap_logi.isOpened():
                self.get_logger().info('✅ Logitech opened')
            else:
                self.get_logger().warn('⚠️  Logitech device found but failed to open')
                self.cap_logi = None
        else:
            self.get_logger().warn(f'⚠️  Logitech not found at {logi_dev} — is it plugged in?')

        # --- Publishers ---
        self.pub_insta = self.create_publisher(
            Image, '/visual_inspection/insta360/image_raw', 10)
        self.pub_logi = self.create_publisher(
            Image, '/visual_inspection/logitech/image_raw', 10)

        # --- Timer ---
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f'📷 Camera publisher running at {fps} Hz')

    def timer_callback(self):
        # Insta360
        if self.cap_insta is not None and self.cap_insta.isOpened():
            ret, frame = self.cap_insta.read()
            if ret and frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'insta360_frame'
                self.pub_insta.publish(msg)

        # Logitech
        if self.cap_logi is not None and self.cap_logi.isOpened():
            ret, frame = self.cap_logi.read()
            if ret and frame is not None:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'logitech_frame'
                self.pub_logi.publish(msg)

    def destroy_node(self):
        if self.cap_insta:
            self.cap_insta.release()
        if self.cap_logi:
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
