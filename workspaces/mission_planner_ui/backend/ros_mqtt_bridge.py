import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Joy, Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import threading
import json
import struct
import math
import time
import base64
import os
import paho.mqtt.client as mqtt
from cv_bridge import CvBridge
import cv2
from dotenv import load_dotenv

load_dotenv()

# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

# Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.hivemq.com")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
BASE_TOPIC = "robo_gen_labs/go2_robot_1"

class RosMqttBridge(Node):
    def __init__(self):
        super().__init__('ros_mqtt_bridge')
        
        # --- MQTT Setup ---
        self.mqtt_client = mqtt.Client(client_id="go2_robot_bridge", protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        try:
            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f"Connected to MQTT Broker at {MQTT_BROKER}:{MQTT_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT: {e}")

        # --- ROS 2 Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.create_subscription(PointCloud2, '/registered_scan_o3d/voxelized', self.pc_callback, 10)
        self.create_subscription(Marker, '/viz_path_topic', self.path_callback, 10)
        self.create_subscription(Odometry, '/state_estimation', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Publishers
        self.goal_pub = self.create_publisher(PointStamped, '/goal_point', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)

        # State Variables
        self.bridge = CvBridge()
        self.last_voxel_time = 0
        self.voxel_interval = 2.0  # Throttle voxel payload over MQTT
        self.last_tf_time = 0
        self.tf_interval = 0.1     # 10Hz TF updates
        self.last_video_time = 0
        self.video_interval = 0.1  # 10Hz Video updates
        self.vehicle_z = 0.0
        self.latest_odom = None

        # ROS Timer for TF Publishing
        self.create_timer(self.tf_interval, self.publish_tf_to_mqtt)

        self.get_logger().info("ROS-MQTT Bridge initialized.")

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("MQTT Connection Successful")
            # Subscribe to commands from the Cloud API
            self.mqtt_client.subscribe(f"{BASE_TOPIC}/commands/navigate")
        else:
            self.get_logger().error(f"MQTT Connection Failed with code {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            if msg.topic == f"{BASE_TOPIC}/commands/navigate":
                payload = json.loads(msg.payload.decode())
                self.get_logger().info(f"DEBUG: ros_mqtt_bridge.py received MQTT command from {msg.topic}: {payload}")
                x = payload.get('x', 0)
                y = payload.get('y', 0)
                z = payload.get('z', self.vehicle_z)
                self.publish_ros_goal(x, y, z)
        except Exception as e:
            self.get_logger().error(f"Error handling MQTT message: {e}")

    def publish_ros_goal(self, x, y, z):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "goalpoint_tool"
        joy_msg.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self.joy_pub.publish(joy_msg)

        goal_msg = PointStamped()
        goal_msg.header.stamp = joy_msg.header.stamp
        goal_msg.header.frame_id = "map"
        goal_msg.point.x = x
        goal_msg.point.y = y
        goal_msg.point.z = z
        self.goal_pub.publish(goal_msg)
        time.sleep(0.01)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Published ROS Goal: {x}, {y}, {z}")


    def pc_callback(self, msg):
        now = time.time()
        if now - self.last_voxel_time < self.voxel_interval:
            return
        self.last_voxel_time = now

        points = []
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for p in gen:
            points.extend([p[0], p[1], p[2]])
            
        payload = struct.pack(f'{len(points)}f', *points)
        # Publish binary data
        self.mqtt_client.publish(f"{BASE_TOPIC}/telemetry/points", payload=payload, qos=0)


    def path_callback(self, msg):
        if msg.type == Marker.LINE_STRIP or msg.type == Marker.LINE_LIST:
            p_list = [{'x': p.x, 'y': p.y, 'z': p.z} for p in msg.points]
            self.mqtt_client.publish(f"{BASE_TOPIC}/telemetry/path", payload=json.dumps({"path": p_list}), qos=0)


    def odom_callback(self, msg):
        self.vehicle_z = msg.pose.pose.position.z
        self.latest_odom = msg


    def image_callback(self, msg):
        now = time.time()
        if now - self.last_video_time < self.video_interval:
            return
        self.last_video_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (320, 240))
            _, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
            b64_img = base64.b64encode(buffer).decode('utf-8')
            self.mqtt_client.publish(f"{BASE_TOPIC}/telemetry/video", payload=b64_img, qos=0)
        except Exception as e:
            self.get_logger().error(f"Image compression error: {e}")

    def publish_tf_to_mqtt(self):
        target_frames = ["base_link", "livox_frame", "odom"]
        tfs_snapshot = {}
        for child in target_frames:
            try:
                t = self.tf_buffer.lookup_transform("camera_init", child, Time(), Duration(seconds=0.0))
                tfs_snapshot[child] = {
                    "translation": {"x": t.transform.translation.x, "y": t.transform.translation.y, "z": t.transform.translation.z},
                    "rotation": {"x": t.transform.rotation.x, "y": t.transform.rotation.y, "z": t.transform.rotation.z, "w": t.transform.rotation.w}
                }
            except:
                pass
        
        # Fallback: If base_link missing from TF tree, use Odometry
        if "base_link" not in tfs_snapshot and self.latest_odom:
             p = self.latest_odom.pose.pose.position
             q = self.latest_odom.pose.pose.orientation
             tfs_snapshot["base_link"] = {
                "translation": {"x": p.x, "y": p.y, "z": p.z},
                "rotation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w}
             }

        if tfs_snapshot:
            self.mqtt_client.publish(f"{BASE_TOPIC}/telemetry/tf", payload=json.dumps({"tfs": tfs_snapshot}), qos=0)


def main(args=None):
    rclpy.init(args=args)
    bridge = RosMqttBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.mqtt_client.loop_stop()
        bridge.mqtt_client.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
