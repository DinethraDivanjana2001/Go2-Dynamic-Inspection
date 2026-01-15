import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Joy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import threading
import asyncio
import json
import struct
import math
import os
import time

# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

# Initialize FastAPI
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global buffers
latest_points = []
latest_path_points = [] 
vehicle_z = 0.0
lock = threading.Lock()
ros_node = None
WAYPOINTS_FILE = "waypoints.json"

# Models
class GoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0

class Waypoint(BaseModel):
    name: str
    x: float
    y: float
    z: float

class ROSNode(Node):
    def __init__(self):
        super().__init__('web_viz_backend')
        self.subscription_pc = self.create_subscription(
            PointCloud2,
            '/registered_scan_o3d/voxelized',
            self.pc_callback,
            10
        )
        self.subscription_marker = self.create_subscription(
            Marker,
            '/viz_path_topic',
            self.marker_callback,
            10
        )
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odom_callback,
            10
        )
        
        # Publishers for Goal Interaction
        self.pub_goal = self.create_publisher(PointStamped, '/goal_point', 5)
        self.pub_joy = self.create_publisher(Joy, '/joy', 5)
        
        # TF Buffer
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

        # State
        self.latest_points = None
        self.latest_path = []
        self.vehicle_z = 0.0
        self.latest_frame = None # JPEG bytes
        self.bridge = CvBridge()
        
        self.last_voxel_time = 0
        self.voxel_interval = 2.0 # 2 Seconds Update Rate

        self.get_logger().info("ROS Node Started. Listening to /registered_scan_o3d/voxelized, /camera/image_raw etc.")

    def pc_callback(self, msg):
        # Throttle Voxel Updates
        now = time.time()
        if now - self.last_voxel_time < self.voxel_interval:
            return
        self.last_voxel_time = now

        points = []
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for p in gen:
            points.extend([p[0], p[1], p[2]])
        
        self.latest_points = struct.pack(f'{len(points)}f', *points)

    def path_callback(self, msg):
        if msg.type == Marker.LINE_STRIP or msg.type == Marker.LINE_LIST:
            p_list = [{'x': p.x, 'y': p.y, 'z': p.z} for p in msg.points]
            self.latest_path = p_list

    def odom_callback(self, msg):
        self.vehicle_z = msg.pose.pose.position.z

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Resize for performance if needed
            cv_image = cv2.resize(cv_image, (320, 240))
            _, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            self.latest_frame = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def publish_goal(self, x, y, z):
        # 1. Publish Joy to trigger autonomy mode (Simulating GoalpointTool.cpp)
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "goalpoint_tool"
        joy_msg.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0] # Button 7 is index 7 (8th button)? C++ push_back order check:
        # C++: 7 zeros, then a 1. Index 7.
        self.joy_pub.publish(joy_msg)

        # 2. Publish Goal Point
        goal_msg = PointStamped()
        goal_msg.header.stamp = joy_msg.header.stamp
        goal_msg.header.frame_id = "map"
        goal_msg.point.x = x
        goal_msg.point.y = y
        # Use current vehicle_z if z not provided or flat
        goal_msg.point.z = z if z != 0.0 else self.vehicle_z 

        # Publish twice like the C++ tool does
        self.goal_pub.publish(goal_msg)
        time.sleep(0.01)
        self.goal_pub.publish(goal_msg)
        
        self.get_logger().info(f"Published Goal: {x}, {y}, {goal_msg.point.z}")


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS Spin Error: {e}")

@app.on_event("startup")
async def startup_event():
    rclpy.init()
    global ros_node
    ros_node = ROSNode()
    t = threading.Thread(target=ros_spin_thread, args=(ros_node,), daemon=True)
    t.start()

@app.on_event("shutdown")
def shutdown_event():
    if rclpy.ok():
        rclpy.shutdown()

# --- WebSocket Endpoints ---
@app.websocket("/ws/points")
async def websocket_points(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await asyncio.sleep(0.1)
            
            if ros_node and ros_node.latest_points:
                await websocket.send_bytes(ros_node.latest_points)
    except WebSocketDisconnect:
        pass

TARGET_FRAMES = ["base_link", "livox_frame", "odom"]

@app.websocket("/ws/tf")
async def websocket_tf(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await asyncio.sleep(0.1)
            
            if not ros_node:
                continue

            tfs_snapshot = {}
            for child in TARGET_FRAMES:
                try:
                    t = ros_node.tf_buffer.lookup_transform(
                        "map",
                        child,
                        Time(),
                        Duration(seconds=0.0) 
                    )
                    tfs_snapshot[child] = {
                        "translation": {
                            "x": t.transform.translation.x,
                            "y": t.transform.translation.y,
                            "z": t.transform.translation.z
                        },
                        "rotation": {
                            "x": t.transform.rotation.x,
                            "y": t.transform.rotation.y,
                            "z": t.transform.rotation.z,
                            "w": t.transform.rotation.w
                        }
                    }
                except:
                    pass
            
            data_packet = {
                "tfs": tfs_snapshot,
                "path": []
            }
            if ros_node:
                data_packet["path"] = ros_node.latest_path
            
            await websocket.send_json(data_packet)
    except WebSocketDisconnect:
        pass

@app.websocket("/ws/video")
async def websocket_video(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            if ros_node and ros_node.latest_frame:
                await websocket.send_text(ros_node.latest_frame)
            await asyncio.sleep(0.1) # 10 FPS
    except Exception:
        pass

# --- HTTP Endpoints for Goals & Waypoints ---

@app.post("/navigate")
async def navigate_to_goal(goal: GoalRequest):
    if ros_node:
        ros_node.publish_goal(goal.x, goal.y, goal.z)
        return {"status": "Goal sent", "target": goal}
    raise HTTPException(status_code=503, detail="ROS Node not ready")

@app.get("/waypoints")
def get_waypoints():
    if not os.path.exists(WAYPOINTS_FILE):
        return []
    with open(WAYPOINTS_FILE, 'r') as f:
        return json.load(f)

@app.post("/waypoints")
def save_waypoint(wp: Waypoint):
    waypoints = []
    if os.path.exists(WAYPOINTS_FILE):
        with open(WAYPOINTS_FILE, 'r') as f:
            try:
                waypoints = json.load(f)
            except:
                pass
    waypoints.append(wp.dict())
    with open(WAYPOINTS_FILE, 'w') as f:
        json.dump(waypoints, f)
    return {"status": "Saved", "waypoints": waypoints}

@app.delete("/waypoints/{name}")
def delete_waypoint(name: str):
    if not os.path.exists(WAYPOINTS_FILE):
        return []
    with open(WAYPOINTS_FILE, 'r') as f:
        waypoints = json.load(f)
    
    waypoints = [w for w in waypoints if w['name'] != name]
    
    with open(WAYPOINTS_FILE, 'w') as f:
        json.dump(waypoints, f)
    return {"status": "Deleted", "waypoints": waypoints}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
