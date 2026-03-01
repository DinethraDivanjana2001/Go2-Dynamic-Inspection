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
from datetime import datetime, timedelta

# SQLAlchemy & Auth
from sqlalchemy import create_engine, Column, Integer, String
from sqlalchemy.orm import sessionmaker, declarative_base, Session
from passlib.context import CryptContext
from jose import JWTError, jwt
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Depends, status
from fastapi.security import OAuth2PasswordBearer, OAuth2PasswordRequestForm
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

load_dotenv()


# TF2
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

# Camera & Image
import cv2
from cv_bridge import CvBridge
import base64
from sensor_msgs.msg import Image

# Initialize FastAPI
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Authentication & Database Config ---
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite:///./test.db")
SECRET_KEY = os.getenv("SECRET_KEY", "fallback_secret_key")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 * 7 # 7 days

engine = create_engine(DATABASE_URL)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="login")

class User(Base):
    __tablename__ = "users"
    id = Column(Integer, primary_key=True, index=True)
    username = Column(String, unique=True, index=True)
    hashed_password = Column(String)

Base.metadata.create_all(bind=engine)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: timedelta | None = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

async def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)):
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise credentials_exception
    except JWTError:
        raise credentials_exception
    user = db.query(User).filter(User.username == username).first()
    if user is None:
        raise credentials_exception
    return user


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
        super().__init__('mission_planner_ui_backend')
        
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
        
        # TF Buffer
        # State
        self.latest_points = None
        self.latest_path = []
        self.vehicle_z = 0.0
        self.latest_odom = None
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
        self.latest_odom = msg

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
async def websocket_points(websocket: WebSocket, token: str = None, db: Session = Depends(get_db)):
    if not token:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return
    try:
        user = await get_current_user(token, db)
    except HTTPException:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

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
async def websocket_tf(websocket: WebSocket, token: str = None, db: Session = Depends(get_db)):
    if not token:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return
    try:
        user = await get_current_user(token, db)
    except HTTPException:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    await websocket.accept()
    try:
        while True:
            await asyncio.sleep(0.05) # 20Hz Update
            
            if not ros_node:
                continue

            tfs_snapshot = {}
            # 1. Try Standard TF Lookup
            for child in TARGET_FRAMES:
                try:
                    t = ros_node.tf_buffer.lookup_transform(
                        "camera_init", # Often the map frame in LIO
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
            
            # 2. Fallback: If base_link missing, use Odometry
            # (Assuming base_link is the robot body)
            if "base_link" not in tfs_snapshot and ros_node.latest_odom:
                 p = ros_node.latest_odom.pose.pose.position
                 q = ros_node.latest_odom.pose.pose.orientation
                 tfs_snapshot["base_link"] = {
                    "translation": {"x": p.x, "y": p.y, "z": p.z},
                    "rotation": {"x": q.x, "y": q.y, "z": q.z, "w": q.w}
                 }

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
async def websocket_video(websocket: WebSocket, token: str = None, db: Session = Depends(get_db)):
    if not token:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return
    try:
        user = await get_current_user(token, db)
    except HTTPException:
        await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
        return

    await websocket.accept()
    try:
        while True:
            if ros_node and ros_node.latest_frame:
                await websocket.send_text(ros_node.latest_frame)
            await asyncio.sleep(0.1) # 10 FPS
    except Exception:
        pass

# --- Auth Endpoints ---

class UserCreate(BaseModel):
    username: str
    password: str

@app.post("/register")
def register_user(user: UserCreate, db: Session = Depends(get_db)):
    db_user = db.query(User).filter(User.username == user.username).first()
    if db_user:
        raise HTTPException(status_code=400, detail="Username already registered")
    hashed_password = get_password_hash(user.password)
    new_user = User(username=user.username, hashed_password=hashed_password)
    db.add(new_user)
    db.commit()
    db.refresh(new_user)
    return {"message": "User created successfully"}

@app.post("/login")
def login_for_access_token(form_data: OAuth2PasswordRequestForm = Depends(), db: Session = Depends(get_db)):
    user = db.query(User).filter(User.username == form_data.username).first()
    if not user or not verify_password(form_data.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

# --- HTTP Endpoints for Goals & Waypoints ---

@app.post("/navigate")
async def navigate_to_goal(goal: GoalRequest, current_user: User = Depends(get_current_user)):
    if ros_node:
        ros_node.publish_goal(goal.x, goal.y, goal.z)
        return {"status": "Goal sent", "target": goal}
    raise HTTPException(status_code=503, detail="ROS Node not ready")

@app.get("/waypoints")
def get_waypoints(current_user: User = Depends(get_current_user)):

    if not os.path.exists(WAYPOINTS_FILE):
        return []
    with open(WAYPOINTS_FILE, 'r') as f:
        return json.load(f)

@app.post("/waypoints")
def save_waypoint(wp: Waypoint, current_user: User = Depends(get_current_user)):
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
def delete_waypoint(name: str, current_user: User = Depends(get_current_user)):
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
