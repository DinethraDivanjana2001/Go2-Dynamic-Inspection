import json
import struct
import math
import os
import time
from datetime import datetime, timedelta
import uvicorn
import asyncio
import paho.mqtt.client as mqtt

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

# --- MQTT Configuration ---
MQTT_BROKER = os.getenv("MQTT_BROKER", "broker.hivemq.com")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
BASE_TOPIC = "robo_gen_labs/go2_robot_1"

# Global State from MQTT
mqtt_state = {
    "points": b"",
    "tfs": {},
    "path": [],
    "video": ""
}

def on_mqtt_connect(client, userdata, flags, rc):
    print(f"MQTT Connected with result code {rc}")
    client.subscribe(f"{BASE_TOPIC}/telemetry/#")

def on_mqtt_message(client, userdata, msg):
    topic = msg.topic
    if topic.endswith("/points"):
        mqtt_state["points"] = msg.payload
    elif topic.endswith("/tf"):
        try:
            data = json.loads(msg.payload.decode())
            mqtt_state["tfs"] = data.get("tfs", {})
        except: pass
    elif topic.endswith("/path"):
        try:
            data = json.loads(msg.payload.decode())
            mqtt_state["path"] = data.get("path", [])
        except: pass
    elif topic.endswith("/video"):
        mqtt_state["video"] = msg.payload.decode('utf-8')

mqtt_client = mqtt.Client(client_id="go2_fastapi_backend", protocol=mqtt.MQTTv311)
mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message


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

@app.on_event("startup")
async def startup_event():
    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"MQTT Startup Error: {e}")

@app.on_event("shutdown")
def shutdown_event():
    mqtt_client.loop_stop()
    mqtt_client.disconnect()

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
            if mqtt_state["points"]:
                await websocket.send_bytes(mqtt_state["points"])
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
            
            data_packet = {
                "tfs": mqtt_state["tfs"],
                "path": mqtt_state["path"]
            }
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
            if mqtt_state["video"]:
                await websocket.send_text(mqtt_state["video"])
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
    payload = json.dumps({"x": goal.x, "y": goal.y, "z": goal.z})
    print(f"DEBUG: server.py received /navigate from frontend: {payload}")
    mqtt_client.publish(f"{BASE_TOPIC}/commands/navigate", payload=payload, qos=1)
    return {"status": "Goal sent to MQTT", "target": goal}

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
