# Visual Inspection ROS2 Package

**Part of:** Autonomous Robot Inspection System  
**Stack:** ROS2 Humble · Python · py_trees_ros · TensorRT on Jetson Orin Nano  
**This module:** Visual inspection — detects objects, centers camera using IBVS, captures images, publishes via MQTT

---

## 📋 What This Does

When the robot arrives at an inspection point, this module:

1. **Detects** all inspection objects (fire extinguishers, gauges) using Insta360 wide-angle camera + YOLO
2. **Points** the Logitech camera at each object using pan-tilt servos
3. **Centers** precisely using IBVS (Image-Based Visual Servoing)
4. **Captures** 4 images per object
5. **Publishes** images + metadata to MQTT broker

---

## 🔌 Interface for Behaviour Tree

### Action Server (call this from BT)

```
/visual_inspection/inspect_objects
Type: visual_inspection_ros/action/InspectObjects
```

**Goal:**
```yaml
max_objects: 0      # 0 = inspect all detected objects, N = inspect only N
return_home: true   # servo returns to 90,90 after inspection done
```

**Result:**
```yaml
success: true/false
objects_inspected: 3
failed_reason: ""   # "no_detection" / "ibvs_timeout" / "mqtt_error" / ""
```

**Feedback (during execution):**
```yaml
current_step: "detecting"  # detecting / coarse / ibvs / capturing / publishing
current_object: 1
ibvs_error_px: 12.3
```

### Topics Published

| Topic | Type | Notes |
|-------|------|-------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` | Wide-angle camera |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` | Narrow IBVS camera |
| `/visual_inspection/status` | `std_msgs/String` | IDLE / COARSE / FINE / CAPTURING |
| `/visual_inspection/ibvs_error` | `geometry_msgs/Point` | Pixel error x,y |
| `/visual_inspection/detections` | `std_msgs/String` | JSON detected objects |

### py_trees_ros Nodes (import into main BT)

```python
from visual_inspection_ros.bt_nodes import (
    DetectObjectsNode,   # Condition: SUCCESS if objects detected
    InspectObjectsNode,  # Action: runs full inspection cycle
)
```

### BT Integration Example

```python
import py_trees
import py_trees_ros
from visual_inspection_ros.bt_nodes import InspectObjectsNode

# In your main BT definition:
inspection = InspectObjectsNode(
    name="Inspect Objects",
    max_objects=0,
    return_home=True
)

root = py_trees.composites.Sequence("Root", memory=True)
root.add_children([
    # ... your navigation nodes ...
    inspection,            # ← add our node here
    # ... rest of your tree ...
])
```

---

## 🚀 Setup on Jetson

### Prerequisites (already on Jetson)
- ROS2 Humble
- `~/Documents/Visual_Inspection_ws/venv/` with ultralytics, opencv, pyserial
- `~/Documents/Visual_Inspection_ws/weights/yolo11n.engine` (TensorRT FP16)
- udev rules for cameras + Arduino (`/dev/insta360`, `/dev/logitech`, `/dev/arduino`)

### Install

```bash
# 1. Clone or SCP this folder to Jetson
# (from laptop):
scp -r inspection_ws/ rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/

# 2. On Jetson — source environment
source ~/Documents/Visual_Inspection_ws/venv/bin/activate
source /opt/ros/humble/setup.bash

# 3. Install extra dependencies
pip install paho-mqtt py_trees py_trees_ros

# 4. Build ROS2 package
cd ~/Documents/Visual_Inspection_ws/inspection_ws
colcon build --packages-select visual_inspection_ros
source install/setup.bash
```

### Run

```bash
# Terminal 1: Camera publisher
ros2 run visual_inspection_ros camera_node

# Terminal 2: IBVS Action Server (main node)
ros2 run visual_inspection_ros ibvs_action_server

# Test from terminal (without BT):
ros2 action send_goal /visual_inspection/inspect_objects \
  visual_inspection_ros/action/InspectObjects \
  "{max_objects: 0, return_home: true}"
```

---

## ⚙️ Configuration

Edit `visual_inspection_ros/config.yaml` before running:

```yaml
mqtt:
  broker: "192.168.x.x"    # ← set your broker IP
  port: 1883
  topic_prefix: "inspection"

ibvs:
  centered_threshold_px: 10
  max_iterations: 200
  servo_delay: 0.1

capture:
  images_per_object: 4
  save_dir: "~/Documents/Visual_Inspection_ws/captures"
```

---

## 📁 Package Structure

```
inspection_ws/
├── README.md                        ← this file
├── PLAN.md                          ← full implementation plan
│
└── visual_inspection_ros/           ← ROS2 package
    ├── package.xml
    ├── setup.py
    ├── action/
    │   └── InspectObjects.action    ← custom action definition
    ├── msg/
    │   └── InspectionResult.msg
    └── visual_inspection_ros/
        ├── camera_node.py           ← publishes camera feeds
        ├── servo_node.py            ← Arduino servo control
        ├── ibvs_action_server.py    ← main IBVS action
        ├── inspection_node.py       ← multi-object coordinator
        └── bt_nodes/
            ├── detect_bt_node.py    ← BT Condition node
            ├── ibvs_bt_node.py      ← BT Action node
            └── capture_bt_node.py   ← BT capture node
```

---

## 🔧 Hardware

| Device | Port | Notes |
|--------|------|-------|
| Insta360 X3 | `/dev/insta360` | udev symlink — stable always |
| Logitech C920 | `/dev/logitech` | udev symlink — stable always |
| Arduino Uno | `/dev/arduino` | udev symlink — stable always |

Pan-tilt servo range: Pan 0-180°, Tilt 0-180°, Home = 90,90

---

## 🐛 Troubleshooting

**Cameras not found:**
```bash
ls -la /dev/insta360 /dev/logitech   # check symlinks
v4l2-ctl --list-devices              # check all video devices
```

**Arduino not found:**
```bash
ls /dev/arduino /dev/ttyACM*
sudo chmod 666 /dev/ttyACM0
```

**TensorRT engine fails:**
```bash
# Check engine exists
ls ~/Documents/Visual_Inspection_ws/weights/yolo11n.engine
# Re-export if missing (takes ~7 min, only needed once)
python3 -c "from ultralytics import YOLO; YOLO('weights/yolo11n.pt').export(format='engine', device=0, half=True)"
```
