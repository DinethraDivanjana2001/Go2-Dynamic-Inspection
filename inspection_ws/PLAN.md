# Inspection System — Phase 2 Plan
# Visual Servoing + Behaviour Tree + Multi-Object + MQTT Publishing

**Date:** March 2026  
**Status:** Planning — current pipeline proven, now building full autonomous inspection system

---

## 🗂️ New Workspace

```
Jetson_orin_nano/
└── inspection_ws/            ← NEW clean workspace (this folder)
    ├── PLAN.md               ← this file
    ├── pipeline/             ← core IBVS pipeline (from jetson_workspace)
    ├── behaviour_tree/       ← BT implementation (py_trees)
    │   └── nodes/            ← individual BT action/condition nodes
    ├── ros_nodes/            ← ROS2 nodes (if ROS2 is set up on Jetson)
    ├── mqtt/                 ← MQTT publisher to broker
    ├── capture/              ← image capture + queue logic
    ├── tests/                ← unit tests for each node
    └── docs/                 ← documentation
```

**Deploy to Jetson:**
```bash
scp -r inspection_ws/ rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/
```

---

## 🌳 Behaviour Tree — Full Structure

```
Root [Sequence]
│
├── [Condition]   Is robot at inspection position?
│
├── [Sequence]    Detect & Inspect All Objects
│   │
│   ├── [Action]  Insta360 YOLO Detection
│   │     ├── SUCCESS: 1+ objects found → continue
│   │     └── FAILURE: no objects → [Action] Rotate Robot + RETRY (up to 3x)
│   │
│   ├── [Action]  Sort Detections by Confidence (ByteTracker assigns IDs)
│   │
│   └── [Repeat]  For Each Detected Object (loop over object list)
│       │
│       ├── [Action]  COARSE Positioning
│       │     Insta360 → cubic formula → servo move
│       │     SUCCESS: Logitech sees object
│       │     FAILURE: → back to Insta360 detection
│       │
│       ├── [Action]  FINE Centering (IBVS)
│       │     Logitech IBVS → center object (<10px error)
│       │     SUCCESS: centered
│       │     FAILURE (lost / max_iter) → retry COARSE
│       │
│       ├── [Sequence]  Capture 4 Images
│       │   ├── [Action]  Capture Image 1
│       │   ├── [Action]  Re-center IBVS (ensure still centered)
│       │   ├── [Action]  Capture Image 2
│       │   ├── [Action]  Re-center IBVS
│       │   ├── [Action]  Capture Image 3
│       │   ├── [Action]  Re-center IBVS
│       │   └── [Action]  Capture Image 4
│       │
│       ├── [Action]  Publish 4 Images via MQTT
│       │     Topic: inspection/images/{object_id}
│       │     Payload: {id, timestamp, images[4], object_class, confidence}
│       │
│       └── [Action]  Mark Object as Inspected → next object
│
└── [Action]  Report: Inspection Complete (publish summary via MQTT)
```

---

## 📦 Components to Build

### 1. ByteTracker — Multi-Object Tracking

**Why:** If Insta360 sees 3 fire extinguishers, we need to inspect ALL of them one by one without re-detecting already-inspected objects.

**How it works:**
- YOLO detects N objects in each frame
- ByteTracker assigns persistent IDs (track_id=1, 2, 3...)
- We process objects in order: track_id=1 first, then 2, then 3
- Once an object is inspected, its ID is added to "inspected" list and skipped

**Library:** `pip install bytetracker` or use Ultralytics built-in tracker  
**Replacement of current `SimpleTracker`** in `ibvs_pipeline.py`

```python
# Current (simple):
tracker = SimpleTracker(max_disappeared=30)

# Phase 2 (ByteTracker):
from ultralytics import YOLO
results = model.track(frame, persist=True, tracker="bytetrack.yaml")
# Each box now has .id (track_id)
```

---

### 2. Behaviour Tree — py_trees

**Library:** `pip install py_trees`  
(No ROS2 needed — py_trees is standalone Python, py_trees_ros adds ROS2 integration)

**BT Node Types we need:**

| Node | Type | What it does |
|------|------|-------------|
| `Insta360DetectNode` | Action | Run YOLO on Insta360, populate object list |
| `HasObjectsCondition` | Condition | Check if object list is non-empty |
| `CoarsePositionNode` | Action | Cubic formula → servo move |
| `IBVSCenterNode` | Action | Run IBVS until centered or fail |
| `CaptureImageNode` | Action | Capture one Logitech frame, save |
| `ReCenterNode` | Action | Quick IBVS re-center check |
| `PublishMQTTNode` | Action | Send 4 images + metadata to MQTT |
| `RotateRobotNode` | Action | Tell GO2 robot to rotate slightly |
| `MarkInspectedNode` | Action | Add track_id to inspected set |

---

### 3. Image Capture Logic

After IBVS centers:
```python
captured_images = []
for i in range(4):
    # 1. Verify still centered (quick IBVS check)
    re_center_if_needed()
    
    # 2. Capture frame from Logitech
    ret, frame = cap_logi.read()
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    
    # 3. Save with metadata
    filename = f"capture_{object_id}_{i+1}_{timestamp}.jpg"
    cv2.imwrite(f"capture/{filename}", frame)
    captured_images.append({'path': filename, 'frame': frame})
    
    time.sleep(0.5)  # brief pause between captures
```

4 images per object gives redundancy for downstream AI analysis.

---

### 4. MQTT Publisher

**Library:** `pip install paho-mqtt`

**Topic structure:**
```
inspection/
├── images/{object_id}      ← 4 captured images (base64 encoded)
├── status                  ← pipeline status updates
└── summary                 ← end-of-round inspection report
```

**Payload format:**
```json
{
  "object_id": 1,
  "object_class": "fire_extinguisher",
  "confidence": 0.92,
  "timestamp": "2026-03-04T14:30:00",
  "robot_position": {"x": 0.0, "y": 0.0},
  "servo_angles": {"pan": 95, "tilt": 88},
  "images": [
    {"index": 1, "data": "<base64>", "filename": "cap_1_001.jpg"},
    {"index": 2, "data": "<base64>", "filename": "cap_1_002.jpg"},
    {"index": 3, "data": "<base64>", "filename": "cap_1_003.jpg"},
    {"index": 4, "data": "<base64>", "filename": "cap_1_004.jpg"}
  ]
}
```

**Broker:** configured in `mqtt/config.yaml`

---

### 5. ROS2 Integration (when ROS2 available)

ROS2 nodes to add:

| Node | Topic/Service | Purpose |
|------|--------------|---------|
| `camera_publisher` | `/insta360/image_raw` | Publish Insta360 stream |
| `camera_publisher` | `/logitech/image_raw` | Publish Logitech stream |
| `ibvs_action_server` | `/ibvs/center` (Action) | IBVS as ROS2 action |
| `inspection_service` | `/inspection/start` (Service) | Trigger full inspection round |
| `image_capture_service` | `/inspection/capture` (Service) | Capture + return images |

**ROS Bags** — record with:
```bash
ros2 bag record /insta360/image_raw /logitech/image_raw /ibvs/error /servo/commands
```

For integration with existing BT in the GO2 system:
- Our inspection pipeline becomes a **single BT Action node** 
- Input: trigger signal from main BT
- Output: SUCCESS (all objects inspected) or FAILURE (error)

---

## 🔄 Full Data Flow

```
GO2 arrives at inspection point
      ↓
BT: START INSPECTION
      ↓
Insta360 YOLO → [3 objects found: IDs 1, 2, 3]
      ↓
ByteTracker assigns persistent IDs
      ↓
FOR EACH object (1→2→3):
   Coarse servo move → IBVS center
   Capture 4 images
   MQTT publish {images + metadata}
   Mark inspected
      ↓
BT: INSPECTION COMPLETE → publish summary
      ↓
GO2 moves to next inspection point
```

---

## 📋 Implementation Phases

### Phase 2a — Multi-Object + Capture (no ROS, standalone)
- [ ] Replace `SimpleTracker` with ByteTracker in pipeline
- [ ] Add "inspect all objects" loop logic
- [ ] Implement 4-image capture with IBVS re-centering
- [ ] Save captures to `capture/` folder with metadata JSON

### Phase 2b — MQTT Integration
- [ ] Set up `paho-mqtt` connection to broker
- [ ] Implement `MQTTPublisher` class
- [ ] Test image publishing (base64 encode + send)
- [ ] Verify broker receives and decodes correctly

### Phase 2c — Behaviour Tree
- [ ] Install `py_trees`
- [ ] Convert each pipeline stage to BT node class
- [ ] Build tree structure
- [ ] Add error recovery (retry logic, robot rotation trigger)
- [ ] Test BT with simulated failures

### Phase 2d — ROS2 Integration (when ROS2 set up)
- [ ] Create ROS2 package: `visual_inspection_ros`
- [ ] Wrap BT as ROS2 node
- [ ] Add camera topic publishers
- [ ] Add ROS2 service for MQTT publish
- [ ] Integrate with GO2 main behaviour tree
- [ ] Record ROS bags for validation

---

## ⚙️ Dependencies to Install

```bash
# On Jetson (in clean venv):
pip install py_trees          # Behaviour Tree
pip install paho-mqtt         # MQTT
pip install bytetracker       # Multi-object tracking (or ultralytics tracker)

# Already installed:
# ultralytics, opencv-python, pyserial, numpy, pyyaml
```

---

## 🗂️ File Structure (implementation_ws)

```
inspection_ws/
├── PLAN.md                              ← this file
├── requirements.txt
├── config.yaml                          ← all config in one place
│
├── pipeline/
│   ├── ibvs_pipeline_core.py           ← hardware layer (cameras, arduino, YOLO)
│   ├── calibration_config.py           ← Insta360 calibration formulas
│   └── ibvs_controller.py             ← IBVS PID controller class
│
├── behaviour_tree/
│   ├── inspection_tree.py              ← main BT definition
│   └── nodes/
│       ├── detect_node.py              ← Insta360 YOLO + ByteTracker
│       ├── coarse_node.py              ← servo coarse positioning
│       ├── ibvs_node.py               ← IBVS fine centering
│       ├── capture_node.py            ← 4-image capture
│       ├── recenter_node.py           ← re-center between captures
│       ├── publish_node.py            ← MQTT publish
│       └── rotate_robot_node.py       ← trigger GO2 rotation
│
├── mqtt/
│   ├── publisher.py                   ← MQTTPublisher class
│   └── config.yaml                   ← broker address, topics
│
├── capture/
│   └── (captured images saved here)
│
├── ros_nodes/                         ← when ROS2 available
│   ├── camera_node.py
│   ├── ibvs_action_node.py
│   └── inspection_service_node.py
│
└── tests/
    ├── test_bt_nodes.py
    ├── test_mqtt.py
    └── test_capture.py
```

---

## ❓ Things to Clarify / Decide

1. **MQTT broker address** — what is the broker IP/hostname?
2. **ROS2 version** — is ROS2 installed on Jetson? (Humble / Iron / Jazzy?)
3. **GO2 BT framework** — which BT library does the GO2 use? (BehaviorTree.CPP / py_trees?)
4. **Image format for MQTT** — base64 JPEG or raw bytes?
5. **ByteTracker** — use Ultralytics built-in tracker or separate bytetracker package?
6. **Re-center threshold** — how many px drift is OK between captures?
7. **Object priority** — if 3 extinguishers, do we inspect nearest first or left-to-right?
