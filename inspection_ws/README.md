# Visual Inspection ROS2 System — Complete Reference

> **Jetson Orin Nano** | ROS2 Humble | Python 3.10 | TensorRT YOLO11n

---

## Table of Contents
1. [System Overview](#1-system-overview)
2. [Package Structure](#2-package-structure)
3. [Action Interface](#3-action-interface)
4. [ROS2 Nodes](#4-ros2-nodes)
5. [Topics Reference](#5-topics-reference)
6. [Startup Sequence](#6-startup-sequence)
7. [Testing Each Node](#7-testing-each-node)
8. [Behavior Tree Integration](#8-behavior-tree-integration)
9. [Configuration](#9-configuration)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                     Behavior Tree (py_trees_ros)                │
│   InspectAction ── CheckBackSide ── RotateRobot ── NextObject   │
└────────────────────────────┬────────────────────────────────────┘
                             │ calls action
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│               ibvs_action_server                                │
│                                                                 │
│  Stage 1 DETECT (Insta360, up to 20s)                          │
│    YOLO+ByteTrack → front objects (cy<200px) + back objects     │
│    degree-4 calibration → coarse servo angles                   │
│                                                                 │
│  Stage 2 IBVS (Logitech, up to 40s)                            │
│    YOLO → PID servo loop (KP=0.12, KI=0.002, KD=0.02)          │
│    max 3 deg/step, converge at <10px error                      │
│    object can vanish 3s without aborting                        │
│                                                                 │
│  Capture: 4 images from Logitech → MQTT                        │
└──────────┬──────────────────────┬───────────────────────────────┘
           │ /servo/pan_tilt       │ camera topics
           ▼                       ▼
    servo_node              camera_node
    (Arduino serial)        (Insta360 + Logitech)
```

### Pipeline Decisions

| Condition | Result |
|-----------|--------|
| Object detected in front (cy < 200px) | Proceed with inspection |
| Objects ONLY in back (cy > 200px) | `object_in_back=True` → BT rotates robot 180° |
| Mix of front + back | Inspect front first, then signal BT for back |
| Multiple front objects | ByteTrack assigns IDs → inspect in ID order |
| Object vanishes ≤3s during IBVS | Wait and retry (noise/occlusion tolerance) |
| IBVS fails for one object | Skip it, continue to next object |

---

## 2. Package Structure

```
inspection_ws/
├── visual_inspection_interfaces/     # Custom action definitions
│   └── action/
│       └── InspectObjects.action
│
└── visual_inspection_ros/            # All ROS2 nodes
    └── visual_inspection_ros/
        ├── camera_node.py            # Camera publisher
        ├── servo_node.py             # Arduino servo control
        └── ibvs_action_server.py     # Full inspection pipeline
```

---

## 3. Action Interface

**File:** `visual_inspection_interfaces/action/InspectObjects.action`

### Goal
```
int32  max_objects   # 0=all, N=first N front objects only
bool   return_home   # true=servo goes to 90,90 after done
```

### Result
```
bool   success             # true if at least one object inspected
int32  objects_inspected   # number successfully inspected
int32  objects_found       # total objects detected (front+back)
bool   object_in_back      # BT should rotate robot 180° if true
string failed_reason       # "" | "no_detection" | "all_in_back" |
                           # "ibvs_timeout" | "logi_no_detection"
```

### Feedback (published during execution)
```
string  current_step    # "detecting" | "coarse" | "ibvs" | "capturing"
int32   current_object  # 1, 2, 3... which object being processed
float32 ibvs_error_px   # pixel error during IBVS phase
```

### BT Decision Table Based on Result

| `success` | `object_in_back` | `failed_reason` | BT Action |
|-----------|-----------------|-----------------|-----------|
| `true`    | `false`          | `""`            | Done — return to base |
| `true`    | `true`           | `""`            | Done front, ROTATE 180° |
| `false`   | `true`           | `"all_in_back"` | ROTATE 180°, retry |
| `false`   | `false`          | `"no_detection"`| Navigate to next position |
| `false`   | `false`          | `"ibvs_timeout"`| Retry or move on |

---

## 4. ROS2 Nodes

### 4.1 `camera_node` (`camera_publisher`)

Publishes both camera streams at 30 Hz.

**Detection strategy** (3 layers):
1. udev symlink `/dev/insta360` and `/dev/logitech`
2. USB vendor ID scan via sysfs
3. Name-based fallback

**Subscriptions:** None

**Publications:**

| Topic | Type | Rate |
|-------|------|------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` | ~15 Hz |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` | ~30 Hz |

---

### 4.2 `servo_node` (`servo_controller`)

Receives pan/tilt commands, writes to Arduino via serial.

**Subscriptions:**

| Topic | Type | Format |
|-------|------|--------|
| `/servo/pan_tilt` | `std_msgs/Int16MultiArray` | `[tilt, pan]` |

**Serial command format:** `"tilt,pan\n"` at 9600 baud

**Arduino detection:** udev symlink `/dev/arduino` → VID scan (`2341`, `1a86`, `0403`)

**On shutdown:** sends `90,90` to return servos home.

**Test:**
```bash
ros2 topic pub --once /servo/pan_tilt std_msgs/msg/Int16MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [90, 90]}"
```

---

### 4.3 `ibvs_action_server` (`ibvs_action_server`)

Full inspection pipeline as ROS2 Action Server.

**Subscriptions:**

| Topic | Type |
|-------|------|
| `/visual_inspection/insta360/image_raw` | `sensor_msgs/Image` |
| `/visual_inspection/logitech/image_raw` | `sensor_msgs/Image` |

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `/servo/pan_tilt` | `std_msgs/Int16MultiArray` | servo commands |
| `/visual_inspection/debug` | `sensor_msgs/Image` | combined debug view (1280×360) |

**Action:** `/visual_inspection/inspect_objects` (InspectObjects)

**Key parameters (edit in class constants):**
```python
CONF_INSTA           = 0.5    # YOLO confidence for initial detection
CONF_IBVS            = 0.3    # YOLO confidence during IBVS (lower = more tolerant)
INSTA_SEARCH_TIMEOUT = 20.0   # seconds to search Insta360 before giving up
LOGI_FIRST_DET_WAIT  = 5.0    # seconds to wait for object in Logitech after coarse
IBVS_TOTAL_TIMEOUT   = 40.0   # total IBVS time budget (seconds)
IBVS_LOST_PATIENCE   = 3.0    # seconds object can vanish during IBVS
COARSE_WAIT          = 2.0    # seconds to wait after coarse servo move
FRONT_Y_MAX          = 200    # Insta360 pixels: cy < this = front, cy > this = back
TILT_REVERSED        = True   # flip tilt direction (hardware-specific)
KP, KI, KD          = 0.12, 0.002, 0.02  # IBVS PID gains
MAX_STEP_DEG         = 3.0    # max servo step per IBVS iteration
IBVS_TOL_PX          = 10.0   # converged when pixel error < this
IMAGES_PER_OBJ       = 4      # Logitech images captured per object
```

**Debug view layout (`/visual_inspection/debug`):**
```
┌──────────────────────────┬──────────────────────────┐
│  INSTA360 (Coarse)       │  LOGITECH (IBVS)         │
│                          │                          │
│  Green box = front obj   │  Green box + conf        │
│  Orange box = back obj   │  Blue arrow → center     │
│  FRONT/BACK boundary     │  IBVS err=(x,y) Iter=N   │
│  ID labels from ByteTrack│                          │
│                          │                          │
│  Mode: DETECTING  FPS:4  │                          │
└──────────────────────────┴──────────────────────────┘
```

**Modes visible in debug:**
- `IDLE` — no goal active
- `DETECTING` — searching Insta360
- `COARSE` — servo moving to coarse position
- `IBVS` — fine centering on Logitech
- `CAPTURING` — taking 4 images

---

## 5. Topics Reference

```
/visual_inspection/insta360/image_raw   sensor_msgs/Image     ~15 Hz
/visual_inspection/logitech/image_raw   sensor_msgs/Image     ~30 Hz
/visual_inspection/debug                sensor_msgs/Image     ~4 Hz (idle) / live (goal)
/servo/pan_tilt                         std_msgs/Int16MultiArray  on command
```

---

## 6. Startup Sequence

> **Run these in order.** Each terminal needs sourcing first.

```bash
# Copy into every new terminal:
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
```

| Terminal | Command | Wait for |
|----------|---------|----------|
| **T1** | `ros2 run visual_inspection_ros camera_node` | `Camera publisher running at 30 Hz` |
| **T2** | `ros2 run visual_inspection_ros servo_node` | `Servo node ready -- listening on /servo/pan_tilt` |
| **T3** | `ros2 run visual_inspection_ros ibvs_action_server` | `Action server ready at /visual_inspection/inspect_objects` |
| **T4** | `rviz2` | Open → Add Image → `/visual_inspection/debug` |
| **T5** | *(send goal, see below)* | — |

---

## 7. Testing Each Node

### Test camera_node
```bash
ros2 topic hz /visual_inspection/insta360/image_raw
ros2 topic hz /visual_inspection/logitech/image_raw
```
Expected: ~15 Hz and ~30 Hz respectively.

### Test servo_node
```bash
# Move to 45,90
ros2 topic pub --once /servo/pan_tilt std_msgs/msg/Int16MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [45, 90]}"

# Return home
ros2 topic pub --once /servo/pan_tilt std_msgs/msg/Int16MultiArray \
  "{layout: {dim: [], data_offset: 0}, data: [90, 90]}"
```

### Test ibvs_action_server (no object)
```bash
ros2 action send_goal /visual_inspection/inspect_objects \
  visual_inspection_interfaces/action/InspectObjects \
  "{max_objects: 0, return_home: true}"
```
Expected: wai 20s then `failed_reason: no_detection`

### Full pipeline test (with object)
1. Place fire extinguisher / gauge in front of Insta360
2. Watch `/visual_inspection/debug` in RViz2 — green box should appear
3. Send goal:
```bash
ros2 action send_goal --feedback /visual_inspection/inspect_objects \
  visual_inspection_interfaces/action/InspectObjects \
  "{max_objects: 1, return_home: true}"
```
4. Watch terminal for stage progression:
   - `Detected: 1 front object(s)` → `Coarse: pan=XX tilt=XX` → `IBVS converged`

### Test back-side detection
Place object behind the robot (visible in bottom half of Insta360 frame).
Result should include: `object_in_back: true`, `failed_reason: all_in_back`

### Test multi-object detection
Place 2-3 objects in front of Insta360.
```bash
ros2 action send_goal --feedback /visual_inspection/inspect_objects \
  visual_inspection_interfaces/action/InspectObjects \
  "{max_objects: 0, return_home: true}"
```
Watch debug for multiple ID labels (ID0, ID1, ID2...) — processed in that order.

### Check node list
```bash
ros2 node list
# Expected:
# /camera_publisher
# /servo_controller
# /ibvs_action_server
```

### Check all topics
```bash
ros2 topic list | grep visual
ros2 action list
```

---

## 8. Behavior Tree Integration

### Overview

The BT calls the action server and handles the result. Here is the expected BT structure:

```
Root (Sequence)
├── Fallback
│   ├── Sequence (FRONT inspection)
│   │   ├── InspectObjectsAction   ← calls /visual_inspection/inspect_objects
│   │   └── CheckSuccess           ← result.success == true
│   │
│   └── Sequence (BACK case)
│       ├── CheckObjectInBack      ← result.object_in_back == true
│       ├── Rotate180Action        ← Nav2 rotate 180°
│       └── InspectObjectsAction   ← retry after rotate
│
└── ReturnToBase                   ← navigate to charging station
```

### BT Leaf Node: `InspectObjectsAction`

This calls the action `/visual_inspection/inspect_objects`.

**Goal to send:**
```python
goal.max_objects = 0      # inspect all detected front objects
goal.return_home = True   # servo returns to 90,90 after done
```

**Result fields to read:**
```python
result.success            # bool: True if ≥1 object inspected
result.objects_inspected  # int: how many objects were inspected
result.objects_found      # int: total objects visible (front+back)
result.object_in_back     # bool: True → BT must rotate robot 180°
result.failed_reason      # str: "" | "no_detection" | "all_in_back" | ...
```

**Feedback fields (for BT monitoring/blackboard):**
```python
feedback.current_step     # "detecting" / "coarse" / "ibvs" / "capturing"
feedback.current_object   # which object number being processed
feedback.ibvs_error_px    # live IBVS convergence error in pixels
```

### BT Blackboard Keys (suggested)

| Key | Type | Source | Used By |
|-----|------|--------|---------|
| `inspection_success` | bool | result.success | CheckSuccess |
| `object_in_back` | bool | result.object_in_back | CheckObjectInBack |
| `objects_found` | int | result.objects_found | logging/reporting |
| `objects_inspected` | int | result.objects_inspected | reporting |
| `failed_reason` | str | result.failed_reason | error handling |
| `ibvs_error` | float | feedback.ibvs_error_px | monitoring |

### py_trees_ros Node Skeleton

```python
import py_trees
import py_trees_ros
from visual_inspection_interfaces.action import InspectObjects

class InspectObjectsAction(py_trees_ros.action_clients.FromBlackboard):
    """BT action leaf: calls the visual inspection pipeline."""

    def __init__(self, name='InspectObjects'):
        super().__init__(
            name=name,
            action_type=InspectObjects,
            action_name='/visual_inspection/inspect_objects',
            key_result='inspection_result',
            generate_feedback_message=self._feedback_cb
        )

    def _generate_goal(self, blackboard):
        goal = InspectObjects.Goal()
        goal.max_objects = 0     # inspect all front objects
        goal.return_home = True
        return goal

    def _feedback_cb(self, feedback):
        self.logger.info(
            f'[{feedback.current_step}] obj={feedback.current_object} '
            f'err={feedback.ibvs_error_px:.1f}px'
        )

    def update_blackboard_from_result(self, blackboard, result):
        blackboard.set('inspection_success',  result.success)
        blackboard.set('object_in_back',      result.object_in_back)
        blackboard.set('objects_found',       result.objects_found)
        blackboard.set('objects_inspected',   result.objects_inspected)
        blackboard.set('failed_reason',       result.failed_reason)
```

### Status Mapping for BT

| `result.success` | `result.object_in_back` | `result.failed_reason` | BT return |
|-----------------|------------------------|----------------------|-----------|
| `True` | `False` | `""` | **SUCCESS** |
| `True` | `True` | `""` | **SUCCESS** + trigger rotate |
| `False` | `True` | `"all_in_back"` | **FAILURE** + trigger rotate |
| `False` | `False` | `"no_detection"` | **FAILURE** |
| `False` | `False` | `"ibvs_timeout"` | **FAILURE** (retry optional) |

---

## 9. Configuration

### Calibration

Calibration maps Insta360 pixel (x, y) → servo (pan, tilt).

**Formula:** degree-4 polynomial (auto-generated by `tools/calibration_analysis_v2_script.py`)

**Recalibrate:**
```bash
cd ~/Documents/Visual_Inspection_ws
python3 tools/calib_fov.py   # collect points with W/A/S/D + SPACE
python3 tools/calibration_analysis_v2_script.py  # generates calibration_config.py
```

**Polynomial baked into:** `ibvs_action_server.py` → `calculate_pan()` / `calculate_tilt()`

### YOLO Model

| File | Location | Used for |
|------|----------|---------|
| `yolo11n.engine` | `~/Documents/Visual_Inspection_ws/weights/` | TensorRT (GPU, fast) |
| `yolo11n.pt` | same | PyTorch fallback (auto-used if .engine missing) |

**Export to TensorRT:**
```bash
python3 test_scripts/03_export_to_tensorrt.py
```

### Servo Hardware

- **Pan:** 0° (right) → 180° (left), home = 90°
- **Tilt:** `TILT_REVERSED=True` → actual command = `180 - tilt`
  - home 90° (horizontal), higher = down, lower = up
- **Arduino baud:** 9600
- **Command format:** `"tilt,pan\n"` via serial

---

## 10. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Camera not detected | Metadata device instead of capture | camera_node uses CAP_V4L2 + frame verify — check `ls /dev/insta360 /dev/logitech` |
| `no_detection` immediately | YOLO model path wrong | Check `weights/yolo11n.engine` exists |
| `all_in_back` result | Object is behind robot | BT should rotate 180° then retry |
| `logi_no_detection` | Coarse calibration off | Recalibrate with `tools/calib_fov.py` |
| `ibvs_timeout` | IBVS not converging | Tune `KP`, `IBVS_TOL_PX`, or check tilt direction |
| Debug shows "No Image" | Wrong topic in RViz2 | Subscribe to `/visual_inspection/debug` only |
| Servo moves opposite | Tilt or pan reversed | Toggle `TILT_REVERSED` in ibvs_action_server.py |
| SSH config broken | Accidental redirect to `~/.ssh/config` | `echo "" > ~/.ssh/config` |
| Two action servers | Old process still running | `pkill -f ibvs_action_server` |
| ByteTrack fails | `.yaml` file not found | Falls back to regular detect automatically |
