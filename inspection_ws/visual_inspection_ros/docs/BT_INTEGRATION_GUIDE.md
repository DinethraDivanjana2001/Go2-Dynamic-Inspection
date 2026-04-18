# Behaviour Tree Integration Guide
**Visual Inspection System — Jetson Orin Nano**

---

## Action Interface

**Action name**: `/visual_inspection/inspect_objects`
**Action type**: `visual_inspection_interfaces/action/InspectObjects`

---

## Goal Fields

| Field | Type | Description |
|---|---|---|
| `target_object` | `string` | **Object to inspect** (see table below) |
| `max_objects` | `int32` | `0` = inspect all, `N` = first N only |
| `return_home` | `bool` | `true` = servo returns to 90°,90° after done |
| `location_label` | `string` | Location name from BT (e.g., `"engine_room_A"`) |
| `overview_only` | `bool` | `true` = force overview mode (auto-set for unknown/main_cylinder) |
| `overview_count` | `int32` | Number of Insta360 snapshots (default: 2) |

## Target Objects

| `target_object` value | Mode | What happens |
|---|---|---|
| `"fire_extinguisher"` | **Full IBVS** | Insta360 detect → coarse → Logitech IBVS → capture 3 images |
| `"extinguisher"` | **Full IBVS** | Same (alias) |
| `"door"` | **Full IBVS** | Same |
| `"person"` | **Full IBVS** | Same |
| `"gauge"` | **Full IBVS** | Same |
| `"unknown"` | **Overview only** | Servo home → capture Insta360 + Logitech (no IBVS) |
| `"main_cylinder"` | **Overview only** | Servo home → capture Insta360 + Logitech (no IBVS) |
| `""` or `"any"` | **Full IBVS (all)** | Detects ALL YOLO classes, inspects each |

### Full IBVS Mode (fire_extinguisher, door, person, gauge)
1. Insta360 detects objects with YOLO + ByteTrack
2. Filters to requested class only
3. Numbers multiple objects by confidence (#1 = highest)
4. For each object: coarse pan-tilt → IBVS fine centering → wait for focus → capture 3 images
5. Stores confidence score in filename + metadata.json

### Overview-Only Mode (unknown, main_cylinder)
1. Servo returns to home (90°, 90°)
2. Captures Insta360 overview images (with any YOLO detections drawn)
3. Captures Logitech images from home position
4. **No pan-tilt movement, no IBVS** — we can't identify these with YOLO

### Aliases (also accepted)
- `"fire_extinguisher"`, `"fire extinguisher"`, `"extinguisher"` → YOLO class `extinguisher`
- `"people"` → YOLO class `person`
- `"pressure_gauge"` → YOLO class `gauge`

---

## YOLO Model Classes (yolov26s.engine)

| Class ID | YOLO Class Name | BT Names |
|---|---|---|
| 0 | `door` | `"door"` |
| 1 | `extinguisher` | `"fire_extinguisher"`, `"extinguisher"` |
| 2 | `gauge` | `"gauge"`, `"pressure_gauge"` |
| 3 | `person` | `"person"`, `"people"` |

*`unknown` and `main_cylinder` are NOT YOLO classes — they trigger overview-only mode.*

---

## Result Fields

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | At least one object inspected successfully |
| `objects_inspected` | `int32` | Number of objects fully inspected |
| `objects_found` | `int32` | Total objects detected on Insta360 |
| `object_in_back` | `bool` | Objects in Insta360 back half → **BT should rotate 180°** |
| `failed_reason` | `string` | See table below |

### Failed Reasons

| Value | BT Action |
|---|---|
| `""` | Success — continue |
| `"no_detection"` | No objects found → navigate to next position |
| `"ibvs_timeout"` | IBVS didn't converge → move closer or recalibrate |
| `"logi_no_detection"` | Object not in Logitech after coarse → recalibrate |
| `"all_in_back"` | All objects in back half → rotate robot 180° |
| `"no_frames"` | Camera capture failed |

---

## Feedback Fields

| Field | Type | Description |
|---|---|---|
| `current_step` | `string` | `"detecting"` / `"coarse"` / `"ibvs"` / `"capturing"` / `"overview"` |
| `current_object` | `int32` | Which object number (1, 2, 3...) |
| `ibvs_error_px` | `float32` | Current pixel error during IBVS (converges to <10px) |

---

## Object Numbering (Multiple Same-Class Objects)

When YOLO detects multiple objects of the same class (e.g., 2 gauges):
- Objects are numbered by **confidence score** (highest = #1)
- Each is inspected separately: IBVS centres on #1 → capture → then #2 → capture
- ByteTrack maintains consistent IDs across frames
- Example log: `gauge #1 (conf=0.92)`, `gauge #2 (conf=0.78)`

### Capture Folder Structure
```
captures/
├── inspection/20260418_180045/          # full IBVS session
│   ├── extinguisher/
│   │   └── instance_1/
│   │       ├── img_01_conf0.85.jpg      # 3 Logitech ROI images
│   │       ├── img_02_conf0.85.jpg
│   │       ├── img_03_conf0.85.jpg
│   │       ├── overview_engine_room_01.jpg  # Insta360 overview
│   │       └── metadata.json            # {class, instance_id, confidence, ...}
│   └── gauge/
│       ├── instance_1/                  # highest confidence gauge
│       │   ├── img_01_conf0.92.jpg
│       │   ├── img_02_conf0.92.jpg
│       │   ├── img_03_conf0.92.jpg
│       │   └── metadata.json
│       └── instance_2/                  # lower confidence gauge
│           ├── img_01_conf0.78.jpg
│           ├── img_02_conf0.78.jpg
│           ├── img_03_conf0.78.jpg
│           └── metadata.json
└── overview/20260418_181500/            # overview-only session
    ├── overview_unknown_01.jpg          # Insta360 overview
    ├── overview_unknown_02.jpg
    ├── unknown/
    │   └── instance_1/
    │       ├── img_01_conf0.00.jpg      # 3 Logitech from home position
    │       ├── img_02_conf0.00.jpg
    │       ├── img_03_conf0.00.jpg
    │       └── metadata.json
```

---

## Test Commands (Simulates BT Goals)

All commands need these running first:
```bash
# Terminal 1 — Camera
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node

# Terminal 2 — Servo
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node

# Terminal 3 — Action Server
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server

# Terminal 4 — RViz2 (optional)
source /opt/ros/humble/setup.bash
rviz2
```

### Terminal 5 — Run inspection (7 commands):

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# 1. Fire Extinguisher (full IBVS — filters to extinguisher only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object fire_extinguisher --once

# 2. Door (full IBVS — filters to door only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object door --once

# 3. Person (full IBVS — filters to person only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object person --once

# 4. Gauge (full IBVS — filters to gauge only, inspects each gauge)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object gauge --once

# 5. Unknown (overview only — servo home, captures Insta360 + Logitech)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object unknown --once

# 6. Main Cylinder (overview only — servo home, captures Insta360 + Logitech)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --object main_cylinder --once

# 7. Any/All (full IBVS — detects ALL classes, inspects each object found)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py --once
```

---

## BT Leaf Node Examples

### Python (rclpy)
```python
goal = InspectObjects.Goal()
goal.target_object  = "fire_extinguisher"   # from BT blackboard
goal.max_objects    = 0                      # inspect all found
goal.return_home    = True
goal.location_label = "engine_room_A"

future = client.send_goal_async(goal)
```

### BT XML (BehaviorTree.CPP)
```xml
<Action ID="InspectObject"
        target_object="{target_object}"
        max_objects="0"
        return_home="true"
        location_label="{location_label}"
        overview_only="false"
        overview_count="2" />
```

---

## ROS2 Topics for Monitoring

| Topic | Type | Description |
|---|---|---|
| `/visual_inspection/debug` | `Image` | Side-by-side Insta360 + Logitech (subscribe in RViz2) |
| `/visual_inspection/status` | `String` | Current state: IDLE/DETECTING/COARSE/IBVS/CAPTURING/OVERVIEW |
| `/visual_inspection/ibvs_error` | `Point` | x=pan_error, y=tilt_error in pixels |
| `/visual_inspection/detections` | `String` | JSON of detected objects |
| `/servo/pan_tilt` | `Int16MultiArray` | Current servo angles [tilt, pan] |
