# Behaviour Tree Integration Guide
**Visual Inspection System — Jetson Orin Nano**
*Updated: 2026-04-20 — Migrated from ROS2 Action → ROS2 Service*

---

## Overview

The inspection pipeline is exposed as a **ROS2 Service** (not an Action):

| | Old (Action) | New (Service) ✅ |
|---|---|---|
| Interface | `InspectObjects.action` | `Inspect.srv` |
| Node | `ibvs_action_server` | `inspection_service` |
| Endpoint | `/visual_inspection/inspect_objects` | `/visual_inspection/inspect` |
| Feedback | Continuous (step/error/object) | None (blocks until done) |
| Cancellation | Supported | Not needed (BT controls timing) |
| BT integration | Complex (ActionClient) | Simple (ServiceClient) |

---

## Service Interface

**Service name**: `/visual_inspection/inspect`
**Service type**: `visual_inspection_interfaces/srv/Inspect`

### Request Fields

| Field | Type | Description |
|---|---|---|
| `target_object` | `string` | Object class to inspect (see table below) |
| `location_label` | `string` | Location name from BT (e.g. `"engine_room_A"`) |
| `max_objects` | `int32` | `0` = inspect all found, `N` = first N only |
| `return_home` | `bool` | `true` = servo returns to 90°,90° when done |

### Response Fields

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | `true` if ≥1 object inspected |
| `status` | `string` | `"ok"` / `"no_detection"` / `"ibvs_timeout"` / `"all_in_back"` / `"no_frames"` / `"busy"` |
| `objects_found` | `int32` | Objects detected on Insta360 |
| `objects_inspected` | `int32` | Objects successfully centred + captured |
| `object_in_back` | `bool` | `true` → BT should rotate robot 180° |
| `image_paths` | `string[]` | Absolute paths to all saved images |
| `info` | `string` | Human-readable summary |

---

## Target Object Classes

| `target_object` | Mode | Pipeline |
|---|---|---|
| `"fire_extinguisher"` | **Full IBVS** | Insta360 detect → coarse pan-tilt → Logitech IBVS → 3 images |
| `"extinguisher"` | **Full IBVS** | Same (alias) |
| `"door"` | **Full IBVS** | Same |
| `"person"` | **Full IBVS** | Same |
| `"gauge"` | **Full IBVS + Sweep** | Same + serpentine sweep if Insta360 misses |
| `"unknown"` | **Overview only** | Servo home → Insta360 raw + 1 Logitech image |
| `"main_cylinder"` | **Overview only** | Same |
| `""` or `"any"` | **All classes** | Detect all YOLO classes, inspect each |

### YOLO Model Classes (yolov26s.engine)

| ID | YOLO Name | Accepted `target_object` values |
|---|---|---|
| 0 | `door` | `"door"` |
| 1 | `extinguisher` | `"fire_extinguisher"`, `"extinguisher"` |
| 2 | `gauge` | `"gauge"`, `"pressure_gauge"` |
| 3 | `person` | `"person"`, `"people"` |

*`unknown` and `main_cylinder` are NOT YOLO classes → overview-only mode.*

### Confidence Thresholds (per class)

| Class | Threshold | Reason |
|---|---|---|
| extinguisher | 0.5 | High confidence needed |
| door | 0.5 | |
| person | 0.5 | |
| gauge | **0.3** | Gauge is visually weak — lower to avoid missing it |

---

## Status Values

| `status` | BT Action |
|---|---|
| `"ok"` | Success — continue |
| `"no_detection"` | Nothing found → navigate to next position |
| `"ibvs_timeout"` | IBVS didn't converge → move closer |
| `"all_in_back"` | Objects in back half → rotate robot 180° |
| `"no_frames"` | Camera failure |
| `"busy"` | Another inspection running — retry |

---

## Gauge Special Case: Sweep Scan

If gauge is **not detected** on Insta360 after 8s:
1. Servo does a **serpentine sweep**: tilt=[20, 50, 80]°, pan=[20→160]° (10° steps)
2. Logitech checks for gauge at every position
3. If found → **IBVS fires immediately** from that position (no coarse step)
4. If not found after full sweep → `status="no_detection"`

---

## Object Numbering (Multiple Instances)

When multiple same-class objects are detected (e.g. 2 gauges):
- Numbered by **confidence** (highest = #1)
- Inspected in order: #1 → #2 → ...
- Each saved to its own folder with `metadata.json`

### Capture Folder Structure
```
captures/
├── inspection/20260420_120000/        ← full IBVS session
│   ├── extinguisher/
│   │   └── instance_1/
│   │       ├── img_01_conf0.85.jpg   ← 3 Logitech ROI images
│   │       ├── img_02_conf0.85.jpg
│   │       ├── img_03_conf0.85.jpg
│   │       ├── overview_engine_room_A_01.jpg  ← Insta360 raw
│   │       └── metadata.json
│   └── gauge/
│       ├── instance_1/               ← highest confidence
│       └── instance_2/               ← lower confidence
└── overview/20260420_120100/         ← overview-only session
    ├── overview_unknown_01.jpg       ← Insta360 raw
    └── unknown/instance_1/
        ├── img_01_conf0.00.jpg       ← 1 Logitech from home
        └── metadata.json
```

---

## BT Leaf Node — Python (rclpy)

```python
from visual_inspection_interfaces.srv import Inspect

# Create client (once, in BT node __init__)
self.client = self.create_client(Inspect, '/visual_inspection/inspect')

# Call service (in BT tick)
req = Inspect.Request()
req.target_object  = "fire_extinguisher"   # from BT blackboard
req.location_label = "engine_room_A"
req.max_objects    = 0                      # inspect all found
req.return_home    = True

future = self.client.call_async(req)
rclpy.spin_until_future_complete(self, future)
res = future.result()

if res.success:
    # res.image_paths → list of saved images
    # res.info        → summary string
    return NodeStatus.SUCCESS
elif res.object_in_back:
    # Signal BT to rotate robot 180°
    return NodeStatus.FAILURE
else:
    return NodeStatus.FAILURE
```

---

## BT XML (BehaviorTree.CPP)

```xml
<Action ID="InspectObject"
        service_name="/visual_inspection/inspect"
        target_object="{target_object}"
        location_label="{location_label}"
        max_objects="0"
        return_home="true" />
```

---

## Running the Service

```bash
# ── Every terminal needs this ──────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# Terminal 1 — Cameras
ros2 run visual_inspection_ros camera_node

# Terminal 2 — Servo
ros2 run visual_inspection_ros servo_node

# Terminal 3 — Inspection service (replaces ibvs_action_server)
ros2 run visual_inspection_ros inspection_service
```

---

## Test Commands (Simulate BT — 7 classes)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# 1. Fire Extinguisher
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object fire_extinguisher

# 2. Door
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object door

# 3. Person
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object person

# 4. Gauge (with sweep fallback if not found on Insta360)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object gauge

# 5. Unknown (overview only — Insta360 + 1 Logitech at home)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object unknown

# 6. Main Cylinder (overview only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object main_cylinder

# 7. All classes
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py
```

---

## Monitoring Topics (Optional — for debugging only)

| Topic | Type | Description |
|---|---|---|
| `/visual_inspection/debug` | `Image` | Side-by-side Insta360 + Logitech (RViz2) |
| `/visual_inspection/status` | `String` | `IDLE` / `DETECTING` / `COARSE` / `IBVS` / `CAPTURING` / `OVERVIEW` / `SWEEP` |
| `/visual_inspection/ibvs_error` | `Point` | x/y pixel error during IBVS |
| `/visual_inspection/detections` | `String` | JSON of detected objects |
| `/servo/pan_tilt` | `Int16MultiArray` | `[tilt, pan]` servo angles |

> **Note:** These topics are optional — only subscribe when debugging. During real deployment, skip RViz2 to save GPU/CPU resources.

---

## File Locations

| File | Path |
|---|---|
| Service definition | `inspection_ws/visual_inspection_interfaces/srv/Inspect.srv` |
| Service server | `inspection_ws/visual_inspection_ros/visual_inspection_ros/inspection_service.py` |
| Action server (kept) | `inspection_ws/visual_inspection_ros/visual_inspection_ros/ibvs_action_server.py` |
| Test client | `test_scripts/test_inspection_service.py` |
| This doc | `inspection_ws/visual_inspection_ros/docs/BT_INTEGRATION_GUIDE.md` |
