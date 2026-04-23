# Behaviour Tree Integration Guide
**Visual Inspection System — Jetson Orin Nano**

---

## 1. Overview
The visual inspection pipeline exposes a single **ROS2 Service** for the Behavior Tree (BT) to interact with. 

**IMPORTANT FOR BT DEVELOPERS:** The visual inspection module is a ROS2 **Service** (not an Action). The BT simply triggers the service and waits until the inspection completes. 

**Service Name**: `/visual_inspection/inspect`
**Service Type**: `visual_inspection_interfaces/srv/Inspect`

---

## 2. Input: From Behavior Tree to Inspection System

When the Behavior Tree reaches the Inspection Node, it must send the following request fields to the service:

| Input Field | Type | What BT should provide |
|---|---|---|
| `target_object` | `string` | What to inspect: `"fire_extinguisher"`, `"door"`, `"person"`, `"gauge"`, or `"unknown"` |
| `location_label` | `string` | The current location on the map (e.g. `"engine_room_A"`). Used for naming saved folders. |
| `max_objects` | `int32` | How many to inspect. Send `0` to inspect ALL objects of that type found in the area. |
| `return_home` | `bool` | Send `true` so the camera resets its pan/tilt to center after finishing. |

---

## 3. Output: From Inspection System back to Behavior Tree

When the inspection is complete, the Jetson will return the following response to the BT. 

**IMPORTANT:** The images themselves and their metadata (JSON files containing confidence, convergence, errors, etc.) are saved on the Jetson's disk. The BT receives the **paths** to these images so it knows exactly where to find them and their metadata.

| Output Field | Type | What BT receives |
|---|---|---|
| `success` | `bool` | `true` if at least 1 object was successfully inspected and captured. |
| `status` | `string` | Reason for completion: `"ok"`, `"no_detection"`, `"ibvs_timeout"`, `"all_in_back"`, `"no_frames"`, or `"busy"`. |
| `objects_found` | `int32` | How many were spotted by the wide Insta360 camera. |
| `objects_inspected` | `int32` | How many were actually focused on and successfully captured by the Logitech camera. |
| `object_in_back` | `bool` | `true` means the object is behind the robot. The BT should read this and trigger a 180° rotation of the robot body, then try again. |
| `image_paths` | `string[]` | **CRITICAL:** A list of absolute paths to the saved images. The BT or Server can use these paths to read the images. Every image folder also contains a `metadata.json` file with all the metrics (confidence, time taken, etc). |
| `info` | `string` | Human-readable log summary string. |

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
| Test client | `test_scripts/test_inspection_service.py` |
| This doc | `inspection_ws/visual_inspection_ros/docs/BT_INTEGRATION_GUIDE.md` |
