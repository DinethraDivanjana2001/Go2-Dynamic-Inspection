# Weekly Standup Report — Week of 4 March to 9 March 2026

**Project:** Visual Inspection System — Jetson Orin Nano  
**Team Member:** Dinethra  
**Report Date:** 2026-03-09  
**Period:** Tuesday 4 March → Sunday 9 March 2026

---

## Summary

This week focused on converting the working Python IBVS pipeline into a full ROS2 action server, adding multi-object tracking, MQTT image delivery to ThingsBoard, Behaviour Tree leaf nodes, and comprehensive documentation for team handover.

---

## ✅ What Was Completed This Week

### 1. ROS2 Action Server — Core Pipeline (`ibvs_action_server.py`)

Previously the inspection pipeline was a standalone Python script (`ibvs_pipeline.py`). This week it was fully converted into a ROS2 Action Server with proper feedback, result, and cancel handling.

**Pipeline stages inside the action server:**
```
Goal received
  → Stage 1 DETECTING: YOLO on Insta360 (up to 20s)
  → Stage 2 COARSE:    polynomial formula → servo angles
  → Stage 3 IBVS:      PID loop on Logitech camera (up to 40s, tol=10px)
  → Stage 4 CAPTURE:   10s autofocus wait → 4 Logitech ROI + 1 Insta360 overview
  → MQTT:              send 5 images to ThingsBoard
  → Result:            success/failure + counts + BT signals
```

### 2. ByteTrack Multi-Object Tracking

- Integrated `model.track(persist=True, tracker='bytetrack.yaml')` from Ultralytics
- Each object assigned a **stable track ID** across frames
- Objects processed in track ID order for consistent inspection sequence
- Multiple same-class objects handled (e.g., 2 fire extinguishers → instance_1, instance_2)

### 3. Front / Back Zone Detection

- Insta360 frame split at `FRONT_Y_MAX = 200px`
  - `cy < 200px` → **FRONT** (green box) → full IBVS inspection
  - `cy > 200px` → **BACK** (orange box) → signal BT to rotate robot 180°
- Result field `object_in_back=True` tells BT to rotate and retry

### 4. Action Interface Updated (`InspectObjects.action`)

New fields added this week:

**Goal (new):**
```
string location_label   # "unknown" or known label from BT e.g. "gauge_room_A"
bool   overview_only    # true = only capture 360 snapshots, skip IBVS
int32  overview_count   # how many Insta360 images (default 2)
```

**Result (new):**
```
int32  objects_found    # total detected on Insta360 (front + back)
bool   object_in_back   # true → BT must rotate robot 180°
```

### 5. MQTT Integration — ThingsBoard Cloud

- **Platform:** ThingsBoard (`demo.thingsboard.io:1883`)
- **Auth:** ThingsBoard token-based auth — `username = access_token`, `password = ""`
- **Config file:** `config/mqtt_config.yaml` (separate from code)
- **Per image payload:**
  ```json
  {
    "session":     "20260309_120000",
    "object_id":   1,
    "class_name":  "fire_extinguisher",
    "image_idx":   1,
    "total":       5,
    "timestamp":   1741500000.0,
    "image_b64":   "..."
  }
  ```
- **KEEP_LOCAL = True** → images always saved locally AND sent via MQTT (dataset mode)
- **Delete after MQTT** → can be enabled by setting `KEEP_LOCAL = False`

### 6. Local Capture Folder Structure

```
captures/
├── inspection/                    ← Full IBVS ROI captures
│   └── 20260309_120000/           ← Session ID = timestamp of inspection
│       └── fire_extinguisher/
│           └── instance_1/
│               ├── img_01.jpg     ← Logitech close-up (focused, after 10s wait)
│               ├── img_02.jpg
│               ├── img_03.jpg
│               ├── img_04.jpg
│               └── overview_01.jpg ← Insta360 with YOLO bounding boxes
│
└── overview/                      ← Insta360-only snapshots for VLM
    └── 20260309_121000/
        ├── gauge_room_A_01.jpg
        └── gauge_room_A_02.jpg
```

### 7. Object Class Name Propagation

- YOLO class name extracted from `results.names[box.cls[0]]`  
  - E.g., `fire_extinguisher`, `gauge`, `door`
- Shown in bounding box label: `[gauge] ID38 0.82`
- Passed through capture → folder name → MQTT payload
- Shown in Logitech debug view with IBVS arrow

### 8. Three New ROS2 Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/visual_inspection/status` | `std_msgs/String` | Live mode: `IDLE/DETECTING/COARSE/IBVS/CAPTURING/OVERVIEW` |
| `/visual_inspection/ibvs_error` | `geometry_msgs/Point` | Live pixel error (x, y, magnitude) |
| `/visual_inspection/detections` | `std_msgs/String` | JSON with all detected objects, class, conf, track_id, zone |

### 9. 10-Second Autofocus Wait Before Capture

- After IBVS converges (error < 10px), servo stops
- System waits **10 seconds** for Logitech camera to autofocus on object
- Then captures 4 images → much sharper, readable gauge readings

### 10. Behaviour Tree Nodes Package (`bt_nodes/`)

Created `visual_inspection_ros/bt_nodes/inspection_bt_nodes.py` with three py_trees leaf nodes:

#### `InspectObjectsAction` (Action leaf)
- Calls `/visual_inspection/inspect_objects` action
- Writes to BT blackboard: `inspection_success`, `objects_found`, `objects_inspected`, `object_in_back`, `failed_reason`, `ibvs_error_px`
- Status: `RUNNING` while inspecting, `SUCCESS` if done, `FAILURE` if error

#### `CaptureOverviewAction` (Action leaf)
- Sends `overview_only=True` goal
- Just captures Insta360 snapshots for VLM, no IBVS
- BT uses this when arriving at a location to get a 360° view before inspection
- Writes: `overview_success`

#### `CheckObjectInBack` (Condition leaf)
- Reads `object_in_back` from BT blackboard
- `SUCCESS` → BT should issue RotateRobot(180°) command then retry
- `FAILURE` → no back objects

#### BT Decision Table
| `success` | `object_in_back` | `failed_reason` | BT Action |
|-----------|-----------------|-----------------|-----------|
| True | False | `""` | All done, continue mission |
| True | True | `""` | Front done, rotate 180° and reinspect back |
| False | True | `"all_in_back"` | Rotate 180° then retry |
| False | False | `"no_detection"` | Move to next position |
| False | False | `"ibvs_timeout"` | Skip, log alert |

### 11. Overview-Only Mode (for VLM pipeline)

BT can request `overview_only=True` with a known `location_label`:
- Server **skips IBVS entirely** (no 20s search, no coarse, no fine)
- Captures N Insta360 images with YOLO boxes drawn
- Saves to `captures/overview/SESSION/` (separate from inspection captures)
- MQTT sends with `location_label` as class_name
- Later: images will be sent to VLM for object identification

### 12. Test Script — Replaces Behaviour Tree (`test_full_pipeline.py`)

Since the BT is not connected yet, created a standalone test script that:
- Sends goals to action server
- Monitors feedback (live IBVS error bar in terminal)
- Handles all result cases (success / back-detected / multi-object / timeout)
- Auto-retries if `object_in_back=True` (simulates robot rotation)
- Loops continuously until Ctrl+C

### 13. Tilt Servo Reverse Fix

- Added `TILT_REVERSED = True` flag
- When True: sends `180 - tilt` to servo (corrects physical mounting)
- Configurable without code change

### 14. Combined Debug Topic (`/visual_inspection/debug`)

- Side-by-side Insta360 (left) + Logitech (right) in one RViz2 image
- Overlays: bounding boxes with class name + track ID + confidence
- IBVS arrow (blue) showing direction of correction
- Mode text (DETECTING / IBVS / CAPTURING)
- FPS counter
- Front/back boundary line on Insta360 view

---

## 🚧 Blockers Encountered This Week

### Blocker 1: `ValueError: too many values to unpack` in IBVS loop
**Root cause:** `_detect_insta()` returns 4 values `(front, back, frame, debug)` but one call inside `_ibvs()` only unpacked 3.  
**Fix:** Changed `_, _, insta_dbg = ...` → `_, _, _, insta_dbg = ...`  
**Time lost:** ~1 hour (many failed runs before found in terminal log)

### Blocker 2: `UnboundLocalError: front_dets referenced before assignment`
**Root cause:** During refactoring to add `overview_only` mode, the `_search_insta()` call was accidentally removed from the full inspection path.  
**Fix:** Restored `front_dets, back_dets = self._search_insta(goal_handle)` in FULL INSPECTION MODE section.  
**Time lost:** ~30 min

### Blocker 3: Colcon build fails on laptop with `.venv` active
**Root cause:** `.venv` overrides system Python path → `rosidl_adapter` picks up venv Python → missing `em` module.  
**Rule:** Never activate `.venv` before `colcon build`. Build only on Jetson, not laptop.  
**Fix:** Only use Jetson for ROS2 builds. Laptop only for editing + SCP.

### Blocker 4: MQTT `wait_for_publish(timeout=5)` crashes on Jetson
**Root cause:** Jetson has `paho-mqtt 1.x` which doesn't support `timeout` argument in `wait_for_publish()`.  
**Fix:** Added try/except to fall back to `wait_for_publish()` without timeout.

### Blocker 5: MQTT not sending from pipeline (silent failure)
**Root cause:** `paho-mqtt` was installed in `.venv` (Python 3.10 venv) but ROS2 action server uses **system Python**. Import fails silently (caught by `except Exception`).  
**Fix:** `pip3 install paho-mqtt` (system pip, without venv active).  
**Status:** Fixed at end of week

### Blocker 6: `colcon build` CMake JSON error (corrupted cache)
**Root cause:** Previous failed build left corrupt CMake cache files.  
**Fix:**
```bash
rm -rf build/visual_inspection_interfaces install/visual_inspection_interfaces
colcon build --packages-select visual_inspection_interfaces
```

### Blocker 7: SCP fails `No such file or directory` for new directories
**Root cause:** Target directory doesn't exist on Jetson (e.g., `test_scripts/`, `bt_nodes/`).  
**Fix:** Always `mkdir -p <dir>` on Jetson before SCP.

---

## 📊 Testing Results

| Test | Result | Notes |
|------|--------|-------|
| Single object detection + IBVS | ✅ Passes | Error converges to 5-10px consistently |
| Multi-object (2 objects) | ✅ Passes | Processed in track ID order |
| No object (timeout) | ✅ Returns `no_detection` | 20s timeout correct |
| Back-side object | ✅ Returns `object_in_back=True` | Needs BT rotation test |
| MQTT basic telemetry | ✅ Visible on ThingsBoard | `message`, `session`, `timestamp` keys |
| MQTT image payload | 🔄 In progress | paho-mqtt system install just fixed |
| BT nodes import | ✅ Code created | Not yet integrated with main BT |
| Overview-only mode | ✅ Code complete | Not yet tested end-to-end |

---

## 🔜 Next Steps

### Immediate (this week → next week)
1. **Verify MQTT image delivery** — now that paho-mqtt is installed on system Python, run full pipeline and confirm `image_b64` appears in ThingsBoard
2. **Test overview-only mode** — `overview_only=True` goal, check `captures/overview/` folder
3. **Test back-side detection** — place object in back zone, confirm BT signal works

### Short-term
4. **Integrate BT nodes with Ravith's main BT** — share `inspection_bt_nodes.py`, agree on blackboard key names
5. **Dataset collection** — use pipeline to systematically capture ROIs for 3 classes:
   - `door` — capture 50+ instances
   - `gauge` — capture 50+ instances (make sure readable after autofocus)
   - `fire_extinguisher` — capture 50+ instances

### Medium-term
6. **VLM integration** — pass `overview/` images to VLM for object identification when label is `unknown`
7. **Navigation integration** — connect BT to robot navigation to drive to inspection locations

---

## 🧠 Knowledge Transfer — Understanding the System

> This section explains the architecture for anyone joining the project.

### Why ROS2 Action Server?
ROS2 Actions (not regular topics or services) are used because:
- Inspection takes 20-60 seconds (long-running)
- Need **feedback** while running (live IBVS error)
- Need ability to **cancel** mid-inspection
- Need structured **result** with multiple fields

### Why Two Cameras?
- **Insta360 (wide-angle 360°):** Used for initial detection. Wide FOV means object always visible. Not good for close-up inspection.
- **Logitech (narrow FOV):** Used for IBVS centering and final capture. Sharp, focused images of objects.

### Why IBVS (Image-Based Visual Servoing)?
Using pixel coordinates directly in the control loop instead of converting to 3D coordinates:
- Simpler (no depth sensor needed)
- More accurate at close range
- Error = pixel distance from object center to frame center
- Servos move until error < 10px

### Why ByteTrack?
Multiple identical objects (e.g., 3 fire extinguishers) need stable IDs so:
- We don't inspect the same object twice
- We process them in consistent order
- ID stays stable even if object briefly disappears

### Why Two Capture Folders?
- **`inspection/`** — Full IBVS pipeline result. Contains sharp close-ups. Used for: quality inspection, defect detection, dataset collection.
- **`overview/`** — Quick Insta360 snapshot. Contains scene context. Used for: VLM object identification when label is unknown, mission logging.

### MQTT vs ROS2 Topics — Why Both?
- **ROS2 topics** = internal robot communication (fast, local, ephemeral)
- **MQTT** = external cloud delivery (ThingsBoard, persistent, accessible from anywhere)
Images and results need to be sent to the cloud for monitoring/logging = MQTT.

### ThingsBoard Access
- URL: https://demo.thingsboard.io
- Device: "inspection"
- Tab: Latest Telemetry
- Data arrives as JSON key-value pairs
- `image_b64` key contains the base64-encoded JPEG image

---

*Report prepared by: Dinethra | 2026-03-09*
