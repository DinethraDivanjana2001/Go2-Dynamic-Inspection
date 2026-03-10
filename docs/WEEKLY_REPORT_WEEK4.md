# Weekly Progress Report - Visual Inspection System (Week 4)
# Period: 4 March – 10 March 2026

---

## Slide 1: Last Week Recap — Pipeline Running, ROS2 Conversion Complete

○ IBVS pipeline fully working: TensorRT 30 FPS, IBVS converging at 5-10px error

○ Both cameras auto-detected via udev symlinks (`/dev/insta360`, `/dev/logitech`, `/dev/arduino`)

○ Pan-tilt servo controlled via Arduino over serial

○ Started wrapping standalone Python pipeline into ROS2 nodes

○ This week: completed full ROS2 integration, Behaviour Tree interface, MQTT cloud delivery

**Script for Slide 1:**
"Last week the standalone Python pipeline was working reliably — TensorRT giving 30 FPS, IBVS converging to within 10 pixels consistently. This week the goal was to wrap all of that into ROS2 so it can be integrated with the Behaviour Tree that the navigation team is building. I also added MQTT image delivery to ThingsBoard cloud. By the end of this week, the complete Jetson side implementation is done — the Behaviour Tree person just needs to connect our nodes using the leaf node classes we've provided."

---

## Slide 2: ROS2 Package Structure

○ Created two ROS2 packages in `inspection_ws/`:

```
inspection_ws/
├── visual_inspection_interfaces/     ← Custom ROS2 message/action definitions
│   └── action/
│       └── InspectObjects.action     ← Action interface (goal/result/feedback)
│
└── visual_inspection_ros/            ← All ROS2 nodes
    ├── camera_node.py                ← Publishes both camera feeds
    ├── servo_node.py                 ← Arduino servo control via ROS2 topic
    ├── ibvs_action_server.py         ← Full inspection pipeline as Action Server
    └── bt_nodes/
        └── inspection_bt_nodes.py   ← BT leaf nodes for Ravith to use
```

○ Packages built with `colcon build` on Jetson Orin Nano (ROS2 Humble)

○ All code version-controlled on GitHub with full documentation

**Script for Slide 2:**
"The ROS2 workspace has two packages. The first one, visual_inspection_interfaces, only contains the action definition file — this defines the data structure for communicating between the Behaviour Tree and our inspection server. The second one, visual_inspection_ros, contains all the actual node code. There are three nodes: the camera publisher, the servo controller, and the main IBVS action server which contains the entire inspection pipeline. We also added a bt_nodes folder specifically for the Behaviour Tree integration — this is what Ravith needs."

---

## Slide 3: Node 1 — Camera Publisher (`camera_node.py`)

○ Publishes both cameras as standard ROS2 image topics at ~30 Hz

○ **Camera access method:** OpenCV `cv2.VideoCapture` with V4L2 backend (not GStreamer)

```
Why V4L2 and not GStreamer?
  - GStreamer requires additional pipeline string configuration
  - V4L2 works directly from cv2.VideoCapture with udev device symlinks
  - Both cameras detected via /dev/insta360 and /dev/logitech permanently
  - Reliable, simple, sufficient for 30 FPS output
```

○ Topics published:

| Topic | Resolution | Hz |
|-------|------------|-----|
| `/visual_inspection/insta360/image_raw` | 640×480 (cropped from 3840×1920) | ~30 |
| `/visual_inspection/logitech/image_raw` | 640×480 | ~30 |

○ Also publishes combined debug image (side-by-side) to `/visual_inspection/debug`

**Script for Slide 3:**
"The camera node is straightforward. It reads both cameras using OpenCV's VideoCapture with the V4L2 backend. We're not using GStreamer — that would require a custom pipeline string and additional configuration. V4L2 works directly with the udev symlinks we set up in week 3, so the camera node always finds the right device. Both cameras publish at 30 Hz as standard ROS2 sensor_msgs/Image topics. The ibvs_action_server subscribes to these and does all the processing — the camera node itself has no intelligence."

---

## Slide 4: Node 2 — Servo Controller (`servo_node.py`)

○ Subscribes to `/servo/pan_tilt` — `std_msgs/Int16MultiArray` with `[tilt, pan]`

○ Writes to Arduino over serial (`/dev/arduino`) at 115200 baud

○ Arduino reads `tilt,pan\n` format and moves PWM servos accordingly

○ **Tilt reversal:** Hardware-level flag `TILT_REVERSED = True` flips direction
  - Physical servo is mounted reversed → sends `180 - tilt` to correct

○ Servo home position: pan=90, tilt=90 (looking straight forward)

○ `return_home=True` in goal → servos return to 90,90 after inspection done

**Script for Slide 4:**
"The servo node is a simple bridge — it takes a ROS2 topic message and converts it to a serial command for the Arduino. The Arduino then moves the physical pan-tilt servos using PWM. One thing I had to fix this week was the tilt direction — the physical servo was mounted in reverse, so we added a flag that flips the tilt value. There's also a return_home feature — when the BT sends a goal with return_home=true, after the full inspection the servos automatically come back to center position."

---

## Slide 5: Node 3 — IBVS Action Server (Full Pipeline)

**Why a ROS2 Action and not a Service or Topic?**

```
ROS2 Topic   → fire-and-forget, no response, no feedback
ROS2 Service → request/response, but blocks, no streaming feedback  
ROS2 Action  → long-running, streaming feedback, cancellable ✓
```

Inspection takes 20-60 seconds → Action is the correct choice.

**Full pipeline inside the action server:**

```
┌──────────────────────────────────────────────────────┐
│  GOAL received from BT                               │
│                                                      │
│  Stage 1: DETECTING (up to 20s)                      │
│    YOLO11n TensorRT on Insta360 frame (30 FPS)       │
│    ByteTrack → stable IDs for multiple objects       │
│    Split by y-position:                              │
│      cy < 200px = FRONT zone (green box)             │
│      cy > 200px = BACK zone  (orange box)            │
│                                                      │
│  If only BACK objects → signal BT: object_in_back    │
│                                                      │
│  Stage 2: COARSE (per front object)                  │
│    Degree-4 polynomial: (cx, cy) → (pan, tilt)       │
│    Publish servo command → servo moves               │
│    Wait 2s for servo to physically reach position    │
│                                                      │
│  Stage 3: IBVS (per front object, up to 40s)         │
│    YOLO on Logitech → PID servo loop                 │
│    error = pixel distance from object to frame center│
│    Terminate when error < 10px                       │
│    Feedback to BT: ibvs_error_px every tick          │
│                                                      │
│  Stage 4: CAPTURE                                    │
│    Wait 10s for camera autofocus                     │
│    4 Logitech ROI images (sharp, focused)            │
│    1 Insta360 overview image (with YOLO boxes)       │
│    Save to local folder + send via MQTT              │
│                                                      │
│  RESULT to BT: success, objects_found, object_in_back│
└──────────────────────────────────────────────────────┘
```

**Script for Slide 5:**
"The IBVS action server is the heart of the system. It runs the entire four-stage inspection pipeline. I chose a ROS2 Action because inspection is a long-running task — it can take up to 60 seconds. Actions support streaming feedback so the BT gets live updates about the IBVS error, and they're cancellable. The pipeline starts by searching for objects on Insta360 for up to 20 seconds. If it finds front objects, it runs coarse positioning using the polynomial formula from week 2, then IBVS fine-tuning on the Logitech camera. Once centred, it waits 10 seconds for the camera to autofocus — this is important for gauges especially — then captures the images."

---

## Slide 6: ByteTrack Multi-Object Tracking

○ **Problem without tracking:** Multiple same-class objects (3 fire extinguishers) have no stable identity → hard to process sequentially

○ **Solution:** ByteTrack via Ultralytics `model.track(persist=True, tracker='bytetrack.yaml')`
  - Each object gets a stable integer ID that persists across frames
  - ID stays even if object briefly disappears (noise, motion blur)

○ Objects processed in **track ID order** for consistent, predictable sequence

○ Result visible in debug view: `[fire_extinguisher] ID38 0.82`

```python
# Detection tuple (per object):
(cx, cy, x1, y1, x2, y2, confidence, track_id, class_name)
```

○ Front objects sorted by track_id → always same processing order for same scene

**Script for Slide 6:**
"ByteTrack solves the multiple object problem. Without tracking, if YOLO detects 3 fire extinguishers, they have no stable identity — they're just 3 bounding boxes. With ByteTrack, each object gets a persistent ID number. Fire extinguisher at the left might always be ID 12, the middle one ID 15, the right one ID 18. We process them in ID order, which means the inspection sequence is always the same regardless of frame-to-frame detection noise. ByteTrack can also handle brief disappearances — if an object is temporarily blocked, it keeps the same ID when it reappears."

---

## Slide 7: Front / Back Zone Detection

○ Robot's inspection area has two sides — objects can be on FRONT or BACK

○ Insta360 360° output maps physically: **top of frame = FRONT, bottom = BACK**

```
Insta360 frame (640 × 480 equirectangular):

  y=0   ┌─────────────────────────┐
        │  FRONT zone (cy < 200)  │  ← green boxes
  y=200 ├─────────────────────────┤  ← boundary line
        │  BACK zone (cy > 200)   │  ← orange boxes
  y=480 └─────────────────────────┘
```

○ If ALL objects detected are in BACK zone:
  - Action returns `failed_reason = "all_in_back"`, `object_in_back = True`
  - BT must rotate robot 180° and call action again

○ If SOME objects front + SOME back:
  - Front objects are inspected fully
  - Result includes `object_in_back = True` → BT rotates after front done

**Script for Slide 7:**
"The robot can be facing a wall where objects are in front, or they might be behind the robot. The Insta360 gives a 360-degree view as a flat equirectangular image. The top half of this image corresponds to the front direction, the bottom half to the back. We split at 200 pixels — objects above are FRONT objects that we can inspect directly. Objects below are BACK objects. If we detect back objects, we signal the Behaviour Tree via the object_in_back result field. The BT then commands the robot to rotate 180 degrees and calls our action again."

---

## Slide 8: Action Interface — What the BT Sees

```
# GOAL (BT sends this to start inspection):
int32  max_objects      # 0=all, N=first N objects only
bool   return_home      # true = servos go to 90,90 after done
string location_label   # "unknown" or node label e.g. "gauge_room_A"
bool   overview_only    # true = skip IBVS, just capture 360 snapshot
int32  overview_count   # how many 360 images (default 2)

# RESULT (BT receives this when done):
bool   success
int32  objects_inspected  # how many fully centered + captured
int32  objects_found      # total detected on Insta360
bool   object_in_back     # true → BT must rotate 180°
string failed_reason      # ""|"no_detection"|"ibvs_timeout"|"all_in_back"

# FEEDBACK (streamed to BT while running):
string  current_step      # "detecting"/"coarse"/"ibvs"/"capturing"/"overview"
int32   current_object    # object number being processed
float32 ibvs_error_px     # live pixel error during IBVS
```

**BT Decision Table:**

| `success` | `object_in_back` | `failed_reason` | BT Action |
|-----------|-----------------|-----------------|-----------|
| True | False | `""` | ✅ Done, continue mission |
| True | True | `""` | Front done → rotate 180° → reinspect |
| False | True | `"all_in_back"` | Rotate 180° → retry |
| False | False | `"no_detection"` | Move to next position |
| False | False | `"ibvs_timeout"` | Skip, log alert |

**Script for Slide 8:**
"This is the contract between our system and the Behaviour Tree. The BT sends a Goal with parameters — the most important are location_label which gives a text label to the inspection location for logging, and overview_only which allows the BT to just request a 360 snapshot without running the full IBVS pipeline. The Result tells the BT what happened. The most important field is object_in_back — if true, the BT knows it needs to rotate the robot. The feedback streams live IBVS error every tick so the BT can monitor convergence. Ravith just needs to use the decision table when building the BT logic."

---

## Slide 9: Overview-Only Mode (for VLM Pipeline)

○ **New mode added this week:**  When BT sends `overview_only=True`:
  - Skip all IBVS stages entirely (no 20s search, no servo movement)
  - Immediately capture N Insta360 frames with YOLO bounding boxes drawn
  - Save to separate folder: `captures/overview/SESSION/`
  - Send via MQTT with `location_label` as identifier

○ **Why this exists:**
  - BT arrives at a known location but doesn't know what objects are there
  - Captures a wide 360° snapshot of the scene
  - Later: image sent to VLM (Vision Language Model) for object identification
  - VLM responds with object type → BT decides what to inspect

```
BT flow:
  CaptureOverview("location_A")  →  360° image → VLM → "I see a gauge"
  InspectObjects("location_A", label="gauge")  →  Full IBVS + ROI capture
```

**Script for Slide 9:**
"This mode was added because there's a VLM integration planned. The robot might not always know in advance what objects it will find at a location. Overview-only mode lets the BT quickly grab a 360-degree snapshot of the area without moving the servos at all. That image goes to a Vision Language Model which looks at the scene and says 'I see a gauge at position X' or 'there's a fire extinguisher here'. Then the BT calls the full inspection with that label. The two folders — inspection and overview — keep these separate."

---

## Slide 10: Capture Folder Structure & MQTT Delivery

**Local capture folder:**
```
captures/
├── inspection/                  ← Full IBVS ROI captures
│   └── 20260309_120000/         ← Session ID = inspection timestamp
│       └── fire_extinguisher/   ← YOLO class name (auto-detected)
│           └── instance_1/      ← instance_N for multiple same-class objects
│               ├── img_01.jpg   ← Logitech close-up (after 10s autofocus)
│               ├── img_02.jpg
│               ├── img_03.jpg
│               ├── img_04.jpg
│               └── overview_01.jpg  ← Insta360 with YOLO boxes
│
└── overview/                    ← BT-requested 360° snapshots (for VLM)
    └── 20260309_121000/
        ├── gauge_room_A_01.jpg
        └── gauge_room_A_02.jpg
```

**MQTT delivery to ThingsBoard:**
- Platform: ThingsBoard cloud (`demo.thingsboard.io:1883`)
- Auth: device access token (username=token, password=empty)
- Each image sent as separate JSON payload with class_name, session, object_id, base64 image
- `KEEP_LOCAL = True` — images saved locally AND sent via MQTT (both)
- ThingsBoard: `demo.thingsboard.io` → Devices → "inspection" → Latest Telemetry

**Script for Slide 10:**
"Every inspection creates a time-stamped session folder. Inside that, images are organized by YOLO class name — so gauge images go to the gauge folder, fire extinguisher images to fire_extinguisher folder. Each object instance gets its own subfolder. This structure makes it easy to use the captures directly as a labeled dataset. At the same time, all images are sent to ThingsBoard via MQTT as base64 JSON payloads. We keep local copies permanently — the KEEP_LOCAL flag is true — so we always have the dataset on the Jetson drive regardless of MQTT connectivity."

---

## Slide 11: Behaviour Tree Leaf Nodes — What We Provide

○ File: `visual_inspection_ros/bt_nodes/inspection_bt_nodes.py`

○ Three py_trees leaf node classes ready for Ravith to import:

```python
from visual_inspection_ros.bt_nodes.inspection_bt_nodes import (
    InspectObjectsAction,   # Runs full IBVS pipeline
    CaptureOverviewAction,  # Captures 360° snapshot for VLM
    CheckObjectInBack,      # Condition: reads object_in_back from blackboard
)
```

**InspectObjectsAction** — wraps the full ROS2 action:
```python
InspectObjectsAction(
    name='Inspect',
    node=ros_node,
    max_objects=0,          # inspect all detected
    return_home=True,
    location_label='unknown'
)
# Writes to blackboard: inspection_success, objects_found,
#   object_in_back, failed_reason, ibvs_error_px
```

**CaptureOverviewAction** — overview-only:
```python
CaptureOverviewAction(node=ros_node, location_label='gauge_room_A', overview_count=2)
# Writes to blackboard: overview_success
```

**CheckObjectInBack** — condition to trigger rotation:
```python
CheckObjectInBack()
# SUCCESS if object_in_back=True on blackboard
# FAILURE if no back objects
```

○ Can also run standalone BT demo:
```bash
ros2 run visual_inspection_ros run_inspection_bt
```

**Script for Slide 11:**
"For Ravith's Behaviour Tree integration, we've done most of the work already. There are three leaf node classes ready to use. InspectObjectsAction wraps the entire inspection pipeline as a py_trees Action — it sends the ROS2 action goal, monitors feedback, and when done writes the results to the BT blackboard. CaptureOverviewAction is for the VLM step — quick 360 snapshot. CheckObjectInBack is a condition that reads the blackboard and returns SUCCESS if we need a rotation. Ravith just imports these three classes and places them in the tree structure. We also have a standalone BT demo that can run without the main BT for testing."

---

## Slide 12: All ROS2 Topics — Complete Reference

```
PUBLISHED by our nodes:

  /visual_inspection/insta360/image_raw     sensor_msgs/Image
      → Raw Insta360 camera feed (30 Hz)

  /visual_inspection/logitech/image_raw     sensor_msgs/Image
      → Raw Logitech camera feed (30 Hz)

  /visual_inspection/debug                  sensor_msgs/Image
      → Combined side-by-side debug view with:
        - Class name + track ID + confidence on bounding boxes
        - IBVS arrow (blue) showing correction direction
        - Front/back boundary line
        - Mode text overlay (IDLE/DETECTING/COARSE/IBVS/CAPTURING)
        - FPS counter

  /visual_inspection/status                 std_msgs/String
      → Current mode: IDLE/DETECTING/COARSE/IBVS/CAPTURING/OVERVIEW

  /visual_inspection/ibvs_error             geometry_msgs/Point
      → x=pixel_error_x, y=pixel_error_y, z=total_magnitude

  /visual_inspection/detections             std_msgs/String
      → JSON: {front:[{cx,cy,conf,track_id,class}], back:[...]}

SUBSCRIBED by our nodes:

  /servo/pan_tilt                           std_msgs/Int16MultiArray
      → [tilt, pan] servo angles 0-180°

ACTION SERVER:

  /visual_inspection/inspect_objects        InspectObjects action
      → Full inspection pipeline
```

**Monitoring commands:**
```bash
ros2 topic list                              # all active topics
ros2 topic hz /visual_inspection/debug      # check frame rate
ros2 topic echo /visual_inspection/status   # watch mode changes
ros2 topic echo /visual_inspection/ibvs_error  # live pixel error
ros2 topic echo /visual_inspection/detections  # YOLO output JSON
ros2 action list                            # confirm server active
```

**Script for Slide 12:**
"Here's the complete ROS2 topic reference. The debug topic is especially useful — it's a single image topic you view in RViz2 that shows everything happening in real time: both cameras side by side, bounding boxes with class names and track IDs, the IBVS correction arrow, and the current mode. The status topic gives a simple string so any BT or monitoring node can see what stage we're in. The detections topic gives the full JSON detection output including which objects are in front and which are in back."

---

## Slide 13: Summary — Jetson Side Complete

```
┌─────────────────── JETSON ORIN NANO ─────────────────────┐
│                                                           │
│  camera_node ──→ /insta360/image_raw                      │
│                → /logitech/image_raw                      │
│                                                           │
│  servo_node ←── /servo/pan_tilt ←── ibvs_action_server   │
│       ↓                                                   │
│    Arduino ──→ Pan-Tilt Servos                            │
│                                                           │
│  ibvs_action_server:                                      │
│    ← Goal from BT (or test script)                        │
│    → Feedback: IBVS error, current step                   │
│    → Result: success, objects_found, object_in_back       │
│    → MQTT: Images to ThingsBoard cloud                    │
│    → Local: captures/ folder (labeled by class)           │
│                                                           │
│  bt_nodes/ → Ready for Ravith to import                  │
└───────────────────────────────────────────────────────────┘

STATUS: ✅ Jetson implementation COMPLETE
Next: BT integration (Ravith) + VLM integration
```

**What Ravith needs to do:**
1. `pip install py_trees`
2. Import the three leaf node classes from `inspection_bt_nodes.py`
3. Place `InspectObjectsAction` in BT sequence
4. Add `CheckObjectInBack` → `RotateRobot180` → `InspectObjectsAction` for back-side
5. Optionally add `CaptureOverviewAction` before inspection for VLM

**Script for Slide 13:**
"To summarise: the Jetson side implementation is complete. All three ROS2 nodes are working and tested. The action server handles the full inspection pipeline including multi-object tracking, front-back detection, IBVS centering, autofocus wait, image capture, and MQTT delivery. We've verified IBVS error converges to 5-10 pixels on real objects. ThingsBoard is receiving telemetry. The BT leaf node classes are ready. For Ravith to integrate, it's really five steps — install py_trees, import the classes, connect them in the tree following the decision table we've documented, and test."

---

## Technical Explanations

**1. Why ROS2 Action and not Service?**
- `ros2 service call` is synchronous — it blocks until complete. For 60-second tasks this hangs the BT.
- ROS2 Actions are asynchronous with streaming feedback. The BT stays responsive, gets live error values, and can cancel if needed.
- Actions also have proper state machine (ACCEPTED → EXECUTING → SUCCEEDED/ABORTED/CANCELED).

**2. Why OpenCV V4L2 and not GStreamer for cameras?**
- GStreamer uses pipeline strings like `gst-launch-1.0 v4l2src device=/dev/video0 ! ...` — more complex
- OpenCV's `cv2.VideoCapture('/dev/insta360')` works directly with udev symlinks
- V4L2 backend is more reliable for this use case on Jetson
- If higher performance were needed (raw MJPEG decode on GPU), GStreamer would be used

**3. What is ByteTrack?**
- A multi-object tracker that gives each detected object a stable integer ID
- Works by matching bounding boxes across frames using IoU (Intersection over Union)
- High-confidence detections tracked immediately, low-confidence detections buffered
- Result: `model.track(frame, persist=True)` returns boxes with `.id` attribute
- Enables: processing 3 fire extinguishers one at a time without losing track which is which

**4. Why IBVS and not direct angle calculation?**
- We don't know the 3D position of the object (no depth sensor)
- IBVS works purely in image space: error = (object_center - frame_center) in pixels
- PID controller converts pixel error to servo angle change
- No calibration needed beyond the camera's field of view
- Extremely robust to model errors because it's feedback controlled

**5. Why 10-second autofocus wait?**
- Logitech C920 uses continuous autofocus
- After servo stops moving, the camera needs time to detect the new focal distance and adjust
- Without waiting: blurry images, unreadable gauge text
- 10 seconds ensures the camera fully focuses regardless of distance
- This is the most important quality factor for gauge inspection

**6. What is equirectangular projection (Insta360 output)?**
- Insta360 maps the 360° sphere onto a flat rectangular image
- Left edge = right edge (they connect in 3D)
- Top = front/up, bottom = back/down
- This is why cy < 200px means front and cy > 200px means back in our system
- It also means objects at the edges of the frame are physically adjacent, not far apart

**7. YOLO class names and dataset collection:**
- Our trained YOLO model detects 3 classes: `door`, `gauge`, `fire_extinguisher`
- Class name comes directly from YOLO: `results.names[int(box.cls[0])]`
- This class name automatically becomes the folder name in the capture structure
- So captures are already labeled: `captures/inspection/SESSION/gauge/instance_1/...`
- Ready for use as training data with no additional labeling step

**8. ThingsBoard MQTT authentication:**
- ThingsBoard uses device access token as MQTT username, empty password
- Token: printed on ThingsBoard device page
- Connection: `client.username_pw_set(token, '')`
- Topic: `v1/devices/me/telemetry` (hard-coded in ThingsBoard protocol)
- Images sent as base64 strings in JSON — ThingsBoard stores as string telemetry values

---

## Next Steps

1. **Ravith — BT integration** with our leaf nodes (see README for complete instructions)
2. **Dataset collection** — systematic capture of doors, gauges, fire extinguishers across different locations and distances
3. **VLM integration** — images from `captures/overview/` → VLM API → object label → BT decision
4. **Field testing** — test on actual inspection locations with robot navigation

---

*Report prepared by: Dinethra | 2026-03-10*
