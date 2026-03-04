# Visual Inspection — Phase 2 Plan
# What we build to connect into the Behaviour Tree

**Date:** March 2026  
**Workspace:** `inspection_ws/` on laptop → SCP to `~/Documents/Visual_Inspection_ws/`  
**Uses:** SAME venv + TensorRT engine already on Jetson — no reinstallation

---

## 🗂️ Workspace Clarification

```
LAPTOP (develop):
  Jetson_orin_nano/
  └── inspection_ws/       ← new code here
  └── jetson_workspace/    ← existing pipeline (unchanged)

JETSON (runs):
  ~/Documents/Visual_Inspection_ws/
  ├── venv/                ← EXISTING — same venv, don't touch
  ├── weights/
  │   └── yolo11n.engine   ← EXISTING — same TRT engine, don't touch
  ├── ibvs_pipeline.py     ← EXISTING — working pipeline
  └── inspection_ws/       ← NEW code SCP'd here, uses same venv
```

**Rule:** `source ~/Documents/Visual_Inspection_ws/venv/bin/activate` — then run anything from `inspection_ws/` directly. Same TRT, same packages.

---

## 🌳 Who Builds What

```
Other person builds:
├── Full robot BT (navigation, waypoint, ramp, etc.)
├── BT framework setup (BehaviorTree.CPP or py_trees_ros)
└── Calls our inspection actions from the BT

YOU build:
└── Visual Inspection BT Node(s) — self-contained module
    that the other person can plug into the tree
```

**Your job: build the inspection module so it has a clean interface.  
Their job: call that interface from the main BT.**

---

## 🔌 Interface — What We Expose to the BT

The other person's BT will call ONE action from you:

```
BT Action:  "InspectObjects"
─────────────────────────────
INPUT:
  - trigger: bool (BT sends True to start)
  - max_objects: int (how many objects to inspect, default=all)

OUTPUT (returns to BT):
  - status: SUCCESS / FAILURE
  - objects_inspected: int (how many done)
  - failed_reason: str (if FAILURE — "no_detection" / "ibvs_timeout" / "mqtt_error")

SIDE EFFECTS:
  - 4 images per object published to MQTT broker
  - Servo returned to home position (90,90) after done
```

That's it — one clean action. The rest is internal to us.

---

## 📋 What WE Add to the Behaviour Tree (our steps)

Even though the other person builds the BT, **you need to tell them what steps exist inside your action** so they can plan error handling. Here is what happens inside `InspectObjects`:

```
InspectObjects [Sequence]
│
├── STEP 1: Insta360 Multi-Object Detection
│   ├── Run YOLO on Insta360
│   ├── SUCCESS: objects detected → get list with positions
│   └── FAILURE: no detection
│       └── → Returns FAILURE to BT with reason="no_detection"
│           (BT person handles retry / robot rotation / skip)
│
├── STEP 2: For Each Object (loop)
│   │
│   ├── STEP 2a: COARSE Positioning
│   │   ├── Cubic formula → calculate servo angles
│   │   ├── Send angles to Arduino → servo moves
│   │   ├── SUCCESS: Logitech can see object
│   │   └── FAILURE: object not in Logitech view
│   │       └── Log skip, continue to next object
│   │
│   ├── STEP 2b: FINE Centering (IBVS)
│   │   ├── Run IBVS loop until centered (<10px error)
│   │   ├── SUCCESS: centered
│   │   └── FAILURE: max iterations hit OR object lost
│   │       └── Log skip, continue to next object
│   │
│   ├── STEP 2c: Capture 4 Images
│   │   ├── Capture Image 1
│   │   ├── Quick IBVS re-center (ensure still on target)
│   │   ├── Capture Image 2
│   │   ├── Quick IBVS re-center
│   │   ├── Capture Image 3
│   │   ├── Quick IBVS re-center
│   │   └── Capture Image 4
│   │
│   ├── STEP 2d: Publish via MQTT
│   │   ├── Pack 4 images + metadata (class, confidence, angles)
│   │   ├── Publish to broker
│   │   ├── SUCCESS: broker ACK received
│   │   └── FAILURE: no ACK → save locally, retry later
│   │
│   └── STEP 2e: Mark Object Done → next object
│
└── STEP 3: Return to Home Position
    ├── Send servo angles 90,90 to Arduino
    └── Return SUCCESS to BT
```

---

## 📤 What to Tell the Other Person (BT Interface Spec)

Give them this info to connect in the main BT:

```yaml
# inspection_interface.yaml — share this with BT person

action_name: InspectObjects

call_when:
  - Robot has arrived at inspection waypoint
  - Robot is stationary

inputs:
  trigger: bool

outputs:
  status: [SUCCESS, FAILURE]
  objects_inspected: int
  failed_reason: str   # only set on FAILURE

failure_reasons:
  no_detection: "YOLO found nothing on Insta360 — robot should rotate and retry"
  ibvs_timeout: "Could not center any object — check camera / servo"
  mqtt_error:   "Images captured but could not publish — check broker"

runtime:
  per_object: ~5-10 seconds (coarse + IBVS + 4 captures)
  total: ~20-40 seconds for 3 objects

side_effects:
  - Servos move during inspection
  - Servos return HOME (90,90) when done
  - Images published to MQTT topic: inspection/images/{object_id}
```

---

## 🔧 Implementation Plan (our side only)

### Phase 2a — Multi-Object + ByteTracker (no BT yet)
- [ ] Replace `SimpleTracker` with Ultralytics ByteTracker in pipeline
  ```python
  results = model.track(frame, persist=True, tracker="bytetrack.yaml")
  # Each box gets .id (track_id) → process in order
  ```
- [ ] Add "inspect all objects" loop — process each track_id one by one
- [ ] Mark inspected IDs → skip on next Insta360 frame
- [ ] Test with multiple fire extinguishers / gauges visible

### Phase 2b — 4-Image Capture with Re-centering
- [ ] After IBVS centered: capture image 1
- [ ] Quick IBVS check (3-5 iterations max) → recenter if drifted
- [ ] Capture image 2, 3, 4 — same pattern
- [ ] Save to `capture/{timestamp}/{object_id}_img{1-4}.jpg`
- [ ] Save metadata JSON alongside: class, confidence, servo_angles, timestamp

### Phase 2c — MQTT Publisher
- [ ] `pip install paho-mqtt` (in existing Jetson venv)
- [ ] Connect to broker (need broker IP from team)
- [ ] Publish `{4 images + metadata}` per object
- [ ] Handle offline → save locally, retry when back online

### Phase 2d — Wrap as clean callable module
- [ ] Class `InspectionModule` with single method `run() → SUCCESS/FAILURE`
- [ ] This is what the BT person calls
- [ ] Standalone test: `python3 inspection_ws/test_inspection.py`

### Phase 2e — ROS2 (when ROS2 available on Jetson)
- [ ] Wrap `InspectionModule` as ROS2 Action Server
- [ ] Action: `/inspection/run` (std_msgs)
- [ ] BT calls this via ROS2 action client

---

## ❓ Clarify with Team

1. **MQTT broker IP** — what is the broker host address?
2. **BT framework** — which does the other person use? (BehaviorTree.CPP / py_trees_ros)
   - If BehaviorTree.CPP → we expose a ROS2 action (C++ compatible)
   - If py_trees_ros → we write a py_trees Action node class directly
3. **ROS2 on Jetson?** — installed? which version? (affects how we expose the interface)
4. **Image format** — raw JPEG or base64 encode in JSON for MQTT?
5. **Object inspection order** — highest confidence first? left-to-right?
6. **Re-center threshold** — how many px drift before recapture?
