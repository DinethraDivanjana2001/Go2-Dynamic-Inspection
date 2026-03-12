# Visual Inspection System — Data Collection & Evaluation Master Guide

**System:** Go2 quadruped robot + Jetson Orin Nano + Insta360 (pole-mounted) + Logitech C920 (pan-tilt)  
**Pipeline:** Insta360 → YOLOv11n TensorRT → ByteTrack → Coarse (degree-4 polynomial) → IBVS PID → Logitech ROI → Server (Gauge 10-step or VLM/Gemini)  
**See also:** `EVALUATION_PLAN.md` — full metric explanations, formulas, and evaluation scripts

---

## SYSTEM CONTEXT (for any new agent reading this)

This is a two-stage visual inspection robot system:

1. **Jetson side (Ubuntu, ROS2 Humble):** The Insta360 360° camera sits on a tall pole on the robot's back. It gives a panoramic equirectangular view where **top half = front of robot, bottom half = back**. YOLOv11n (TensorRT FP16, ~30 FPS) runs on the Insta360 feed with ByteTrack for stable object IDs. When a FRONT object is found (cy < 200px), a calibrated **degree-4 polynomial formula** converts pixel (cx,cy) to pan/tilt servo angles (Arduino controls physical servos). The Logitech C920 on the pan-tilt then does **IBVS** — PID loop centering the object to < 10px error in the 640×360 frame. Then 4 ROI images are captured (after 10s autofocus wait) and sent via MQTT to ThingsBoard.

2. **Server side (Windows, FastAPI port 8001):** Receives ROI image via `POST /api/v1/jobs`. Routes by `object_type`:
   - `gauge` → 10-step geometric+DL pipeline (detection→crop→keypoints→ellipse→OCR→decimal→segmentation→projection→RANSAC→reading)
   - `fire_extinguisher` / `door` / `emergency_exit` / `main_cylinder` → VLM (Gemini 2.5 Flash with task-specific YAML prompt)
   - `unknown` → VLM auto-identifies object first, then applies rules; if gauge found, recommends gauge pipeline

---

## WHAT TARGET OBJECTS TO USE IN DEMO LOCATION

### Gauge (Most Critical — Professor Focus)

**Best option: Analog pressure gauge (Bourdon type)**
- Found on: air compressors, water pipes, gas cylinders, boiler rooms
- Requirements:
  - **Dial diameter > 80mm** (OCR needs to read numbers clearly)
  - Clear number markings (not faded)
  - Visible needle distinct from the dial face
  - Scale: 0-10 bar, 0-100 PSI, or similar — doesn't matter as long as numbers are clear
  - **Not digital** — must be analog with a physical rotating needle
- Where to find on campus: mechanical lab compressors, plumbing rooms, workshop air supply panels
- **For ground truth measurement:** The gauge itself IS the ground truth — read what it says, write it down, photograph it. No extra equipment needed.
- If no real gauge available: a **large clock face** (≥ 15cm diameter) works as a substitute for testing the pipeline — the numbers and hands are similar to a gauge. Note in your log it's a clock substitute.

**Demo setup recommendation:**
- Mount the gauge on a wall or board at **camera height ± 30cm**
- Place it so the robot can approach from at least 45° left and right without obstruction
- Good even lighting — avoid direct glare on the gauge glass
- Clear space in front (no clutter within 2m)

### Fire Extinguisher
- Standard red ABC dry powder or CO2 extinguisher
- Must be **wall-mounted on a bracket** (not lying on floor)
- Mounted at approximately chest height
- For FAIL scenario: place a cardboard box or chair directly in front blocking access path

### Door / Emergency Exit
- Any standard lab or corridor door works
- If you have a door with a **green EXIT sign** above it — that covers both door + emergency exit evaluation
- For FAIL scenario: stack chairs or boxes blocking the door

### Main Cylinder (if available)
- Any hydraulic cylinder, large pipe fitting, or industrial equipment with a floor area below it
- For FAIL scenario: pour a small amount of water on the floor near the base (simulates oil leak)
- If no cylinder available: skip this object type — it is lower priority

---

## COLLECTION PRIORITY ORDER

```
PRIORITY 1 — Angle Evaluation (Professor specifically required this)
PRIORITY 2 — Reference Images (needed baseline for ALL image quality metrics)
PRIORITY 3 — Distance Evaluation (1m, 2m, 3m, 4m)
PRIORITY 4 — Gauge Ground Truth (for MAE/RMSE accuracy evaluation)
PRIORITY 5 — VLM Labelled Images (fire ext PASS/FAIL, exit, door)
PRIORITY 6 — Occlusion Evaluation
PRIORITY 7 — IBVS Convergence Logs (automatic if logger added)
PRIORITY 8 — Multi-object scenes
```

---

## FOLDER STRUCTURE — Create this on Jetson NOW

```bash
mkdir -p ~/eval_dataset/reference
mkdir -p ~/eval_dataset/angle_eval/horizontal/{0deg,15deg_L,15deg_R,30deg_L,30deg_R,45deg_L,45deg_R}
mkdir -p ~/eval_dataset/angle_eval/vertical/{0deg,15deg_up,15deg_down,30deg_up,30deg_down}
mkdir -p ~/eval_dataset/distance_eval/{1m,2m,3m,4m}
mkdir -p ~/eval_dataset/occlusion/{0pct,25pct,50pct,75pct}
mkdir -p ~/eval_dataset/gauge_accuracy
mkdir -p ~/eval_dataset/vlm_eval/{fire_ext_pass,fire_ext_fail,exit_pass,exit_fail,cylinder_pass,cylinder_fail,door_pass,door_fail,unknown_various}
mkdir -p ~/eval_dataset/ibvs_logs
mkdir -p ~/eval_dataset/multi_object/{2_objects,3_objects}
```

Also create a metadata log file:
```bash
touch ~/eval_dataset/capture_log.csv
echo "timestamp,folder,filename,object_type,distance_m,angle_deg,angle_direction,occlusion_pct,n_objects,ibvs_time_s,final_error_px,converged,ground_truth_value,notes" > ~/eval_dataset/capture_log.csv
```

---

## PRIORITY 1 — ANGLE EVALUATION (Professor Required)

**What this proves:** How far off-angle can the robot approach and still successfully capture a usable ROI?

**Setup:** Fix the target object (e.g. fire extinguisher) in one place. Move the **robot** to different angles around it. Keep distance at **2m** for all angle tests.

### Horizontal Angles (most important)

For each angle position, run the full pipeline (Insta360→YOLO→Coarse→IBVS→Logitech capture):

| Position | Robot placement | Folder |
|----------|----------------|--------|
| 0° (head-on) | Directly facing object | `angle_eval/horizontal/0deg/` |
| 15° Left | Robot offset 15° to the left | `angle_eval/horizontal/15deg_L/` |
| 15° Right | Robot offset 15° to the right | `angle_eval/horizontal/15deg_R/` |
| 30° Left | Robot offset 30° to the left | `angle_eval/horizontal/30deg_L/` |
| 30° Right | Robot offset 30° to the right | `angle_eval/horizontal/30deg_R/` |
| 45° Left | Robot offset 45° to the left | `angle_eval/horizontal/45deg_L/` |
| 45° Right | Robot offset 45° to the right | `angle_eval/horizontal/45deg_R/` |

**For EACH angle position, capture:**
- 5 full pipeline runs (Insta360→IBVS→Logitech) — saves ROI to captures/ folder automatically
- Copy each captured Logitech image to the right angle folder with a numbered name:
  `img_01.jpg, img_02.jpg, img_03.jpg, img_04.jpg, img_05.jpg`
- Note down in your phone/paper: ibvs_time, final_error_px, did it converge?

**Also save the Insta360 screenshot** for each angle (shows how the detection looks in panoramic view from that angle).

### Vertical Angles

| Position | Robot/target placement | Folder |
|----------|----------------------|--------|
| 0° (same height) | Camera at same height as object | `angle_eval/vertical/0deg/` |
| 15° up | Target 30cm above camera height | `angle_eval/vertical/15deg_up/` |
| 15° down | Target 30cm below camera height | `angle_eval/vertical/15deg_down/` |
| 30° up | Target 60cm above camera height | `angle_eval/vertical/30deg_up/` |
| 30° down | Target 60cm below camera height | `angle_eval/vertical/30deg_down/` |

**Capture:** 3 images per position.

### Metadata to record for EACH angle capture (fill in capture_log.csv):
```
timestamp, angle_eval/horizontal/30deg_L, img_01.jpg, fire_extinguisher, 2.0, 30, Left, 0, 1, 4.2, 8.5, True, N/A, "converged after 4.2s"
```

---

## PRIORITY 2 — REFERENCE IMAGES (Needed for ALL image metrics)

**What this is:** The "gold standard" image for each object type captured in perfect conditions. SSIM, PSNR, VIF, AlexNet cosine ALL compare test images against these references.

**Conditions:** 1m distance, 0° angle (head-on), good indoor lighting, IBVS fully converged, after 10s autofocus.

**Capture for each object:**
- `reference/gauge_ref_01.jpg` through `gauge_ref_05.jpg` — gauge at some mid reading
- `reference/fire_ext_ref_01.jpg` through `fire_ext_ref_05.jpg`
- `reference/door_ref_01.jpg` through `door_ref_05.jpg`
- `reference/exit_ref_01.jpg` through `exit_ref_05.jpg` (if you have an exit sign target)

**Take 5 of each** — you will choose the best one as the reference later.

**Note:** These should be the BEST images you capture today. Good lighting. Use autofocus wait. Logitech camera only. Full IBVS convergence.

---

## PRIORITY 3 — DISTANCE EVALUATION

**What this proves:** How image quality degrades with distance. Find the maximum useful inspection distance.

**Setup:** Keep robot angle at exactly 0° (head-on). Objects must be always the same (don't swap objects mid-session).

### For Each Distance (1m, 2m, 3m, 4m):

Run full pipeline 5 times. Copy Logitech images to:
- `distance_eval/1m/img_01.jpg` ... `img_05.jpg`
- `distance_eval/2m/img_01.jpg` ... `img_05.jpg`
- `distance_eval/3m/img_01.jpg` ... `img_05.jpg`
- `distance_eval/4m/img_01.jpg` ... `img_05.jpg`

**Do this for GAUGE specifically** — because at 4m you need to check if OCR can still read the numbers.

**Record in log:** For each distance: ibvs convergence time, final pixel error, whether you could personally read the gauge numbers in the captured image (yes/no).

---

## PRIORITY 4 — GAUGE ACCURACY GROUND TRUTH

**What this proves:** Gauge reading pipeline (10-step CV+DL) accuracy against known real values.

**This is THE most important dataset for gauge evaluation.**

### How to set up ground truth:
1. Get a **real pressure gauge** (or whatever gauge you have)
2. Set the pointer to a **known, verifiable position** (e.g. manually push pointer to 1.0 bar, tape it there)
3. Capture with full pipeline — save the Logitech ROI
4. Record the TRUE value in your log

### Target: 20+ gauge images spanning full scale

| Reading target | Folder | How many images |
|---------------|--------|-----------------|
| Lowest readable value (e.g. 0.5) | `gauge_accuracy/` | 3 images |
| 25% of scale | `gauge_accuracy/` | 3 images |
| 50% of scale (mid) | `gauge_accuracy/` | 3 images |
| 75% of scale | `gauge_accuracy/` | 3 images |
| Highest readable value | `gauge_accuracy/` | 3 images |
| Various other spots | `gauge_accuracy/` | rest |

**Naming convention:** `gauge_TRUE-VALUE_dist-DISTANCE_attempt-N.jpg`
Example: `gauge_2.5bar_dist2m_n1.jpg`

**Log CRITICAL — for each gauge image:**
```
timestamp, gauge_accuracy, gauge_2.5bar_dist2m_n1.jpg, gauge, 2.0, 0, center, 0, 1, 3.1, 7.2, True, 2.5, "pointer between 2 and 3 markings"
```

The `ground_truth_value` column in the CSV = true reading. This is THE key number for MAE/RMSE calculation.

---

## PRIORITY 5 — VLM EVALUATION IMAGES

**What this proves:** VLM makes correct PASS/FAIL decisions across object types.

Need images with **known correct decision**. For each scenario run full pipeline, save Logitech ROI.

### Fire Extinguisher
| Scenario | Ground truth decision | How to set up | How many |
|----------|----------------------|---------------|---------|
| Accessible extinguisher | PASS | Mount on wall, nothing blocking | 5 images |
| Blocked extinguisher | FAIL | Place box/bag/chair directly in front | 5 images |
| Extinguisher missing | FAIL | Empty bracket/wall space | 3 images |

Save to: `vlm_eval/fire_ext_pass/` and `vlm_eval/fire_ext_fail/`

### Emergency Exit / Door
| Scenario | Ground truth | Setup | Count |
|----------|-------------|-------|-------|
| Clear exit path | PASS | Door with clear walkway | 5 |
| Blocked exit | FAIL | Stack boxes/chairs in front of door | 5 |
| Door open | PASS | Open door, identifiable state | 3 |
| Door closed | PASS | Closed door, identifiable state | 3 |

Save to: `vlm_eval/exit_pass/`, `vlm_eval/exit_fail/`, `vlm_eval/door_pass/`, `vlm_eval/door_fail/`

### Main Cylinder (if you have access to machinery)
| Scenario | Ground truth | Setup | Count |
|----------|-------------|-------|-------|
| No leak | PASS | Clean machinery, dry floor | 5 |
| Simulated leak | FAIL | Pour a little water on floor near cylinder (simulate) | 5 |

Save to: `vlm_eval/cylinder_pass/`, `vlm_eval/cylinder_fail/`

### Unknown Objects (for unknown→auto-detect VLM routing test)
Capture images of:
- A gauge (for testing: VLM identifies it → routes to gauge pipeline)
- Random objects (book, bottle, fire ext from a new angle)
Save to: `vlm_eval/unknown_various/` with notes in log

**For every VLM image, write a caption in the log (REQUIRED for BERTScore and LLM-as-judge):**
Example:
```
..., vlm_eval/fire_ext_fail, blocked_01.jpg, fire_extinguisher, 2.0, 0, center, 0, 1, -, -, -, FAIL, "Red fire extinguisher on wall, large cardboard box placed directly in front completely blocking access path"
```

---

## PRIORITY 6 — OCCLUSION EVALUATION

**What this proves:** How much partial occlusion the detection/IBVS/capture chain can tolerate.

**Setup:** Use tape, cardboard, or a cardboard box to cover portions of the object. Keep distance at 2m, angle at 0°.

| Occlusion | How to achieve | Images |
|-----------|---------------|--------|
| 0% | Full object visible | 5 |
| 25% | Cover bottom quarter of object | 5 |
| 50% | Cover bottom half | 5 |
| 75% | Cover three quarters | 5 |

For each: Does YOLO detect? Does IBVS converge? Save the captured ROI image.
Save to: `occlusion/0pct/`, `occlusion/25pct/`, `occlusion/50pct/`, `occlusion/75pct/`

**Also take a photo/screenshot with your PHONE** showing the actual setup (how much is covered) — this is evidence for your report.

---

## PRIORITY 7 — IBVS CONVERGENCE LOGGING

**Add this logging BEFORE you start collecting data today:**

In `ibvs_action_server.py`, find the IBVS loop (`_ibvs` function) and add at the end:

```python
# At the end of _ibvs() function, before return True:
import csv
from pathlib import Path
log_path = Path(os.path.expanduser('~/eval_dataset/ibvs_logs/convergence_log.csv'))
log_path.parent.mkdir(exist_ok=True)
with open(log_path, 'a', newline='') as f:
    writer = csv.writer(f)
    writer.writerow([
        time.strftime('%Y%m%d_%H%M%S'),  # timestamp
        ibvs_iter,                        # number of IBVS iterations
        round(time.time() - start_time, 2),  # total IBVS time in seconds
        round(err, 2),                    # final pixel error
        round(pan, 1),                    # final pan angle
        round(tilt, 1),                   # final tilt angle
    ])
```

This log saves automatically every time IBVS converges. You get a full log of all sessions today without doing anything extra.

---

## PRIORITY 8 — MULTI-OBJECT SCENES

**What this proves:** ByteTrack-based multi-object handling works correctly.

| Scene | Setup | Expected result |
|-------|-------|----------------|
| 2 same-class objects | 2 fire extinguishers side by side | Both inspected in consistent order |
| 3 same-class objects | 3 gauges across a panel | All 3 inspected, stable ByteTrack IDs |
| Mixed class | 1 gauge + 1 fire ext in same view | Both inspected, different classes handled |
| One front, one back | Object in front half + back half | Front inspected, back flagged |

Save all captures to `multi_object/2_objects/` and `multi_object/3_objects/`

---

## TODAY'S CAPTURE CHECKLIST

Print this. Tick as you go.

```
ANGLE EVALUATION (Professor Required)
[ ] 0°   head-on   2m — 5 images each object
[ ] 15°  left      2m — 5 images
[ ] 15°  right     2m — 5 images
[ ] 30°  left      2m — 5 images
[ ] 30°  right     2m — 5 images
[ ] 45°  left      2m — 5 images (if time)
[ ] 45°  right     2m — 5 images (if time)
[ ] 15°  tilt up   2m — 3 images (if time)
[ ] 15°  tilt down 2m — 3 images (if time)

REFERENCE IMAGES
[ ] gauge reference — 5 images (1m, 0°, good light)
[ ] fire ext reference — 5 images
[ ] door reference — 5 images

DISTANCE
[ ] 1m — gauge — 5 images
[ ] 2m — gauge — 5 images
[ ] 3m — gauge — 5 images
[ ] 4m — gauge — 5 images (to find failure distance)

GAUGE GROUND TRUTH
[ ] 5 different reading positions × 3 images = 15+ gauge images
[ ] Write true reading value for EVERY image in the log

VLM IMAGES
[ ] Fire ext PASS — 5 images
[ ] Fire ext FAIL — 5 images (box blocking)
[ ] Exit PASS — 5 images
[ ] Exit FAIL — 5 images (blocked)
[ ] Door open — 3 images
[ ] Door closed — 3 images
[ ] Write caption for EVERY VLM image in log

OCCLUSION
[ ] 0% — 5 images
[ ] 25% — 5 images
[ ] 50% — 5 images
[ ] 75% — 3 images

MULTI-OBJECT
[ ] 2 objects in scene — 3 runs
[ ] 3 objects in scene — 3 runs (if time)

IBVS LOGGING
[ ] Added CSV logger to ibvs_action_server.py before starting
[ ] Confirmed log file writes after first run
```

---

## METADATA CSV — Fill in DURING capture (not after)

**File:** `~/eval_dataset/capture_log.csv`

Columns:
```
timestamp, folder, filename, object_type, distance_m, angle_deg, angle_direction, 
occlusion_pct, n_objects, ibvs_time_s, final_error_px, converged, ground_truth_value, notes
```

Example rows:
```
2026-03-12_09:15:00, angle_eval/horizontal/30deg_L, img_01.jpg, fire_extinguisher, 2.0, 30, L, 0, 1, 4.2, 8.5, True, N/A, converged fine
2026-03-12_09:18:00, angle_eval/horizontal/45deg_L, img_01.jpg, fire_extinguisher, 2.0, 45, L, 0, 1, 7.1, 14.2, False, N/A, IBVS did not converge at 45deg
2026-03-12_09:22:00, gauge_accuracy, gauge_2.5_2m_n1.jpg, gauge, 2.0, 0, center, 0, 1, 2.8, 6.1, True, 2.5, pointer between 2 and 3 marks
2026-03-12_09:31:00, vlm_eval/fire_ext_fail, blocked_01.jpg, fire_extinguisher, 2.0, 0, center, 0, 1, 3.1, 7.8, True, FAIL, box placed in front of extinguisher completely blocking access
```

> **TIP:** Open the CSV in a second terminal and `echo "row..." >> capture_log.csv` as you go. Do not rely on memory.

---

## AFTER DATA COLLECTION — Copy to PC

```bash
# On Jetson — compress everything
tar -czvf eval_dataset_2026-03-12.tar.gz ~/eval_dataset/

# Copy to PC via SCP
scp jetson@JETSON_IP:~/eval_dataset_2026-03-12.tar.gz .

# Also copy the ibvs convergence log separately
scp jetson@JETSON_IP:~/eval_dataset/ibvs_logs/convergence_log.csv .
```

Then copy everything into your server workspace:
```
vi_server/eval_dataset/
```

---

## MINIMUM VIABLE DATASET (if time runs out)

If you run out of time, prioritize in this order:

1. **Angle images** (0°, 15°, 30°, 45° at 2m) — professor asked, MUST have
2. **Reference images** for each object — needed for all image quality metrics
3. **Gauge ground truth** — needed for numerical accuracy evaluation
4. **Fire ext PASS + FAIL** — most common VLM evaluation target
5. **Distance images** (1m, 2m, 3m) — shows system range

Everything else can be approximated or skip if no time.

---

*Robot session: 2026-03-12 | Capture EVERYTHING today — evaluate later*