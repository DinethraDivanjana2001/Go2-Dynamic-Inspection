# Visual Inspection — Data Collection Guide (Jetson Only)
**Date:** 2026-03-12 | Version: 2.0

---

## WHAT THIS GUIDE IS

This guide is ONLY for collecting the dataset on Jetson.
- ✅ Where to position robot
- ✅ Which commands to run
- ✅ Where to copy images
- ✅ What to write in the log
- ❌ NO evaluation scripts — those run on laptop LATER

---

## FOLDER STRUCTURE ON JETSON

Create this once on Jetson (inside `/home/rgen/Documents/Visual_Inspection_ws/`) — completely separate from `inspection_ws/`:

```bash
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/reference
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/angle_eval/horizontal/{0deg,15deg_L,15deg_R,30deg_L,30deg_R,45deg_L,45deg_R}
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/angle_eval/vertical/{0deg,15deg_up,15deg_down,30deg_up,30deg_down}
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/distance_eval/{1m,2m,3m,4m}
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/occlusion/{0pct,25pct,50pct,75pct}
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/gauge_accuracy
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/vlm_eval/{fire_ext_pass,fire_ext_fail,exit_pass,exit_fail,door_pass,door_fail,cylinder_pass,cylinder_fail}
mkdir -p ~/Documents/Visual_Inspection_ws/evaluation/multi_object/{2_objects,3_objects}

# Create the log file
echo "timestamp,folder,filename,object_type,distance_m,angle_deg,angle_direction,occlusion_pct,n_objects,ibvs_time_s,final_error_px,converged,ground_truth_value,notes" \
  > ~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv

echo "Done!"
```

---

## JETSON — PIPELINE (same every session, never changes)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node

# Terminal 2
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node

# Terminal 3 — WATCH THIS for IBVS time and error
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server

# Terminal 4 — Run one inspection
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py
```

---

## WHERE IMAGES AUTO-SAVE

Every time you run Terminal 4, images automatically appear here:
```
~/Documents/Visual_Inspection_ws/captures/inspection/SESSION_TS/CLASSNAME/instance_1/
  img_01.jpg       ← USE THIS — best Logitech close-up
  img_02.jpg
  img_03.jpg
  img_04.jpg
  overview_01.jpg  ← Insta360 wide view
```
`SESSION_TS` = timestamp like `20260313_101500`

**Find latest session:**
```bash
ls -lt ~/Documents/Visual_Inspection_ws/captures/inspection/ | head -5
```

---

## HOW TO READ IBVS TIME AND ERROR FROM TERMINAL 3

After each run, look at Terminal 3 for:
```
  IBVS converged: err=7.8px at iter 38
```
- **ibvs_time_s** = iter × 0.066 → 38 × 0.066 ≈ **2.5 seconds**
- **final_error_px** = **7.8**
- **converged** = **True**

If timeout:
```
  IBVS timeout after 40.0s
```
- converged = **False**, final_error_px = last printed error

---

## HOW TO WRITE THE LOG

Open a second terminal on Jetson and add one line per run:
```bash
LOGFILE=~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv

echo "2026-03-13_10:00:00, angle_eval/horizontal/30deg_L, img_01.jpg, fire_extinguisher, 2.0, 30, L, 0, 1, 4.2, 8.5, True, N/A, converged fine" >> $LOGFILE
```

Columns: `timestamp, folder, filename, object_type, distance_m, angle_deg, angle_direction, occlusion_pct, n_objects, ibvs_time_s, final_error_px, converged, ground_truth_value, notes`

---

## PAPER LOG TEMPLATE (print and bring)

```
Date: _________  Object: _____________  Location: ___________  Distance: ___m

| Run | Angle | ibvs_time(s) | final_err(px) | Converged | Notes |
|-----|-------|--------------|---------------|-----------|-------|
| 1   |       |              |               | Y / N     |       |
| 2   |       |              |               | Y / N     |       |
| 3   |       |              |               | Y / N     |       |
| 4   |       |              |               | Y / N     |       |
| 5   |       |              |               | Y / N     |       |

Gauge true reading (write down): _______________
VLM caption (write): _____________________________________________
```

---

---
# SESSION 1 — REFERENCE IMAGES
**Do this FIRST — needed for all quality metrics later**

**Object:** fire extinguisher, gauge, door (one by one)
**Distance:** 1m
**Angle:** 0° — robot facing object directly
**Lighting:** good indoor, no glare

### Steps:
1. Place object, position robot at 1m, 0°
2. Run Terminal 4 → inspect
3. Check Terminal 3 → note ibvs_time and error
4. Copy best image:
```bash
# Find latest session and copy
SESSION=$(ls -t ~/Documents/Visual_Inspection_ws/captures/inspection/ | head -1)
cp ~/Documents/Visual_Inspection_ws/captures/inspection/$SESSION/fire_extinguisher/instance_1/img_01.jpg \
   ~/Documents/Visual_Inspection_ws/evaluation/reference/fire_ext_ref_01.jpg
```
5. Repeat 5 times → you'll have `fire_ext_ref_01.jpg` to `fire_ext_ref_05.jpg`
6. Do same for gauge → `gauge_ref_01.jpg` ... `gauge_ref_05.jpg`
7. Do same for door → `door_ref_01.jpg` ... `door_ref_05.jpg`

**Log example:**
```
2026-03-13_10:00:00, reference, fire_ext_ref_01.jpg, fire_extinguisher, 1.0, 0, center, 0, 1, 2.5, 7.8, True, N/A, best reference 1m head-on
```

---
# SESSION 2 — ANGLE EVALUATION (Professor Required)
**Object:** fire extinguisher (fixed on wall)
**Distance:** 2m — keep fixed, only move robot angle

### For each position (5 images each):

**0° Head-on**
1. Robot facing object directly at 2m
2. Run pipeline 5 times
3. Copy images:
```bash
SESSION=$(ls -t ~/Documents/Visual_Inspection_ws/captures/inspection/ | head -1)
DEST=~/Documents/Visual_Inspection_ws/evaluation/angle_eval/horizontal/0deg
cp ~/Documents/Visual_Inspection_ws/captures/inspection/$SESSION/fire_extinguisher/instance_1/img_01.jpg $DEST/img_01.jpg
# Repeat for each run renaming img_01 → img_02, img_03 etc
```
4. Log each run in capture_log.csv

**15° Left**
- File to: `angle_eval/horizontal/15deg_L/`

**15° Right**
- File to: `angle_eval/horizontal/15deg_R/`

**30° Left**
- File to: `angle_eval/horizontal/30deg_L/`

**30° Right**
- File to: `angle_eval/horizontal/30deg_R/`

**45° Left (if time)**
- File to: `angle_eval/horizontal/45deg_L/`

**45° Right (if time)**
- File to: `angle_eval/horizontal/45deg_R/`

**Vertical angles (3 images each, if time):**

| Position | How | Folder |
|----------|-----|--------|
| 0° | Camera at same height as object center | `vertical/0deg/` |
| 15° up | Raise object 30cm higher than camera | `vertical/15deg_up/` |
| 15° down | Lower object 30cm below camera | `vertical/15deg_down/` |
| 30° up | Object 60cm above camera | `vertical/30deg_up/` |

---
# SESSION 3 — DISTANCE EVALUATION
**Object:** GAUGE (important — checking if OCR reads at 4m)
**Angle:** 0° fixed — only change distance

**1 metre**
- 5 runs → copy to `distance_eval/1m/img_01.jpg` ... `img_05.jpg`
- **Note:** Can you personally read the gauge numbers in the image? (yes/no in log notes)

**2 metres**
- 5 runs → copy to `distance_eval/2m/`

**3 metres**
- 5 runs → copy to `distance_eval/3m/`

**4 metres**
- 5 runs → copy to `distance_eval/4m/`
- Expect possible blurry/failure — that is expected, just capture and note

---
# SESSION 4 — GAUGE ACCURACY GROUND TRUTH
**Most important for gauge evaluation**

**Setup:**
- Real pressure/water gauge
- Set pointer to a known position (tape if needed)
- Read the value yourself — write it down
- Distance: 2m, angle: 0°

**Target: 15+ images at 5 different positions**

| Position | Approx value (0-10 bar) | Target images |
|----------|-------------------------|---------------|
| Low 10% | 0.5 bar | 3 images |
| Quarter 25% | 2.5 bar | 3 images |
| Mid 50% | 5.0 bar | 3 images |
| Three-quarters 75% | 7.5 bar | 3 images |
| High 90% | 9.0 bar | 3 images |

**Naming:** `gauge_TRUE-VALUE_dist2m_n1.jpg`
Example: `gauge_2.5bar_dist2m_n1.jpg`

```bash
SESSION=$(ls -t ~/Documents/Visual_Inspection_ws/captures/inspection/ | head -1)
DEST=~/Documents/Visual_Inspection_ws/evaluation/gauge_accuracy
cp ~/Documents/Visual_Inspection_ws/captures/inspection/$SESSION/gauge/instance_1/img_01.jpg \
   $DEST/gauge_2.5bar_dist2m_n1.jpg
```

**CRITICAL log column `ground_truth_value`:**
```
2026-03-13_11:45:00, gauge_accuracy, gauge_2.5bar_dist2m_n1.jpg, gauge, 2.0, 0, center, 0, 1, 2.8, 6.1, True, 2.5, pointer between 2 and 3 marks
```

---
# SESSION 5 — VLM IMAGES
**For every image write a 1-sentence caption in log notes column**

### Fire Extinguisher PASS (5 images)
- Mount on wall, NOTHING blocking
- Copy to: `vlm_eval/fire_ext_pass/`
- Caption: "Red fire extinguisher on wall bracket, fully accessible, clear path in front"

### Fire Extinguisher FAIL (5 images)
- Place large box/chair directly in front blocking it
- Copy to: `vlm_eval/fire_ext_fail/`
- Caption: "Red fire extinguisher on wall, large box placed directly in front blocking access"

### Fire Extinguisher MISSING (3 images)
- Empty bracket (no extinguisher)
- Copy to: `vlm_eval/fire_ext_fail/`
- Caption: "Empty wall bracket, fire extinguisher is absent"

### Exit/Door PASS (5 images)
- Door with completely clear walkway
- Copy to: `vlm_eval/exit_pass/`

### Exit FAIL (5 images)
- Stack chairs/boxes blocking the door
- Copy to: `vlm_eval/exit_fail/`
- Caption: "Emergency exit blocked by stack of chairs"

### Door open (3 images) → `vlm_eval/door_pass/`
### Door closed (3 images) → `vlm_eval/door_pass/`

---
# SESSION 6 — OCCLUSION
**Object:** fire extinguisher
**Distance:** 2m, **Angle:** 0°
Use tape/cardboard to cover the object

| Level | How | Folder |
|-------|-----|--------|
| 0% | Nothing covering | `occlusion/0pct/` |
| 25% | Tape over bottom quarter | `occlusion/25pct/` |
| 50% | Tape over bottom half | `occlusion/50pct/` |
| 75% | Cover 3/4 of object | `occlusion/75pct/` |

**5 images per level**
Log occlusion_pct column for each image.

---
# SESSION 7 — MULTI-OBJECT (if time)

| Scene | Setup | Folder |
|-------|-------|--------|
| 2 same-class | 2 fire extinguishers side by side | `multi_object/2_objects/` |
| Mixed class | 1 gauge + 1 fire ext | `multi_object/2_objects/` |
| Front + Back | 1 object front half, 1 back half of Insta360 | `multi_object/2_objects/` |

---

## COLLECTION CHECKLIST

```
SESSION 1 — REFERENCE
[ ] fire_extinguisher — 5 images at 1m, 0°
[ ] gauge             — 5 images at 1m, 0°
[ ] door              — 5 images at 1m, 0°

SESSION 2 — ANGLE (object=fire_extinguisher, distance=2m fixed)
[ ] 0°    head-on    5 images
[ ] 15°L             5 images
[ ] 15°R             5 images
[ ] 30°L             5 images
[ ] 30°R             5 images
[ ] 45°L             5 images (if time)
[ ] 45°R             5 images (if time)
[ ] Vertical up/down 3 images each (if time)

SESSION 3 — DISTANCE (object=gauge, angle=0° fixed)
[ ] 1m — 5 images
[ ] 2m — 5 images
[ ] 3m — 5 images
[ ] 4m — 5 images

SESSION 4 — GAUGE GROUND TRUTH (2m, 0°)
[ ] 5 reading positions × 3 images = 15 images
[ ] True value written in log for EVERY image

SESSION 5 — VLM
[ ] fire_ext PASS — 5 images + captions written
[ ] fire_ext FAIL — 5 images + captions written
[ ] exit PASS     — 5 images + captions
[ ] exit FAIL     — 5 images + captions
[ ] door open     — 3 images + captions
[ ] door closed   — 3 images + captions

SESSION 6 — OCCLUSION (2m, 0°)
[ ] 0%  — 5 images
[ ] 25% — 5 images
[ ] 50% — 5 images
[ ] 75% — 5 images

SESSION 7 — MULTI-OBJECT (if time)
[ ] 2 same-class — 3 runs
[ ] Mixed class  — 3 runs
```

---

## AFTER EACH SESSION — SCP TO LAPTOP

```bash
# Run from LAPTOP after each session to pull all collected images
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/

# Also pull the log file
scp rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/capture_log.csv \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/capture_log.csv
```

---

*Jetson workspace untouched — all evaluation scripts run on laptop later*
