# Visual Inspection — Data Collection Guide
**Version:** 3.0 — Everything automatic via collect_dataset.py

---

## STEP 1 — Start Pipeline (3 terminals, same every session)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node
```

```bash
# Terminal 2
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node
```

```bash
# Terminal 3 — watch this for IBVS time and error
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server
```

---

## STEP 2 — Run Collection Script (Terminal 4, only command you need)

```bash
python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

**You will see:**
```
═══════════════════════════════════════════════
  VISUAL INSPECTION — DATASET COLLECTION
═══════════════════════════════════════════════
  1: Reference images (1m, 0°)
  2: Angle evaluation
  3: Distance evaluation
  4: Gauge accuracy ground truth
  5: VLM PASS/FAIL images
  6: Occlusion evaluation
  7: Multi-object scenes

Select session type:
```

**What the script does automatically:**
- Runs the inspection
- Detects the new captured image in captures/
- Copies it to the correct evaluation folder
- Writes a row to capture_log.csv
- Loops for the next image

**Your only jobs:**
1. Pick session type from menu
2. Move robot to position
3. Press Enter
4. Read ibvs_time and error from Terminal 3 (type them in when asked)
5. Type caption for VLM images (1 sentence)

---

## WHAT EACH SESSION COLLECTS

### Session 1 — Reference Images
- **When asked:** pick object (fire_ext / gauge / door)
- **Setup:** robot at 1m, directly facing object, good lighting
- **Repeat:** 5 times per object
- **Purpose:** baseline images for ALL quality metrics

### Session 2 — Angle Evaluation ← Professor required
- **When asked:** pick angle from menu (0°, 15°L, 15°R, 30°L, 30°R, 45°L, 45°R)
- **Setup:** robot at 2m, positioned at that angle
- **Object:** fire extinguisher (fixed position, only robot moves)
- **Repeat:** 5 images per angle position
- **Order to collect:** 0° → 15°L → 15°R → 30°L → 30°R → (45° if time)

### Session 3 — Distance Evaluation
- **When asked:** pick distance (1m / 2m / 3m / 4m)
- **Setup:** robot at 0° head-on, only distance changes
- **Object:** GAUGE (most important — checking OCR readability at distance)
- **Repeat:** 5 images per distance
- **Note when asked:** can you read gauge numbers? (y/n)

### Session 4 — Gauge Accuracy Ground Truth
- **Before starting:** set gauge pointer to a known position (tape if needed)
- **When asked:** type the TRUE reading value (e.g. `2.5`)
- **Target:** 5 positions × 3 images = 15 images total
  - Low (10% of scale)
  - 25% of scale
  - Mid (50%)
  - 75% of scale
  - High (90%)
- **CRITICAL:** type true value correctly when asked — this is the ground truth for MAE/RMSE

### Session 5 — VLM PASS/FAIL Images
- **When asked:** pick scenario from menu:
  - `fire_ext_pass` → extinguisher on wall, nothing blocking
  - `fire_ext_fail` → large box/chair placed directly in front blocking it
  - `exit_pass` → door with completely clear walkway
  - `exit_fail` → stack chairs/boxes blocking the door
  - `door_pass` → open door OR closed door (both are PASS)
- **Caption when asked:** write 1 sentence describing exactly what camera sees
  - Example PASS: `"Red fire extinguisher on wall bracket, fully accessible, clear path"`
  - Example FAIL: `"Red fire extinguisher on wall, large cardboard box blocking access"`
  - Example EXIT FAIL: `"Emergency exit door blocked by 3 stacked chairs in front"`
- **Repeat:** 5 images per scenario

### Session 6 — Occlusion Evaluation
- **When asked:** pick occlusion level (0% / 25% / 50% / 75%)
- **Setup:** cover that fraction of object with tape or cardboard
  - 0% = nothing covering
  - 25% = tape over bottom quarter
  - 50% = tape over bottom half
  - 75% = cover three-quarters (expect detection to fail — that's OK)
- **Object:** fire extinguisher | Distance: 2m | Angle: 0°
- **Repeat:** 5 images per occlusion level

### Session 7 — Multi-object (if time)
- **When asked:** enter number of objects and description
- **Setup:** 2 extinguishers side by side, or mixed gauge+extinguisher
- **Repeat:** 3 runs per scene

---

## READING IBVS STATS FROM TERMINAL 3

After each inspection, Terminal 3 shows:
```
  IBVS converged: err=7.8px at iter 38
```
- **ibvs_time_s** = iter × 0.066 → 38 × 0.066 = **2.5 seconds**
- **final_error_px** = **7.8**
- **converged** = y

If it timed out:
```
  IBVS timeout after 40.0s
```
- converged = n
- ibvs_time_s = 40
- final_error_px = 0

---

## AFTER ALL SESSIONS — COPY TO LAPTOP

```bash
# Run on LAPTOP to pull everything
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/
```

---

## COLLECTION CHECKLIST

```
SESSION 1 — REFERENCE (1m, 0°, best lighting)
[ ] fire_extinguisher — 5 images
[ ] gauge             — 5 images
[ ] door              — 5 images

SESSION 2 — ANGLE (fire_ext, 2m fixed, move robot)
[ ] 0°   — 5 images
[ ] 15°L — 5 images
[ ] 15°R — 5 images
[ ] 30°L — 5 images
[ ] 30°R — 5 images
[ ] 45°L — 5 images (if time)
[ ] 45°R — 5 images (if time)
[ ] Vertical angles — 3 images each (if time)

SESSION 3 — DISTANCE (gauge, 0° fixed)
[ ] 1m — 5 images
[ ] 2m — 5 images
[ ] 3m — 5 images
[ ] 4m — 5 images

SESSION 4 — GAUGE GROUND TRUTH (2m, 0°)
[ ] low reading  × 3 images (true value written)
[ ] 25% of scale × 3 images
[ ] 50% mid      × 3 images
[ ] 75% scale    × 3 images
[ ] high reading × 3 images

SESSION 5 — VLM IMAGES (captions written for each)
[ ] fire_ext_pass — 5 images
[ ] fire_ext_fail — 5 images
[ ] exit_pass     — 5 images
[ ] exit_fail     — 5 images
[ ] door_pass     — 3 images open + 3 closed

SESSION 6 — OCCLUSION (fire_ext, 2m, 0°)
[ ] 0%  — 5 images
[ ] 25% — 5 images
[ ] 50% — 5 images
[ ] 75% — 5 images

SESSION 7 — MULTI-OBJECT (if time)
[ ] 2 objects — 3 runs
[ ] mixed     — 3 runs
```

*inspect_ws untouched — evaluation/ folder is completely separate*
