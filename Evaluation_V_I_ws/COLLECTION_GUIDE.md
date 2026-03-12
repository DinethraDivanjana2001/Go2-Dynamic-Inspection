# Visual Inspection System — Complete Data Collection & Evaluation Guide
**Date:** 2026-03-12 | **Robot:** Go2 + Jetson Orin Nano | **Version:** 1.0

---

## OVERVIEW

```
JETSON (data collection only — NO code changes)
  ↓  Run pipeline normally → images auto-save to captures/ folder
  ↓  You note: angle, distance, ibvs_time, final_error on PAPER
  ↓  SCP images to laptop after each session

LAPTOP (evaluation — after each session)
  ↓  Organise images into eval_dataset/ folders
  ↓  Run evaluation scripts
  ↓  Fill in results tables in evaluation_plan.md
```

**Collection priority order:**
1. Reference images ← needed for ALL image quality metrics
2. Angle evaluation ← professor specifically asked for this
3. Distance evaluation
4. Gauge accuracy ground truth
5. VLM images (fire ext, exit, door)
6. Occlusion evaluation
7. Multi-object scenes (if time)

---

## LAPTOP SETUP (one time only)

```bash
cd /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws

# Create virtual environment for evaluation
python3 -m venv eval_venv
source eval_venv/bin/activate
pip install -r scripts/requirements_eval.txt

# Create folder structure
mkdir -p eval_dataset/reference
mkdir -p eval_dataset/angle_eval/horizontal/{0deg,15deg_L,15deg_R,30deg_L,30deg_R,45deg_L,45deg_R}
mkdir -p eval_dataset/angle_eval/vertical/{0deg,15deg_up,15deg_down,30deg_up,30deg_down}
mkdir -p eval_dataset/distance_eval/{1m,2m,3m,4m}
mkdir -p eval_dataset/occlusion/{0pct,25pct,50pct,75pct}
mkdir -p eval_dataset/gauge_accuracy
mkdir -p eval_dataset/vlm_eval/{fire_ext_pass,fire_ext_fail,exit_pass,exit_fail,cylinder_pass,cylinder_fail,door_pass,door_fail,unknown_various}
mkdir -p eval_dataset/ibvs_logs
mkdir -p eval_dataset/multi_object/{2_objects,3_objects}
mkdir -p results

# Create capture log CSV
echo "timestamp,folder,filename,object_type,distance_m,angle_deg,angle_direction,occlusion_pct,n_objects,ibvs_time_s,final_error_px,converged,ground_truth_value,notes" \
  > eval_dataset/capture_log.csv
```

---

## JETSON — START PIPELINE (same 4 terminals every session)

```bash
# Terminal 1
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node

# Terminal 2
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node

# Terminal 3 — READ THIS TERMINAL for IBVS time and final error
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server

# Terminal 4 — Run one inspection
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_full_pipeline.py
```

**After each inspection, check Terminal 3 for these lines:**
```
  IBVS iter 0: err=142.3px ...
  ...
  IBVS converged: err=7.8px at iter 38    ← NOTE: err=7.8, calculate time = 38 × 0.066 ≈ 2.5s
```
Or if timeout:
```
  IBVS timeout after 40.0s               ← converged=False
```

**Images auto-save to:**
```
~/Documents/Visual_Inspection_ws/captures/inspection/SESSION_TS/fire_extinguisher/instance_1/
  img_01.jpg, img_02.jpg, img_03.jpg, img_04.jpg, overview_01.jpg
```

**After each session copy to laptop:**
```bash
# Run on LAPTOP
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/captures/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/raw_captures/
```

---

## SESSION 1 — REFERENCE IMAGES

**Goal:** Best possible images of each object. These are the baseline for SSIM/PSNR/VIF/AlexNet.

**Setup:**
- Distance: 1m
- Angle: 0° (head-on, facing directly at object)
- Lighting: good indoor lighting, no glare
- After IBVS converges → let autofocus settle (10s auto-wait already in pipeline)

**Objects to capture:**

### Fire Extinguisher Reference
1. Place fire extinguisher on wall mount, nothing blocking it
2. Position robot exactly 1m away, facing it directly
3. Run pipeline: `python3 test_full_pipeline.py`
4. Check Terminal 3: "IBVS converged: err=X.Xpx"
5. Repeat 5 times (pipeline auto-captures 4 images each run)
6. Take best image → copy to `eval_dataset/reference/fire_ext_ref_01.jpg`

### Gauge Reference
1. Mount gauge at camera height (± 30cm)
2. Good lighting on gauge face
3. Run pipeline 5 times → copy best image to `reference/gauge_ref_01.jpg`

### Door Reference
1. Choose a door you'll use for all door tests
2. Run pipeline 5 times → copy best image to `reference/door_ref_01.jpg`

**Copy to laptop (after session):**
```bash
# On laptop:
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/captures/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/raw_captures/
```
Then manually copy the best images into `eval_dataset/reference/`.

**Log in capture_log.csv (for each reference image):**
```
2026-03-13_10:00:00, reference, fire_ext_ref_01.jpg, fire_extinguisher, 1.0, 0, center, 0, 1, 2.5, 7.8, True, N/A, best reference image 1m head-on
```

---

## SESSION 2 — ANGLE EVALUATION (Professor Required)

**Goal:** Test how far off-angle the robot can be and still get usable images.

**Setup:**
- Object: fire extinguisher (mounted on wall)
- Distance: 2m FIXED for all angles
- Move the ROBOT (not the object!)

**Horizontal Angles — 5 images each position:**

### Position 1: 0° Head-on
- Robot facing directly at object
- Run pipeline 5 times
- **Write down for each run:** ibvs_time_s, final_error_px, converged (True/False)
- Copy images to: `eval_dataset/angle_eval/horizontal/0deg/`

### Position 2: 15° Left
- Robot stands 15° to the left side
- Run pipeline 5 times
- Copy to: `angle_eval/horizontal/15deg_L/`

### Position 3: 15° Right
- Robot stands 15° to the right side
- Run pipeline 5 times
- Copy to: `angle_eval/horizontal/15deg_R/`

### Position 4: 30° Left
- Run pipeline 5 times
- Copy to: `angle_eval/horizontal/30deg_L/`

### Position 5: 30° Right
- Run pipeline 5 times
- Copy to: `angle_eval/horizontal/30deg_R/`

### Position 6: 45° Left (if time)
- Run pipeline 5 times
- Copy to: `angle_eval/horizontal/45deg_L/`

### Position 7: 45° Right (if time)
- Copy to: `angle_eval/horizontal/45deg_R/`

**Log in capture_log.csv for every image:**
```
2026-03-13_10:30:00, angle_eval/horizontal/30deg_L, img_01.jpg, fire_extinguisher, 2.0, 30, L, 0, 1, 4.2, 8.5, True, N/A, converged fine
2026-03-13_10:33:00, angle_eval/horizontal/45deg_L, img_01.jpg, fire_extinguisher, 2.0, 45, L, 0, 1, 0, 0, False, N/A, IBVS did not converge at 45deg
```

**Vertical Angles — 3 images each:**

| Position | Setup | Folder |
|----------|-------|--------|
| 0° same height | Camera at same height as object center | `vertical/0deg/` |
| 15° up | Raise object target 30cm above camera | `vertical/15deg_up/` |
| 15° down | Lower object 30cm below camera | `vertical/15deg_down/` |
| 30° up | Object 60cm above camera | `vertical/30deg_up/` |

**Run evaluation on laptop (after copying images):**
```bash
cd /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws
source eval_venv/bin/activate

# Evaluate angle=0deg (head-on) vs reference
python3 scripts/evaluate_image_quality.py \
  --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
  --test eval_dataset/angle_eval/horizontal/0deg/ \
  --out  results/angle_0deg.csv

# Evaluate 15deg left
python3 scripts/evaluate_image_quality.py \
  --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
  --test eval_dataset/angle_eval/horizontal/15deg_L/ \
  --out  results/angle_15deg_L.csv

# Evaluate 30deg left
python3 scripts/evaluate_image_quality.py \
  --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
  --test eval_dataset/angle_eval/horizontal/30deg_L/ \
  --out  results/angle_30deg_L.csv

# And so on for each angle folder...
# Also IBVS convergence stats grouped by angle:
python3 scripts/evaluate_ibvs.py \
  --log eval_dataset/capture_log.csv \
  --group_by angle \
  --out results/ibvs_by_angle.csv
```

---

## SESSION 3 — DISTANCE EVALUATION

**Goal:** How does image quality change with distance? Find maximum useful distance.

**Setup:**
- Object: **GAUGE** specifically (most important — need to check if OCR reads at 4m)
- Angle: 0° fixed
- Change only the distance

**Run pipeline at each distance:**

### 1 metre
- Position robot 1m from gauge
- Run pipeline 5 times
- Copy images to: `eval_dataset/distance_eval/1m/`
- **Note:** Can you read the gauge numbers in the captured image? (yes/no)

### 2 metres
- Move robot to 2m away
- Run pipeline 5 times
- Copy to: `distance_eval/2m/`

### 3 metres
- Copy to: `distance_eval/3m/`

### 4 metres
- Copy to: `distance_eval/4m/`
- **Expect possible failures** — this is to find the limit

**Log example:**
```
2026-03-13_11:00:00, distance_eval/1m, img_01.jpg, gauge, 1.0, 0, center, 0, 1, 2.1, 6.5, True, N/A, numbers clearly readable
2026-03-13_11:20:00, distance_eval/4m, img_01.jpg, gauge, 4.0, 0, center, 0, 1, 6.8, 15.2, True, N/A, numbers blurry cannot read OCR will fail
```

**Run evaluation on laptop:**
```bash
source eval_venv/bin/activate

for DIST in 1m 2m 3m 4m; do
  python3 scripts/evaluate_image_quality.py \
    --ref  eval_dataset/reference/gauge_ref_01.jpg \
    --test eval_dataset/distance_eval/$DIST/ \
    --out  results/distance_${DIST}.csv
done

# IBVS stats by distance:
python3 scripts/evaluate_ibvs.py \
  --log eval_dataset/capture_log.csv \
  --group_by distance \
  --out results/ibvs_by_distance.csv
```

---

## SESSION 4 — GAUGE ACCURACY GROUND TRUTH

**Goal:** Collect images with known true gauge readings for MAE/RMSE evaluation.

**Setup:**
- Distance: 2m (consistent)
- Angle: 0°
- Lighting: good, no glare on gauge glass
- Ground truth: read the gauge yourself and write it down

**How to set known readings:**
- Use a physical pressure/water gauge
- Manually move pointer to known positions (tape it if needed)
- Take a PHONE photo of the gauge setup as evidence
- OR: photograph gauge at its current reading and write the value down

**Target: 15+ images at different scale positions**

### Reading positions to cover:
| Target position | Example (0-10 bar scale) | Images |
|----------------|--------------------------|--------|
| Low (~10% scale) | 0.5 bar | 3 images |
| Quarter (25%) | 2.5 bar | 3 images |
| Mid (50%) | 5.0 bar | 3 images |
| Three-quarters (75%) | 7.5 bar | 3 images |
| High (~90%) | 9.0 bar | 3 images |
| Total | | 15 images |

**Naming convention (IMPORTANT):**
Name images as: `gauge_TRUE-VALUE_dist2m_n1.jpg`
Examples: `gauge_2.5bar_dist2m_n1.jpg`, `gauge_5.0bar_dist2m_n2.jpg`

```bash
# After pipeline runs, rename the files with true values:
# The captured files are in captures/inspection/SESSION/gauge/instance_1/
# Copy to eval_dataset/gauge_accuracy/ with correct names
cp captures/.../img_01.jpg eval_dataset/gauge_accuracy/gauge_2.5bar_dist2m_n1.jpg
```

**Log MUST include ground_truth_value column:**
```
2026-03-13_11:45:00, gauge_accuracy, gauge_2.5bar_dist2m_n1.jpg, gauge, 2.0, 0, center, 0, 1, 2.8, 6.1, True, 2.5, pointer between 2 and 3 markings
2026-03-13_11:50:00, gauge_accuracy, gauge_5.0bar_dist2m_n1.jpg, gauge, 2.0, 0, center, 0, 1, 3.0, 7.5, True, 5.0, pointer at midpoint
```

**Run evaluation on laptop (requires server running):**
```bash
source eval_venv/bin/activate

python3 scripts/evaluate_gauge.py \
  --images eval_dataset/gauge_accuracy/ \
  --log    eval_dataset/capture_log.csv \
  --server http://localhost:8001 \
  --out    results/gauge_eval_results.csv
```

---

## SESSION 5 — VLM EVALUATION IMAGES

**Goal:** Test PASS/FAIL decision accuracy for each object type.

**For EVERY image, write a 1-sentence caption in the log (required for LLM judge).**

### Fire Extinguisher PASS (5 images)
- Mount on wall, NOTHING blocking it
- Run pipeline 5 times
- Copy to: `eval_dataset/vlm_eval/fire_ext_pass/`
- Caption example: "Red fire extinguisher mounted on wall bracket, fully accessible with clear path in front"

### Fire Extinguisher FAIL (5 images)
- Place a large cardboard box or chair directly in front blocking access
- Run pipeline 5 times
- Copy to: `eval_dataset/vlm_eval/fire_ext_fail/`
- Caption: "Red fire extinguisher on wall, large cardboard box placed directly in front completely blocking access"

### Fire Extinguisher MISSING (3 images)
- Empty bracket (no extinguisher)
- Copy to: `eval_dataset/vlm_eval/fire_ext_fail/`
- Caption: "Empty wall bracket where fire extinguisher should be mounted, extinguisher is absent"

### Emergency Exit PASS (5 images)
- Door/exit with completely clear walkway
- Copy to: `vlm_eval/exit_pass/`
- Caption: "Emergency exit door clearly visible with no obstructions in front, path completely clear"

### Emergency Exit FAIL (5 images)
- Stack 2-3 chairs or boxes directly blocking the door
- Copy to: `vlm_eval/exit_fail/`
- Caption: "Emergency exit door blocked by stack of chairs placed directly in front preventing access"

### Door PASS — Open (3 images)
- Regular open door
- Copy to: `vlm_eval/door_pass/`

### Door PASS — Closed (3 images)
- Closed door (PASS because closed state is normal for a door)
- Copy to: `vlm_eval/door_pass/`

**Log format (notes column = caption):**
```
2026-03-13_13:00:00, vlm_eval/fire_ext_fail, blocked_01.jpg, fire_extinguisher, 2.0, 0, center, 0, 1, 3.2, 7.9, True, FAIL, Red fire extinguisher on wall blocked by large cardboard box
```

**Run evaluation on laptop (requires server):**
```bash
source eval_venv/bin/activate

# Standard VLM evaluation
python3 scripts/evaluate_vlm.py \
  --base   eval_dataset/vlm_eval/ \
  --log    eval_dataset/capture_log.csv \
  --server http://localhost:8001 \
  --out    results/vlm_eval_results.csv

# Consistency test (3 runs per image)
python3 scripts/evaluate_vlm.py \
  --base   eval_dataset/vlm_eval/ \
  --log    eval_dataset/capture_log.csv \
  --server http://localhost:8001 \
  --out    results/vlm_consistency_results.csv \
  --consistency

# LLM Judge (requires GEMINI_API_KEY)
export GEMINI_API_KEY=your_key_here
python3 scripts/evaluate_vlm_llm_judge.py \
  --results results/vlm_eval_results.csv \
  --images  eval_dataset/vlm_eval/ \
  --out     results/llm_judge_results.csv
```

---

## SESSION 6 — OCCLUSION EVALUATION

**Goal:** Find how much occlusion the system can tolerate.

**Setup:**
- Object: fire extinguisher
- Distance: 2m, angle: 0°
- Use cardboard/tape to cover portions

| Level | How | Folder |
|-------|-----|--------|
| 0% | Nothing covering | `occlusion/0pct/` |
| 25% | Tape over bottom quarter | `occlusion/25pct/` |
| 50% | Cover bottom half | `occlusion/50pct/` |
| 75% | Cover bottom 3/4 | `occlusion/75pct/` |

- 5 images per level (3 if detection fails)
- Take a PHONE photo showing the actual occlusion setup at each level

**Log:**
```
2026-03-13_14:00:00, occlusion/25pct, img_01.jpg, fire_extinguisher, 2.0, 0, center, 25, 1, 3.5, 9.2, True, N/A, bottom quarter covered with tape
2026-03-13_14:10:00, occlusion/75pct, img_01.jpg, fire_extinguisher, 2.0, 0, center, 75, 1, 0, 0, False, N/A, YOLO did not detect at 75%
```

**Run evaluation on laptop:**
```bash
source eval_venv/bin/activate

for OCC in 0pct 25pct 50pct 75pct; do
  python3 scripts/evaluate_image_quality.py \
    --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
    --test eval_dataset/occlusion/$OCC/ \
    --out  results/occlusion_${OCC}.csv
done

python3 scripts/evaluate_ibvs.py \
  --log eval_dataset/capture_log.csv \
  --group_by occlusion \
  --out results/ibvs_by_occlusion.csv
```

---

## SESSION 7 — MULTI-OBJECT SCENES (if time)

**Goal:** Verify ByteTrack handles multiple objects correctly.

| Scene | Setup | Images |
|-------|-------|--------|
| 2 same-class | 2 fire extinguishers side by side | 3 runs |
| Mixed class | 1 gauge + 1 fire ext | 3 runs |
| Front + Back | 1 object front, 1 behind robot | 3 runs |

Copy to: `eval_dataset/multi_object/2_objects/` and `3_objects/`

---

## PAPER LOG TEMPLATE (print this out)

```
Date: _________ | Object: _________________ | Location: ___________

| Run | Distance | Angle | IBVS time (s) | Final error (px) | Converged | Notes |
|-----|----------|-------|---------------|------------------|-----------|-------|
| 1   |          |       |               |                  | Y / N     |       |
| 2   |          |       |               |                  | Y / N     |       |
| 3   |          |       |               |                  | Y / N     |       |
| 4   |          |       |               |                  | Y / N     |       |
| 5   |          |       |               |                  | Y / N     |       |

Gauge true reading (if applicable): _____________
Caption (for VLM images): ___________________________________
```

---

## COMPLETE EVALUATION COMMANDS — ALL AT ONCE

After all sessions, run everything from laptop:

```bash
cd /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws
source eval_venv/bin/activate

# 1. Image quality — all angle folders
for ANGLE in 0deg 15deg_L 15deg_R 30deg_L 30deg_R 45deg_L 45deg_R; do
  python3 scripts/evaluate_image_quality.py \
    --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
    --test eval_dataset/angle_eval/horizontal/$ANGLE/ \
    --out  results/image_quality_$ANGLE.csv
done

# 2. Image quality — distance (use gauge reference)
for DIST in 1m 2m 3m 4m; do
  python3 scripts/evaluate_image_quality.py \
    --ref  eval_dataset/reference/gauge_ref_01.jpg \
    --test eval_dataset/distance_eval/$DIST/ \
    --out  results/image_quality_dist_$DIST.csv
done

# 3. Image quality — occlusion
for OCC in 0pct 25pct 50pct 75pct; do
  python3 scripts/evaluate_image_quality.py \
    --ref  eval_dataset/reference/fire_ext_ref_01.jpg \
    --test eval_dataset/occlusion/$OCC/ \
    --out  results/image_quality_occ_$OCC.csv
done

# 4. IBVS stats (grouped different ways)
python3 scripts/evaluate_ibvs.py --log eval_dataset/capture_log.csv --group_by angle    --out results/ibvs_by_angle.csv
python3 scripts/evaluate_ibvs.py --log eval_dataset/capture_log.csv --group_by distance --out results/ibvs_by_distance.csv
python3 scripts/evaluate_ibvs.py --log eval_dataset/capture_log.csv --group_by occlusion --out results/ibvs_by_occlusion.csv

# 5. Gauge pipeline (server must be running)
python3 scripts/evaluate_gauge.py \
  --images eval_dataset/gauge_accuracy/ \
  --log    eval_dataset/capture_log.csv \
  --server http://localhost:8001 \
  --out    results/gauge_eval_results.csv

# 6. VLM evaluation (server must be running)
python3 scripts/evaluate_vlm.py \
  --base   eval_dataset/vlm_eval/ \
  --log    eval_dataset/capture_log.csv \
  --server http://localhost:8001 \
  --out    results/vlm_eval_results.csv

# 7. LLM Judge
export GEMINI_API_KEY=your_key_here
python3 scripts/evaluate_vlm_llm_judge.py \
  --results results/vlm_eval_results.csv \
  --images  eval_dataset/vlm_eval/ \
  --out     results/llm_judge_results.csv
```

---

## WHAT JETSON CAPTURES AUTOMATICALLY

Every pipeline run saves to (NO manual intervention):
```
~/Documents/Visual_Inspection_ws/captures/inspection/SESSION_TS/
  fire_extinguisher/instance_1/
    img_01.jpg   ← Logitech close-up (after 10s autofocus)
    img_02.jpg
    img_03.jpg
    img_04.jpg
    overview_01.jpg  ← Insta360 with YOLO boxes
```

**Best image per run = img_01.jpg** (most time for autofocus, sharpest)

---

## MINIMUM VIABLE DATASET (if time runs out)

Priority if you must cut sessions short:
1. ✅ Reference images (1m, 0°) — MUST HAVE
2. ✅ Angle: 0°, 15°, 30° at 2m — professor requirement
3. ✅ Distance: 1m, 2m, 3m (skip 4m if no time)
4. ✅ Gauge ground truth: 5 reading positions × 3 images
5. ✅ Fire ext PASS + FAIL — 5 each minimum
6. Skip occlusion and multi-object if no time

---

*Guide version: 2026-03-12 | Jetson stays clean — no code changes needed*
