# Data Collection Guide

---

## Before You Start

Folders already created at:
```
~/Documents/Visual_Inspection_ws/evaluation/
```

Script already at:
```
~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

---

## Every Session — Same 4 Commands

```bash
# Terminal 1
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros camera_node
```

```bash
# Terminal 2
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros servo_node
```

```bash
# Terminal 3
source /opt/ros/humble/setup.bash && source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash
ros2 run visual_inspection_ros ibvs_action_server
```

```bash
# Terminal 4 — THIS IS THE ONLY ONE YOU INTERACT WITH
python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

---

## What the Script Does

- Prints exactly what to set up
- Press **ENTER** to trigger each capture
- Runs inspection automatically
- Saves image to correct folder automatically
- Writes to `capture_log.csv` automatically
- Auto-detects IBVS time and error — you never type it

---

## Sessions — What to Collect

| # | Session | Object | Distance | Angle | Images |
|---|---------|--------|----------|-------|--------|
| 1 | Reference | fire_ext, gauge, door | 1m | 0° head-on | 5 each |
| 2 | Angle | fire_extinguisher | 2m fixed | 0°,15°L,15°R,30°L,30°R,45°L,45°R | 10 each |
| 3 | Distance | gauge | 1m,2m,3m,4m | 0° fixed | 10 each |
| 4 | Gauge ground truth | gauge | 2m | 0° | 3 per reading (15 total) |
| 5 | VLM PASS/FAIL | fire_ext, exit, door | 2m | 0° | 10 each scenario |
| 6 | Occlusion | fire_extinguisher | 2m | 0° | 10 per level |
| 7 | Multi-object | mixed | 2m | 0° | 5 per scene |

---

## Extra Action For VLM Images Only

When collecting VLM images (Session 5), after each capture the script asks:

```
Caption (1 sentence what camera sees):
```

Write exactly what the camera sees. Examples:
- PASS: `fire extinguisher on wall bracket, fully accessible, clear path`
- FAIL: `fire extinguisher on wall, large cardboard box blocking access`
- EXIT FAIL: `emergency exit door blocked by stacked chairs`

---

## Gauge Ground Truth (Session 4)

Before each set of 3 images, script asks:
```
Enter TRUE gauge reading (e.g. 2.5):
```

Collect at 5 positions:
1. Low reading (~10% of scale)
2. 25% of scale
3. Mid (50%)
4. 75% of scale
5. High (~90% of scale)

= **15 images total minimum**

---

## After All Sessions — Copy to Laptop

```bash
# Run on LAPTOP
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/
```

---

## Checklist

```
[ ] Session 1 — Reference:  fire_ext + gauge + door  (5 each)
[ ] Session 2 — Angle:      0° 15°L 15°R 30°L 30°R  (10 each)
[ ] Session 2 — Angle:      45°L 45°R + vertical     (if time)
[ ] Session 3 — Distance:   1m 2m 3m 4m gauge        (10 each)
[ ] Session 4 — Gauge GT:   5 reading positions × 3  (15 total)
[ ] Session 5 — VLM:        fire_ext pass+fail        (10 each)
[ ] Session 5 — VLM:        exit pass+fail, door      (10 each)
[ ] Session 6 — Occlusion:  0% 25% 50% 75%           (10 each)
[ ] Session 7 — Multi-obj:  (if time)
```
