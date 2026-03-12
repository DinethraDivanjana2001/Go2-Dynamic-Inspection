# Data Collection Guide

Everything is handled by ONE script. No manual work.

---

## Start These 4 Terminals

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
# Terminal 4 — the only one you interact with
python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
```

---

## What Happens in Terminal 4

1. A menu appears — pick a session number
2. The script tells you exactly what to set up (angle, distance, scene)
3. Press **ENTER** when robot is in position
4. Script runs the inspection pipeline automatically
5. Script copies the image to the correct folder automatically
6. Script writes to capture_log.csv automatically
7. Repeat for next image

**You do nothing manually. No copying. No CSV editing.**

---

## Sessions

**Session 1 — Reference**
Pick object (fire_ext / gauge / door) → robot at **1m, facing directly** → press ENTER × 5

**Session 2 — Angle**
Pick angle from menu → robot at **2m, that angle** → press ENTER × 10
Angles: 0° → 15°L → 15°R → 30°L → 30°R → 45°L → 45°R

**Session 3 — Distance**
Pick distance → robot at **0° head-on, that distance** → press ENTER × 10
Distances: 1m → 2m → 3m → 4m

**Session 4 — Gauge Ground Truth**
Type the **true reading value** when asked → robot at 2m, 0° → press ENTER × 3
Do this for 5 different reading positions (low, 25%, mid, 75%, high)

**Session 5 — VLM Images**
Pick scenario → set up the scene → press ENTER × 10
After each capture: type **1 sentence** describing what the camera sees
Scenarios: fire_ext_pass, fire_ext_fail, exit_pass, exit_fail, door_pass, door_fail

**Session 6 — Occlusion**
Pick level → cover that fraction of object → press ENTER × 10
Levels: 0% → 25% → 50% → 75%

**Session 7 — Multi-object**
Pick scene → arrange objects → press ENTER × 5

---

## Checklist

```
[ ] Reference:  fire_ext, gauge, door  (5 images each)
[ ] Angle:      0° 15°L 15°R 30°L 30°R 45°L 45°R  (10 each)
[ ] Distance:   1m 2m 3m 4m  (10 each, use gauge)
[ ] Gauge GT:   5 reading positions × 3 = 15 images
[ ] VLM:        fire_ext_pass/fail, exit_pass/fail, door_pass  (10 each)
[ ] Occlusion:  0% 25% 50% 75%  (10 each)
[ ] Multi-obj:  (if time, 5 images)
```

---

## Copy to Laptop After

```bash
scp -r rgen@192.168.8.181:~/Documents/Visual_Inspection_ws/evaluation/ \
    /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/
```
