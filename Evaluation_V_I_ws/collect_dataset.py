#!/usr/bin/env python3
"""
collect_dataset.py  —  Visual Inspection Dataset Collector
Run on Jetson as Terminal 4. Terminals 1,2,3 must already be running.

python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py
"""

import os, sys, re, time, csv, shutil, subprocess
from pathlib import Path
from datetime import datetime

# ── Paths ──────────────────────────────────────────────────────
BASE      = Path.home() / 'Documents/Visual_Inspection_ws'
EVAL      = BASE / 'evaluation'
CAPTURES  = BASE / 'captures/inspection'
LOG_CSV   = EVAL / 'capture_log.csv'
SCRIPT    = BASE / 'test_scripts/test_full_pipeline.py'
ROS_CMD   = (f'bash -c "source /opt/ros/humble/setup.bash && '
             f'source {BASE}/inspection_ws/install/setup.bash && '
             f'python3 {SCRIPT}"')

# ── How many images per position ─────────────────────────────
IMAGES_PER_ANGLE    = 10   # angle eval
IMAGES_PER_DISTANCE = 10   # distance eval
IMAGES_PER_GAUGE    = 3    # per reading position
IMAGES_PER_VLM      = 10   # per scenario
IMAGES_PER_OCCLUSION= 10   # per level
IMAGES_PER_REF      = 5    # reference images

# ── Colors ─────────────────────────────────────────────────────
R='\033[1;31m'; G='\033[1;32m'; Y='\033[1;33m'
B='\033[1;34m'; C='\033[1;36m'; W='\033[1m'; X='\033[0m'

def div(c='─', n=60): print(c*n)
def hdr(t, color=C): print(f'\n{color}{"═"*60}\n  {t}\n{"═"*60}{X}')
def step(n, t): print(f'\n{W}  [{n}] {t}{X}')
def ok(t):  print(f'{G}  ✓ {t}{X}')
def bad(t): print(f'{R}  ✗ {t}{X}')
def info(t):print(f'{Y}  → {t}{X}')
def clr():  os.system('clear')

# ── CSV ────────────────────────────────────────────────────────
COLS = ['timestamp','folder','filename','object_type','distance_m',
        'angle_deg','angle_direction','occlusion_pct','n_objects',
        'ibvs_time_s','final_error_px','converged','ground_truth_value','notes']

def init_log():
    EVAL.mkdir(parents=True, exist_ok=True)
    if not LOG_CSV.exists():
        with open(LOG_CSV,'w',newline='') as f:
            csv.writer(f).writerow(COLS)

def log_row(**kw):
    kw.setdefault('timestamp', datetime.now().strftime('%Y-%m-%d_%H:%M:%S'))
    with open(LOG_CSV,'a',newline='') as f:
        csv.DictWriter(f,fieldnames=COLS).writerow(kw)

def count_rows():
    if not LOG_CSV.exists(): return 0
    return sum(1 for _ in open(LOG_CSV))-1

# ── Run pipeline, parse ibvs stats ─────────────────────────────
def run_and_parse():
    """Run inspection pipeline, auto-parse IBVS stats from output."""
    before = set()
    if CAPTURES.exists():
        before = {f for f in CAPTURES.rglob('img_*.jpg')}

    info('Launching inspection pipeline...')
    ibvs_time, final_err, converged = 0.0, 0.0, False
    output_lines = []

    try:
        proc = subprocess.Popen(ROS_CMD, shell=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        for line in proc.stdout:
            l = line.strip()
            if l: print(f'     {l}')
            output_lines.append(l)
            # Parse converge line: "IBVS converged: err=7.8px at iter 38"
            m = re.search(r'IBVS converged.*err=([\d.]+)px.*iter\s+(\d+)', l)
            if m:
                final_err   = float(m.group(1))
                ibvs_time   = round(int(m.group(2)) * 0.066, 1)
                converged   = True
            # Parse timeout: "IBVS timeout after 40.0s"
            m2 = re.search(r'IBVS timeout', l)
            if m2:
                converged = False
                ibvs_time = 40.0
            if 'success=True' in l or 'success=False' in l or 'RESULT' in l:
                break
        proc.wait(timeout=5)
    except Exception as e:
        bad(f'Pipeline error: {e}')

    # Find new image
    time.sleep(1)
    after = set()
    if CAPTURES.exists():
        after = {f for f in CAPTURES.rglob('img_*.jpg')}
    new = after - before
    img_path = None
    if new:
        newest = max(new, key=lambda f: f.stat().st_mtime)
        img01  = newest.parent / 'img_01.jpg'
        img_path = img01 if img01.exists() else newest

    return img_path, ibvs_time, final_err, converged

def save_image(img_path, dest_dir, filename):
    dest_dir.mkdir(parents=True, exist_ok=True)
    dst = dest_dir / filename
    if img_path and img_path.exists():
        shutil.copy2(img_path, dst)
        ok(f'Saved → {dst.relative_to(EVAL)}')
        return True
    bad(f'No image found to copy')
    return False

def next_n(folder, prefix='img_'):
    existing = list(folder.glob(f'{prefix}*.jpg'))
    if not existing: return 1
    nums = []
    for f in existing:
        m = re.search(r'(\d+)', f.stem.replace(prefix,''))
        if m: nums.append(int(m.group(1)))
    return max(nums)+1 if nums else 1

def capture_loop(dest_folder, subfolder_name, obj_type, distance, angle_deg,
                 angle_dir, occlusion, n_objects, n_images,
                 ground_truth='N/A', ask_caption=False, filename_prefix='img_'):
    """Generic capture loop used by all sessions."""
    dest = EVAL / subfolder_name
    dest.mkdir(parents=True, exist_ok=True)
    captured = 0

    for i in range(n_images):
        div()
        print(f'{C}  Image {i+1} of {n_images}{X}')
        input(f'{W}  ▶  Position robot. Press ENTER to capture...{X}')
        print()

        img, ibvs_t, err_px, conv = run_and_parse()
        n     = next_n(dest, prefix=filename_prefix)
        fname = f'{filename_prefix}{n:02d}.jpg'

        if save_image(img, dest, fname):
            captured += 1

        if conv:
            ok(f'IBVS converged in {ibvs_t}s  |  final error = {err_px}px')
        else:
            bad(f'IBVS did NOT converge (timeout)')

        caption = ''
        if ask_caption:
            caption = input(f'{Y}  Caption (1 sentence what camera sees): {X}').strip()

        log_row(folder=subfolder_name, filename=fname, object_type=obj_type,
                distance_m=distance, angle_deg=angle_deg, angle_direction=angle_dir,
                occlusion_pct=occlusion, n_objects=n_objects,
                ibvs_time_s=ibvs_t, final_error_px=err_px, converged=conv,
                ground_truth_value=ground_truth, notes=caption)

        ok(f'Logged  [{captured}/{n_images} captured so far]')
        print()

    return captured

# ── Sessions ────────────────────────────────────────────────────

def session_reference():
    hdr('SESSION 1 — REFERENCE IMAGES')
    print(f'''
  PURPOSE: Gold-standard baseline images for ALL quality metrics.

  SETUP:
   • Pick 1 object
   • Place robot at exactly 1 metre, facing it directly
   • Good even lighting (no glare)

  OBJECTS:  1=fire_extinguisher  2=gauge  3=door  4=emergency_exit
''')
    objs = {'1':'fire_extinguisher','2':'gauge','3':'door','4':'emergency_exit'}
    obj = objs.get(input('  Select object: ').strip(), 'fire_extinguisher')
    prefix = obj.split('_')[0]+'_ref_'

    info(f'Collecting {IMAGES_PER_REF} reference images for {obj}')
    info('Robot: 1m distance, 0° angle, head-on')

    capture_loop(EVAL/'reference', 'reference', obj,
                 distance='1.0', angle_deg='0', angle_dir='center',
                 occlusion='0', n_objects='1',
                 n_images=IMAGES_PER_REF,
                 filename_prefix=prefix)

def session_angle():
    hdr('SESSION 2 — ANGLE EVALUATION  (Professor Required)')
    angles_h = ['0deg','15deg_L','15deg_R','30deg_L','30deg_R','45deg_L','45deg_R']
    angles_v = ['0deg','15deg_up','15deg_down','30deg_up','30deg_down']

    print(f'''
  OBJECT: fire_extinguisher (keep it fixed on wall)
  DISTANCE: 2m for all angles  ← DO NOT CHANGE
  MOVE: only the robot changes angle

  HORIZONTAL ANGLES:
    1: 0deg   (head-on, directly facing)
    2: 15deg_L  3: 15deg_R
    4: 30deg_L  5: 30deg_R
    6: 45deg_L  7: 45deg_R

  VERTICAL ANGLES:
    8: 0deg      9: 15deg_up   10: 15deg_down
   11: 30deg_up  12: 30deg_down
''')
    all_angles = angles_h + angles_v
    choice = int(input('  Select angle (1-12): ').strip()) - 1
    ang = all_angles[choice]

    if choice < len(angles_h):
        subfolder = f'angle_eval/horizontal/{ang}'
        angle_deg = re.sub(r'[^0-9]','',ang)
        angle_dir = 'L' if 'L' in ang else ('R' if 'R' in ang else 'center')
    else:
        subfolder = f'angle_eval/vertical/{ang}'
        angle_deg = re.sub(r'[^0-9]','',ang)
        angle_dir = 'up' if 'up' in ang else ('down' if 'down' in ang else 'center')

    info(f'Collecting {IMAGES_PER_ANGLE} images at {ang}')
    info(f'Robot position: 2m away, {ang} offset')

    capture_loop(EVAL/subfolder, subfolder, 'fire_extinguisher',
                 distance='2.0', angle_deg=angle_deg, angle_dir=angle_dir,
                 occlusion='0', n_objects='1',
                 n_images=IMAGES_PER_ANGLE)

def session_distance():
    hdr('SESSION 3 — DISTANCE EVALUATION')
    print(f'''
  OBJECT: gauge  (best for testing OCR readability)
  ANGLE: 0° head-on  ← DO NOT CHANGE
  CHANGE: only the robot distance

  1: 1m    2: 2m    3: 3m    4: 4m
  (4m expected to show blurry images — that is intentional)
''')
    dists = ['1','2','3','4']
    choice = int(input('  Select distance (1-4): ').strip()) - 1
    dist = dists[choice]
    subfolder = f'distance_eval/{dist}m'

    info(f'Collecting {IMAGES_PER_DISTANCE} images at {dist}m')
    info('Robot: 0° angle, head-on facing gauge')

    capture_loop(EVAL/subfolder, subfolder, 'gauge',
                 distance=dist+'.0', angle_deg='0', angle_dir='center',
                 occlusion='0', n_objects='1',
                 n_images=IMAGES_PER_DISTANCE)

def session_gauge():
    hdr('SESSION 4 — GAUGE ACCURACY GROUND TRUTH')
    print(f'''
  PURPOSE: True reading value for EVERY image — used for MAE/RMSE eval.

  SETUP:
   • Real pressure/water/compound gauge
   • Set pointer to a known position (use tape to fix pointer)
   • Write down the EXACT true reading before each set
   • Distance: 2m, Angle: 0°

  READING POSITIONS TO COLLECT:
   • Low   (~10% of scale)
   • 25%   of scale
   • Mid   (50%)
   • 75%   of scale
   • High  (~90% of scale)
   → 3 images per position = 15 images total minimum
''')
    true_val = input(f'  {W}Enter TRUE gauge reading (e.g. 2.5): {X}').strip()
    info(f'Collecting {IMAGES_PER_GAUGE} images with true value = {true_val}')
    info('Robot: 2m distance, 0° angle')

    for i in range(IMAGES_PER_GAUGE):
        div()
        print(f'{C}  Image {i+1} of {IMAGES_PER_GAUGE} | gauge reading = {true_val}{X}')
        input(f'{W}  ▶  Position robot. Press ENTER to capture...{X}')

        img, ibvs_t, err_px, conv = run_and_parse()
        fname = f'gauge_{true_val}_{2}m_n{i+1}.jpg'
        dest  = EVAL / 'gauge_accuracy'

        if save_image(img, dest, fname):
            if conv: ok(f'IBVS {ibvs_t}s | err {err_px}px')
            else:    bad('IBVS timeout')
            log_row(folder='gauge_accuracy', filename=fname, object_type='gauge',
                    distance_m='2.0', angle_deg='0', angle_direction='center',
                    occlusion_pct='0', n_objects='1',
                    ibvs_time_s=ibvs_t, final_error_px=err_px, converged=conv,
                    ground_truth_value=true_val,
                    notes=f'gauge reading {true_val}')
            ok('Logged')

def session_vlm():
    hdr('SESSION 5 — VLM PASS/FAIL IMAGES')
    folders = {
        '1': ('fire_ext_pass',  'fire_extinguisher', 'PASS'),
        '2': ('fire_ext_fail',  'fire_extinguisher', 'FAIL'),
        '3': ('exit_pass',      'emergency_exit',    'PASS'),
        '4': ('exit_fail',      'emergency_exit',    'FAIL'),
        '5': ('door_pass',      'door',              'PASS'),
        '6': ('door_fail',      'door',              'FAIL'),
        '7': ('cylinder_pass',  'main_cylinder',     'PASS'),
        '8': ('cylinder_fail',  'main_cylinder',     'FAIL'),
    }
    setups = {
        '1': 'Extinguisher on wall, NOTHING blocking it',
        '2': 'Place large box/chair DIRECTLY in front blocking access',
        '3': 'Door/exit with COMPLETELY clear walkway',
        '4': 'Stack 2-3 chairs or boxes blocking the door',
        '5': 'Regular door — open state',
        '6': 'Regular door — blocked by objects',
        '7': 'Machinery/cylinder — floor dry, no leak',
        '8': 'Pour small amount of water near cylinder base (simulates oil leak)',
    }
    captions = {
        '1': 'fire extinguisher on wall bracket, fully accessible, clear path in front',
        '2': 'fire extinguisher on wall, large cardboard box blocking access path',
        '3': 'emergency exit door with completely clear walkway, no obstructions',
        '4': 'emergency exit door blocked by stacked chairs placed in front',
        '5': 'door in open state, access clear',
        '6': 'door blocked by objects',
        '7': 'cylinder area, floor dry and clean, no leak visible',
        '8': 'water puddle on floor near cylinder base simulating oil leak',
    }
    print()
    for k,(f,o,d) in folders.items():
        print(f'  {k}: {f:20s}  ({o}, expected={d})')
    print()
    choice = input('  Select scenario: ').strip()
    if choice not in folders:
        bad('Invalid choice'); return

    folder_name, obj_type, expected = folders[choice]
    subfolder = f'vlm_eval/{folder_name}'

    print(f'''
{Y}  SETUP FOR THIS SCENARIO:
  {setups[choice]}{X}

  Distance: 2m | Angle: 0°
  Caption suggestion: "{captions[choice]}"
  (edit caption when asked if needed)
''')
    input('  Press ENTER when scene is set up...')
    info(f'Collecting {IMAGES_PER_VLM} images for {folder_name}')

    capture_loop(EVAL/subfolder, subfolder, obj_type,
                 distance='2.0', angle_deg='0', angle_dir='center',
                 occlusion='0', n_objects='1',
                 n_images=IMAGES_PER_VLM,
                 ground_truth=expected,
                 ask_caption=True,
                 filename_prefix=f'{folder_name}_')

def session_occlusion():
    hdr('SESSION 6 — OCCLUSION EVALUATION')
    levels = {'1':('0pct','0','Nothing covering object — full visibility'),
              '2':('25pct','25','Cover BOTTOM QUARTER with tape/cardboard'),
              '3':('50pct','50','Cover BOTTOM HALF with tape/cardboard'),
              '4':('75pct','75','Cover THREE-QUARTERS (expect detection to fail)')}
    print()
    for k,(f,p,d) in levels.items():
        print(f'  {k}: {f}  — {d}')
    print()
    choice = input('  Select occlusion level: ').strip()
    if choice not in levels:
        bad('Invalid choice'); return

    folder_name, pct, setup_desc = levels[choice]
    subfolder = f'occlusion/{folder_name}'

    print(f'''
{Y}  SETUP:
  Object:   fire_extinguisher (2m away, 0° angle)
  Occlusion: {setup_desc}{X}
''')
    input('  Apply occlusion to object. Press ENTER when ready...')
    info(f'Collecting {IMAGES_PER_OCCLUSION} images at {folder_name} occlusion')

    capture_loop(EVAL/subfolder, subfolder, 'fire_extinguisher',
                 distance='2.0', angle_deg='0', angle_dir='center',
                 occlusion=pct, n_objects='1',
                 n_images=IMAGES_PER_OCCLUSION)

def session_multi():
    hdr('SESSION 7 — MULTI-OBJECT')
    scenes = {
        '1': ('2_objects', '2 fire extinguishers side by side'),
        '2': ('2_objects', '1 gauge + 1 fire extinguisher in same view'),
        '3': ('2_objects', '1 object front half + 1 object back half of Insta360'),
        '4': ('3_objects', '3 gauges across a panel'),
    }
    print()
    for k,(f,d) in scenes.items():
        print(f'  {k}: {d}')
    print()
    choice = input('  Select scene: ').strip()
    folder_name, desc = scenes.get(choice,('2_objects','multi-object scene'))
    n_obj = '3' if '3_objects' in folder_name else '2'

    print(f'\n{Y}  SETUP: {desc}{X}\n')
    input('  Arrange the scene. Press ENTER when ready...')

    capture_loop(EVAL/f'multi_object/{folder_name}',
                 f'multi_object/{folder_name}', desc,
                 distance='2.0', angle_deg='0', angle_dir='center',
                 occlusion='0', n_objects=n_obj, n_images=5)

# ── Main menu ────────────────────────────────────────────────────

MENU = [
    ('1', 'Reference images      (1m, 0°, 5 images per object)'),
    ('2', 'Angle evaluation      (2m fixed, 10 images per angle)'),
    ('3', 'Distance evaluation   (0° fixed, 10 images per distance)'),
    ('4', 'Gauge ground truth    (3 images per reading value)'),
    ('5', 'VLM PASS/FAIL images  (10 images per scenario)'),
    ('6', 'Occlusion evaluation  (10 images per level)'),
    ('7', 'Multi-object scenes   (5 images per scene)'),
    ('q', 'Quit'),
]

def main():
    init_log()
    while True:
        clr()
        hdr('VISUAL INSPECTION — DATASET COLLECTION', color=B)
        print(f'''
  {W}MAKE SURE THESE ARE ALREADY RUNNING:{X}
    Terminal 1: ros2 run visual_inspection_ros camera_node
    Terminal 2: ros2 run visual_inspection_ros servo_node
    Terminal 3: ros2 run visual_inspection_ros ibvs_action_server

  {Y}Images logged so far: {count_rows()}{X}
  {Y}Save location: {EVAL}{X}
''')
        div()
        print(f'  {W}SELECT SESSION:{X}')
        for k,label in MENU:
            print(f'    {C}{k}{X}: {label}')
        print()
        choice = input('  ▶ Your choice: ').strip().lower()

        fn = {'1':session_reference,'2':session_angle,'3':session_distance,
              '4':session_gauge,    '5':session_vlm, '6':session_occlusion,
              '7':session_multi}.get(choice)

        if choice == 'q':
            break
        elif fn:
            fn()
            print()
            input(f'\n{G}  Session done! Press ENTER to return to menu...{X}')
        else:
            bad('Invalid choice')
            time.sleep(1)

    clr()
    hdr('COLLECTION COMPLETE', color=G)
    print(f'  Log file : {LOG_CSV}')
    print(f'  Images   : {EVAL}')
    print(f'  Total    : {count_rows()} images logged\n')
    print('  SCP to laptop:')
    print(f'  scp -r rgen@192.168.8.181:{EVAL} \\')
    print('      /home/dinethra/Jetson_orin_nano/Evaluation_V_I_ws/eval_dataset/')

if __name__ == '__main__':
    main()
