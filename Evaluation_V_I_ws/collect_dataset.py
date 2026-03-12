#!/usr/bin/env python3
"""
collect_dataset.py
Run on JETSON — smart dataset collection script.

What it does:
  1. You pick session type (reference/angle/distance/gauge/vlm/occlusion/multi)
  2. You set metadata (angle, distance, object etc.)
  3. Press Enter → runs inspection automatically
  4. Detects new images in captures/ → copies to correct eval folder
  5. Asks ibvs_time + final_error (you read from Terminal 3)
  6. Writes to capture_log.csv automatically
  7. Loops for next image

Run:
  python3 ~/Documents/Visual_Inspection_ws/evaluation/collect_dataset.py

Requirements:
  - ROS2 sourced (Terminals 1, 2, 3 already running)
  - Run this AS Terminal 4
"""

import os, sys, time, csv, shutil, subprocess
from pathlib import Path
from datetime import datetime

# ─── Paths ────────────────────────────────────────────────────────────────────
BASE_WS   = Path.home() / 'Documents/Visual_Inspection_ws'
EVAL_DIR  = BASE_WS / 'evaluation'
CAPTURES  = BASE_WS / 'captures/inspection'
LOG_CSV   = EVAL_DIR / 'capture_log.csv'
TEST_SCRIPT = BASE_WS / 'test_scripts/test_full_pipeline.py'

ROS_SETUP  = 'source /opt/ros/humble/setup.bash'
WS_SETUP   = f'source {BASE_WS}/inspection_ws/install/setup.bash'

# ─── Session types → (folder_base, metadata fields) ──────────────────────────
SESSION_TYPES = {
    '1': ('reference',        'Reference images (1m, 0°)'),
    '2': ('angle_eval',       'Angle evaluation (horizontal/vertical)'),
    '3': ('distance_eval',    'Distance evaluation'),
    '4': ('gauge_accuracy',   'Gauge accuracy ground truth'),
    '5': ('vlm_eval',         'VLM PASS/FAIL images'),
    '6': ('occlusion',        'Occlusion evaluation'),
    '7': ('multi_object',     'Multi-object scenes'),
}

OBJECT_TYPES = {
    '1': 'fire_extinguisher',
    '2': 'gauge',
    '3': 'door',
    '4': 'emergency_exit',
    '5': 'main_cylinder',
}

H_ANGLES  = ['0deg','15deg_L','15deg_R','30deg_L','30deg_R','45deg_L','45deg_R']
V_ANGLES  = ['0deg','15deg_up','15deg_down','30deg_up','30deg_down']
DISTANCES = ['1m','2m','3m','4m']
OCCLUSIONS = ['0pct','25pct','50pct','75pct']

VLM_FOLDERS = {
    '1': 'fire_ext_pass',
    '2': 'fire_ext_fail',
    '3': 'exit_pass',
    '4': 'exit_fail',
    '5': 'door_pass',
    '6': 'door_fail',
    '7': 'cylinder_pass',
    '8': 'cylinder_fail',
}

def clr(): os.system('clear')
def h(t): print(f'\n\033[1;36m{t}\033[0m')
def ok(t): print(f'\033[1;32m  ✓ {t}\033[0m')
def err(t): print(f'\033[1;31m  ✗ {t}\033[0m')
def ask(q, default=''): 
    v = input(f'  {q} [{default}]: ').strip()
    return v if v else default

def ensure_log():
    if not LOG_CSV.exists():
        LOG_CSV.parent.mkdir(parents=True, exist_ok=True)
        with open(LOG_CSV, 'w', newline='') as f:
            csv.writer(f).writerow([
                'timestamp','folder','filename','object_type',
                'distance_m','angle_deg','angle_direction',
                'occlusion_pct','n_objects','ibvs_time_s',
                'final_error_px','converged','ground_truth_value','notes'
            ])

def write_log(row: dict):
    with open(LOG_CSV, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'timestamp','folder','filename','object_type',
            'distance_m','angle_deg','angle_direction',
            'occlusion_pct','n_objects','ibvs_time_s',
            'final_error_px','converged','ground_truth_value','notes'
        ])
        writer.writerow(row)

def get_existing_captures():
    """Snapshot of all existing img_ files in captures/inspection/"""
    files = set()
    if CAPTURES.exists():
        for f in CAPTURES.rglob('img_*.jpg'):
            files.add(f)
    return files

def wait_for_new_capture(before: set, timeout=90):
    """Poll captures/ until a new img_ file appears. Returns Path of newest, or None."""
    deadline = time.time() + timeout
    print('  ⏳ Waiting for inspection to complete and image to appear...', end='', flush=True)
    while time.time() < deadline:
        after = set()
        if CAPTURES.exists():
            for f in CAPTURES.rglob('img_*.jpg'):
                after.add(f)
        new = after - before
        if new:
            print(' done!')
            # Return the img_01.jpg (sharpest) from the newest session
            newest = max(new, key=lambda f: f.stat().st_mtime)
            # Prefer img_01 from same session
            session_dir = newest.parent
            img01 = session_dir / 'img_01.jpg'
            return img01 if img01.exists() else newest
        time.sleep(1)
        print('.', end='', flush=True)
    print(' TIMEOUT!')
    return None

def run_inspection():
    """Run the pipeline. Returns True if launched."""
    cmd = f'bash -c "{ROS_SETUP} && {WS_SETUP} && python3 {TEST_SCRIPT}"'
    print(f'\n  🚀 Running inspection...')
    proc = subprocess.Popen(cmd, shell=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    # Print output until done
    for line in proc.stdout:
        l = line.strip()
        if l:
            print(f'     {l}')
        if 'RESULT' in l or 'success=True' in l or 'success=False' in l:
            break
    proc.wait()
    return True

def get_next_number(dest_dir: Path, prefix='img_') -> int:
    existing = list(dest_dir.glob(f'{prefix}*.jpg'))
    if not existing: return 1
    nums = []
    for f in existing:
        try:
            n = int(f.stem.replace(prefix,'').split('_')[0])
            nums.append(n)
        except: pass
    return max(nums)+1 if nums else 1

def ask_ibvs_stats():
    """Ask user to enter ibvs stats from Terminal 3."""
    print('\n  📋 Check Terminal 3 for IBVS result:')
    print('     Look for: "IBVS converged: err=X.Xpx at iter N"')
    print('     ibvs_time ≈ iter × 0.066  (e.g. iter=38 → 38×0.066 ≈ 2.5s)')
    ibvs_t = ask('ibvs_time_s (converge time in seconds, 0 if timeout)', '0')
    err_px = ask('final_error_px (error shown, 0 if timeout)', '0')
    conv   = ask('converged? (y/n)', 'y').lower().startswith('y')
    return ibvs_t, err_px, conv

# ─── Session handlers ─────────────────────────────────────────────────────────

def session_reference():
    h('SESSION 1 — REFERENCE IMAGES')
    print('  Target: 5 images per object | 1m distance | 0° angle')
    obj = OBJECT_TYPES[ask('Object? 1=fire_ext 2=gauge 3=door 4=exit 5=cylinder', '1')]
    n_runs = int(ask('How many images to capture this run', '5'))

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} ──')
        input('  Position robot at 1m, facing object directly. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)
        if not img:
            err('No image captured — check pipeline'); continue

        dest = EVAL_DIR / 'reference'
        n    = get_next_number(dest, prefix=f'{obj.split("_")[0]}_ref_')
        fname= f'{obj.split("_")[0]}_ref_{n:02d}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → reference/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': 'reference', 'filename': fname, 'object_type': obj,
                   'distance_m': '1.0', 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': '0', 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': 'N/A', 'notes': ask('Notes (optional)', 'reference image 1m 0deg')})
        ok('Written to capture_log.csv')

def session_angle():
    h('SESSION 2 — ANGLE EVALUATION')
    obj = OBJECT_TYPES[ask('Object? 1=fire_ext 2=gauge 3=door', '1')]
    dist= ask('Distance (keep at 2m)', '2.0')

    print('\n  Horizontal angles:')
    for i,a in enumerate(H_ANGLES): print(f'    {i+1}: {a}')
    print('  Vertical angles:')
    for i,a in enumerate(V_ANGLES): print(f'    {i+1+len(H_ANGLES)}: {a} (vertical)')

    choice = ask('Select angle (1-12)', '1')
    idx = int(choice)-1
    if idx < len(H_ANGLES):
        angle_str = H_ANGLES[idx]
        subfolder  = f'angle_eval/horizontal/{angle_str}'
        angle_deg  = angle_str.replace('deg_L','').replace('deg_R','').replace('deg','')
        angle_dir  = 'L' if 'L' in angle_str else ('R' if 'R' in angle_str else 'center')
    else:
        angle_str = V_ANGLES[idx-len(H_ANGLES)]
        subfolder  = f'angle_eval/vertical/{angle_str}'
        angle_deg  = angle_str.replace('deg_up','').replace('deg_down','').replace('deg','')
        angle_dir  = 'up' if 'up' in angle_str else ('down' if 'down' in angle_str else 'center')

    n_runs = int(ask('Number of images', '5'))
    dest   = EVAL_DIR / subfolder

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} | {angle_str} | {dist}m ──')
        input(f'  Position robot at {dist}m, {angle_str}. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)
        if not img:
            err('No image captured'); continue

        n     = get_next_number(dest)
        fname = f'img_{n:02d}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → {subfolder}/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': subfolder, 'filename': fname, 'object_type': obj,
                   'distance_m': dist, 'angle_deg': angle_deg, 'angle_direction': angle_dir,
                   'occlusion_pct': '0', 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': 'N/A', 'notes': ask('Notes', f'{angle_str} {dist}m')})
        ok('Written to capture_log.csv')

def session_distance():
    h('SESSION 3 — DISTANCE EVALUATION')
    obj  = OBJECT_TYPES[ask('Object? 1=fire_ext 2=gauge 3=door', '2')]
    print('\n  Distances: 1=1m  2=2m  3=3m  4=4m')
    dist_key = DISTANCES[int(ask('Select distance', '1'))-1]
    dist_val = dist_key.replace('m','')
    subfolder= f'distance_eval/{dist_key}'
    dest     = EVAL_DIR / subfolder
    n_runs   = int(ask('Number of images', '5'))

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} | 0° | {dist_key} ──')
        input(f'  Position robot at {dist_key}, 0° head-on. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)
        if not img:
            err('No image captured'); continue

        n     = get_next_number(dest)
        fname = f'img_{n:02d}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → {subfolder}/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        readable = ask('Can you read gauge numbers in the image? (y/n)', 'y')
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': subfolder, 'filename': fname, 'object_type': obj,
                   'distance_m': dist_val, 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': '0', 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': 'N/A',
                   'notes': f'readable={readable} | {ask("Notes","")}'})
        ok('Written to capture_log.csv')

def session_gauge():
    h('SESSION 4 — GAUGE ACCURACY GROUND TRUTH')
    print('  Set gauge pointer to a known position. Write down the true value.')
    dist = ask('Distance (m)', '2.0')
    n_runs = int(ask('Number of images for this reading position', '3'))
    true_val = ask('True gauge reading (e.g. 2.5 or 5.0)', '')
    dest = EVAL_DIR / 'gauge_accuracy'

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} | gauge={true_val} | {dist}m ──')
        input('  Position robot. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)
        if not img:
            err('No image captured'); continue

        fname = f'gauge_{true_val}_{dist}m_n{i+1}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → gauge_accuracy/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': 'gauge_accuracy', 'filename': fname, 'object_type': 'gauge',
                   'distance_m': dist, 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': '0', 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': true_val,
                   'notes': ask('Notes (e.g. pointer between 2 and 3)', f'gauge reading {true_val}')})
        ok('Written to capture_log.csv')

def session_vlm():
    h('SESSION 5 — VLM IMAGES')
    print('  Select folder:')
    for k,v in VLM_FOLDERS.items(): print(f'    {k}: {v}')
    folder_key = ask('Choice', '1')
    subfolder  = f'vlm_eval/{VLM_FOLDERS[folder_key]}'
    obj_map    = {'fire_ext_pass':'fire_extinguisher','fire_ext_fail':'fire_extinguisher',
                  'exit_pass':'emergency_exit','exit_fail':'emergency_exit',
                  'door_pass':'door','door_fail':'door',
                  'cylinder_pass':'main_cylinder','cylinder_fail':'main_cylinder'}
    obj = obj_map.get(VLM_FOLDERS[folder_key], 'unknown')
    expected = 'PASS' if 'pass' in VLM_FOLDERS[folder_key] else 'FAIL'
    dist = ask('Distance (m)', '2.0')
    n_runs = int(ask('Number of images', '5'))
    dest = EVAL_DIR / subfolder

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} | {VLM_FOLDERS[folder_key]} ──')
        input('  Set up the scene (e.g. place box to block). Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)
        if not img:
            err('No image captured'); continue

        n     = get_next_number(dest)
        fname = f'{VLM_FOLDERS[folder_key]}_{n:02d}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → {subfolder}/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        caption = ask('Write 1-sentence caption describing exactly what the camera sees',
                      f'{obj} {expected} scenario')
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': subfolder, 'filename': fname, 'object_type': obj,
                   'distance_m': dist, 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': '0', 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': expected, 'notes': caption})
        ok('Written to capture_log.csv')

def session_occlusion():
    h('SESSION 6 — OCCLUSION EVALUATION')
    obj  = OBJECT_TYPES[ask('Object? 1=fire_ext 2=gauge', '1')]
    print('\n  Occlusion levels: 1=0%  2=25%  3=50%  4=75%')
    occ_key = OCCLUSIONS[int(ask('Select occlusion level', '1'))-1]
    occ_pct = occ_key.replace('pct','')
    subfolder= f'occlusion/{occ_key}'
    dest     = EVAL_DIR / subfolder
    n_runs   = int(ask('Number of images', '5'))

    print(f'\n  Cover {occ_pct}% of the object with tape/cardboard.')

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} | occlusion={occ_key} ──')
        input('  Set up occlusion. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before)

        n     = get_next_number(dest)
        fname = f'img_{n:02d}.jpg'
        if img:
            shutil.copy2(img, dest / fname)
            ok(f'Saved → {subfolder}/{fname}')
        else:
            fname = f'NO_CAPTURE_{n:02d}'
            err('No capture — YOLO probably did not detect (expected at high occlusion)')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': subfolder, 'filename': fname, 'object_type': obj,
                   'distance_m': '2.0', 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': occ_pct, 'n_objects': '1',
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': 'N/A', 'notes': ask('Notes', f'{occ_key} covering')})
        ok('Written to capture_log.csv')

def session_multi():
    h('SESSION 7 — MULTI-OBJECT')
    n_obj = ask('How many objects in scene (2/3)', '2')
    subfolder = f'multi_object/{n_obj}_objects'
    dest = EVAL_DIR / subfolder
    n_runs = int(ask('Number of runs', '3'))
    obj = ask('Describe objects (e.g. 2 fire extinguishers)', '2 fire_extinguisher')
    dist = ask('Distance', '2.0')

    for i in range(n_runs):
        print(f'\n  ── Run {i+1}/{n_runs} ──')
        input('  Set up multi-object scene. Press ENTER when ready...')
        before = get_existing_captures()
        run_inspection()
        img = wait_for_new_capture(before, timeout=120)
        if not img:
            err('No image captured'); continue

        n     = get_next_number(dest)
        fname = f'img_{n:02d}.jpg'
        shutil.copy2(img, dest / fname)
        ok(f'Saved → {subfolder}/{fname}')

        ibvs_t, err_px, conv = ask_ibvs_stats()
        write_log({'timestamp': datetime.now().strftime('%Y-%m-%d_%H:%M:%S'),
                   'folder': subfolder, 'filename': fname, 'object_type': obj,
                   'distance_m': dist, 'angle_deg': '0', 'angle_direction': 'center',
                   'occlusion_pct': '0', 'n_objects': n_obj,
                   'ibvs_time_s': ibvs_t, 'final_error_px': err_px, 'converged': conv,
                   'ground_truth_value': 'N/A', 'notes': ask('Notes','multi-object run')})
        ok('Written to capture_log.csv')

# ─── Main menu ────────────────────────────────────────────────────────────────

def main():
    ensure_log()
    while True:
        clr()
        print('='*55)
        print('  VISUAL INSPECTION — DATASET COLLECTION')
        print('='*55)
        print('  Make sure Terminals 1, 2, 3 are running first!')
        print()
        for k,(f,label) in SESSION_TYPES.items():
            print(f'  {k}: {label}')
        print('  q: Quit')
        print()
        print(f'  Log: {LOG_CSV}')
        n_rows = sum(1 for _ in open(LOG_CSV))-1 if LOG_CSV.exists() else 0
        print(f'  Images logged so far: {n_rows}')
        print()

        choice = input('  Select session type: ').strip().lower()
        if choice == 'q': break
        elif choice == '1': session_reference()
        elif choice == '2': session_angle()
        elif choice == '3': session_distance()
        elif choice == '4': session_gauge()
        elif choice == '5': session_vlm()
        elif choice == '6': session_occlusion()
        elif choice == '7': session_multi()
        else:
            print('  Invalid choice'); time.sleep(1); continue

        print()
        cont = input('\n  Continue collecting? (y/n): ').strip().lower()
        if cont != 'y': break

    print('\n  Collection session ended.')
    print(f'  Log saved to: {LOG_CSV}')
    print(f'  Images saved to: {EVAL_DIR}')

if __name__ == '__main__':
    main()
