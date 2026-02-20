# Jetson Orin Nano - IBVS Visual Servoing System

## 📦 Workspace Contents

This is a self-contained workspace for deploying the IBVS visual servoing system on Jetson Orin Nano.

### Directory Structure

```
jetson_workspace/
├── ibvs_pipeline.py              # Main IBVS pipeline (two-stage system)
├── test_calibration_live.py      # Test script for calibration validation
├── calibration_config.py         # Insta360 calibration formulas
├── requirements.txt              # Python dependencies
├── README.md                     # This file
├── SETUP.md                      # Detailed setup instructions
├── config/
│   └── logitech_intrinsics.yaml  # Logitech C920 camera calibration
├── weights/
│   └── yolo11n.pt                # YOLO model weights
├── arduino/
│   └── pan_tilt_control.ino      # Arduino servo control code
└── docs/
    ├── IBVS_TUNING_GUIDE.md      # How to tune PID parameters
    ├── IBVS_SYSTEM_SUMMARY.md    # System overview
    └── THE_COMPLETE_STORY.md     # Complete explanation
```

## 🚀 Quick Start

### 1. Transfer to Jetson

On your development machine:
```bash
# Create archive
cd /home/dinethra/Jetson_orin_nano
tar -czf jetson_workspace.tar.gz jetson_workspace/

# Transfer to Jetson (replace with your Jetson's IP)
scp jetson_workspace.tar.gz jetson@<JETSON_IP>:~/
```

On Jetson Orin Nano:
```bash
# Extract
cd ~
tar -xzf jetson_workspace.tar.gz
cd jetson_workspace
```

### 2. Setup Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 3. Hardware Setup

**Connect:**
- Insta360 camera → USB port
- Logitech C920 → USB port  
- Arduino → USB port (should appear as /dev/ttyACM0)

**Upload Arduino code:**
```bash
# Install Arduino CLI (if not already installed)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Upload code
arduino-cli compile --fqbn arduino:avr:uno arduino/pan_tilt_control.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/pan_tilt_control.ino
```

### 4. Verify Setup

```bash
# Check cameras
ls -l /sys/class/video4linux/video*/name

# Test calibration system
python3 test_calibration_live.py
```

### 5. Run IBVS Pipeline

```bash
python3 ibvs_pipeline.py
```

## 📋 System Requirements

**Hardware:**
- Jetson Orin Nano (8GB recommended)
- Insta360 X3 camera
- Logitech C920 webcam
- Arduino Uno + 2x servos (pan-tilt)
- USB hub (if needed)

**Software:**
- JetPack 5.x or 6.x
- Python 3.8+
- CUDA support (for GPU acceleration)

## 🔧 Configuration

### Camera Ports
Edit scripts if camera indices change:
- `ibvs_pipeline.py` - Auto-detects by name
- `test_calibration_live.py` - Auto-detects by name

### Arduino Port
Default: `/dev/ttyACM0`
If different, edit in scripts:
```python
ARDUINO_PORT = '/dev/ttyACM0'  # Change if needed
ARDUINO_BAUD = 9600
```

### PID Tuning
See `docs/IBVS_TUNING_GUIDE.md` for detailed tuning instructions.

Quick tuning (edit `ibvs_pipeline.py` lines 67-75):
```python
IBVS_KP_PAN = 0.12   # Increase for faster, decrease for smoother
IBVS_KI_PAN = 0.002  # Increase to eliminate steady-state error
IBVS_KD_PAN = 0.02   # Increase to reduce oscillations
```

## 🎯 Usage

### Test Calibration
```bash
python3 test_calibration_live.py
```
- Shows dual camera view
- Tests coarse direction (Insta360 → servos)
- Press 'q' to quit

### Run Full IBVS Pipeline
```bash
python3 ibvs_pipeline.py
```
- Stage 1: Insta360 finds object, moves servos (COARSE)
- Stage 2: Logitech centers object precisely (FINE/IBVS)
- Press 'q' to quit

## 📊 Expected Performance

- **Coarse positioning**: 6-9° accuracy
- **Fine centering**: <10 pixels (~1-2°)
- **Convergence time**: 3-5 seconds
- **Frame rate**: ~20-30 FPS
- **Distance range**: 0.5m - 5m (distance-independent)

## 🐛 Troubleshooting

### Cameras not detected
```bash
# Check connected cameras
v4l2-ctl --list-devices

# Check permissions
sudo usermod -aG video $USER
# Logout and login again
```

### Arduino not connecting
```bash
# Check port
ls /dev/ttyACM*

# Fix permissions
sudo usermod -aG dialout $USER
# Logout and login again

# Or temporary fix
sudo chmod 666 /dev/ttyACM0
```

### YOLO model not found
```bash
# Download YOLO weights
cd weights
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
```

### Overshooting / Oscillations
- Reduce `IBVS_KP_PAN` and `IBVS_KP_TILT` (line ~68)
- Reduce `MAX_SPEED_PAN` and `MAX_SPEED_TILT` (line ~303)
- See `docs/IBVS_TUNING_GUIDE.md`

### Object keeps getting lost
- Increase `MAX_SPEED` for faster approach
- Decrease `IBVS_SERVO_DELAY` for faster feedback
- Check lighting conditions

## 📚 Documentation

- **IBVS_TUNING_GUIDE.md** - Complete PID tuning guide
- **IBVS_SYSTEM_SUMMARY.md** - System overview and features
- **THE_COMPLETE_STORY.md** - Full explanation of both stages

## 🔄 Updates

To update from development machine:
```bash
# On dev machine
cd /home/dinethra/Jetson_orin_nano
cp tools/ibvs_pipeline.py jetson_workspace/
tar -czf jetson_workspace_update.tar.gz jetson_workspace/
scp jetson_workspace_update.tar.gz jetson@<JETSON_IP>:~/

# On Jetson
cd ~
tar -xzf jetson_workspace_update.tar.gz
```

## 📝 Notes

- This workspace is self-contained and portable
- All calibration data is included
- Virtual environment keeps dependencies isolated
- Original development files remain untouched

## 🆘 Support

If issues persist:
1. Check all connections (cameras, Arduino, servos)
2. Verify camera indices with `v4l2-ctl --list-devices`
3. Test Arduino separately with Serial Monitor
4. Check YOLO model loads: `python3 -c "from ultralytics import YOLO; YOLO('weights/yolo11n.pt')"`
5. Review console output for specific errors

---

**System ready for deployment! 🚀**
