# Camera Calibration

## **What I Did**
Calibrated the robot's camera to remove lens distortion for accurate gauge measurements.

**Why it matters:** Distortion makes straight lines curved → wrong angle readings → wrong gauge values.

---

## **The Problem: Lens Distortion**

**What happens:**
- Wide-angle lenses bend light
- Straight gauge needles look curved
- Circles look like "squircles"

**Impact:** A 5-degree distortion error = reading "6 Bar" instead of "5 Bar" (danger zone!)

---

## **The Solution: Mathematical Correction**

### What I Calibrated
**Camera Matrix (Intrinsics):**
```
K = | fx   0   cx |
    | 0   fy   cy |
    | 0    0    1 |
```
- `fx, fy`: Focal length (zoom level)
- `cx, cy`: Optical center

**Distortion Coefficients:**
- `k1, k2, k3`: Radial distortion (barrel/pincushion effect)
- `p1, p2`: Tangential distortion (lens tilt)

---

## **My Calibration Process**

### Step 1: Data Collection
- Used checkerboard pattern (8×6 squares)
- Captured **40-50 images** from different angles/distances
- Covered all image corners (where distortion is worst)

### Step 2: Corner Detection
- Algorithm: `cv2.findChessboardCorners`
- Sub-pixel refinement for precision

### Step 3: Optimization
- Algorithm: `cv2.calibrateCamera`
- Minimizes **reprojection error**

---

## **My Results**

| Camera | Distortion Type | Reprojection Error | Quality |
|--------|----------------|-------------------|---------|
| Insta360 | Barrel (k1 = -0.13) | 0.00017 px | Excellent |
| Logitech C920 | Pincushion (k1 = 0.14) | 0.000001 px | Perfect |

**What "Reprojection Error" means:** If error is 0.17 pixels, my math model predicts corner positions accurately to within a fraction of a pixel → very precise.

---

## **Interview Talking Points**

### 1. "What is reprojection error?"
> "It's how wrong my model is. I predict where corners should be, measure where they actually are, and calculate the difference. <0.5 pixels = excellent calibration."

### 2. "Why calibrate if you use deep learning?"
> "Neural networks can handle some distortion, but geometric calculations require linearity. For angle measurement, I need straight lines to BE straight."

### 3. "Radial vs Tangential distortion?"
> "Radial = lens shape bending light (barrel/pincushion). Tangential = lens not parallel to sensor (manufacturing defect). We mostly fix radial."

### 4. "Why 40-50 images? Why not 10?"
> "Need to cover the whole image, especially corners where distortion is strongest. Too few images = poor model fit. 40-50 is the industry sweet spot."

### 5. "What if focus changes?"
> "Technically need recalibration since focal length changes. I use fixed focus (infinity) to keep calibration valid. Auto-focus would break it."

---

## **Tools Used**
- **OpenCV:** `cv2.calibrateCamera`, `cv2.undistort`
- **Kalibr:** Explored for multi-camera systems (stereo)
- **Target:** Checkerboard / Charuco board

---

## **Impact on System**
- Undistortion adds **~2ms** per image (acceptable overhead)
- Applied **before** gauge pipeline (not before YOLO - doesn't need it)
- Enables accurate angle calculation for the geometric method
