# Gauge Reading Pipeline (The Core Method)

## **What I Built**
A hybrid **Deep Learning + Geometry** pipeline that reads analog gauges with 96% accuracy.

**Why hybrid?** End-to-end neural networks are black boxes. My approach is explainable and debuggable.

---

## **The 5-Step Pipeline**

### **1. Preprocessing**
- Resize to 224x224
- Apply CLAHE (contrast enhancement) to make needle visible
- Normalize for model input

### **2. Keypoint Detection (Vision Transformer)**
**What it does:** Finds 4 critical points on the gauge

**Model:** ViTPose (Vision Transformer for Pose Estimation)
- **Architecture:** ViT-Base (12 Transformer layers, 768 hidden dim, 12 attention heads)
- **Why Transformer?** 
  - Self-attention sees ENTIRE gauge at once (global receptive field)
  - Better than CNNs at understanding relationships between keypoints
  - Handles occlusion better (if needle tip hidden, uses context from other points)
  
**Output:** 4 (x,y) coordinates
  - Center (needle pivot)
  - Needle Tip
  - Scale Start (min value)
  - Scale End (max value)

**Training:**
- Loss: Wing Loss (more accurate than MSE for small errors)
- Data: 500 images → 2000 with augmentation
- Optimizer: AdamW (lr=1e-4, weight decay=0.01)
- Framework: PyTorch + timm library

### **3. Needle Segmentation**
**What it does:** Creates a pixel-perfect mask of the needle

**Models I tried:**
- **YOLOv8-Seg:** Fast (10ms) - used in production
- **SegFormer (Transformer):** More accurate but slower (50ms) - tested for research

**Why both?** Backup system - if keypoints fail, segmentation provides the needle shape.

### **4. Scale Reading (Dual OCR)**
**The Challenge:** One OCR engine isn't reliable enough.

**My Solution:** Run TWO models
- **PaddleOCR:** Fast, good for industrial fonts
- **EasyOCR:** Slower but handles rotated text

**Logic:**
```
1. Run PaddleOCR first
2. If confidence > 90% → done
3. Else run EasyOCR too
4. Vote/pick higher confidence
```

**Special Problem:** Decimal points
- Standard OCR misses tiny dots → reads "5.2" as "52"
- **My Fix:** Morphological filter to detect small circular blobs between digits
- Accuracy boost: 80% → 96%

### **5. Angle-to-Value Mapping (RANSAC)**
**The Problem:** OCR sometimes gives wrong numbers (reads '8' as '0')

**My Solution:** RANSAC (Random Sample Consensus)
- Fits a line through good data points
- Automatically ignores outliers
- Works even if 30% of OCR is wrong

**Final Formula:** `Value = Slope × Angle + Intercept`

---

## **Complete Tech Stack Breakdown**

### **Frameworks & Their Purposes**

| Framework | Used For | Why This Choice |
|-----------|----------|----------------|
| **PyTorch** | Training all deep learning models | Industry standard, great debugging tools |
| **timm** | Vision Transformer implementation | Pre-trained ViT models, easy to fine-tune |
| **Ultralytics** | YOLOv8/YOLOv11 object detection & segmentation | Best YOLO implementation, active community |
| **Albumentations** | Image augmentation pipeline | Fastest augmentation library, GPU support |
| **OpenCV** | Geometric operations, CLAHE, ellipse fitting | Speed - optimized C++ backend |
| **NumPy** | Matrix operations, coordinate transforms | Essential for geometric calculations |
| **scikit-learn** | RANSAC regression | Robust outlier rejection |
| **PaddleOCR** | Primary OCR engine | Optimized for industrial fonts |
| **EasyOCR** | Backup OCR engine | Better rotation handling |
| **scikit-image** | Morphological operations | Decimal point blob detection |
| **TensorRT** | Model optimization for Jetson | 3x speedup on edge device |

### **Techniques & Their Purposes**

| Technique | Used For | Impact |
|-----------|----------|--------|
| **Self-Attention (Transformer)** | Keypoint detection | Global context understanding |
| **CLAHE** | Contrast enhancement | Makes needle visible in low light |
| **Ellipse Fitting** | Perspective correction | Handles camera angle distortion |
| **Orthogonal Distance Regression** | Needle angle from segmentation mask | Robust to noise |
| **RANSAC** | Outlier rejection in OCR-angle mapping | Handles 30% bad data |
| **Morphological Closing** | Decimal point detection | Finds small circular blobs |
| **Wing Loss** | Training keypoint model | Sub-pixel accuracy (better than MSE) |
| **Data Augmentation** | Model robustness | Handles blur, rotation, lighting changes |
| **Ensemble Voting** | OCR reliability | Combines two models for accuracy |
| **TensorRT FP16** | Edge inference speedup | 3x faster, no accuracy loss |

---

## **Visual Pipeline Flow**

```
Input Image
    ↓
[1. CLAHE Enhancement]
    ↓
[2. ViTPose (Transformer)] → 4 keypoints detected
    ↓
[3. YOLOv8-Seg] → Needle mask
    ↓
[4. PaddleOCR + EasyOCR] → "0", "10", "20" detected
    ↓
[5. RANSAC Fitting] → Value = f(Angle)
    ↓
Output: 5.2 Bar ✓
```

---

## **Interview Talking Points**

### 1. "Why not end-to-end deep learning?"
> "Black box models can't debug. With my pipeline, if it fails, I know exactly where: keypoints? OCR? This modularity is critical for industrial reliability."

### 2. "What Vision Transformer architecture did you use?"
> "ViTPose with ViT-Base backbone. It's a Vision Transformer adapted for keypoint detection. The self-attention mechanism allows it to see the entire gauge at once, understanding relationships between all keypoints simultaneously. This is superior to CNNs which only see local patches through convolution."

### 3. "Why Vision Transformer over CNN like ResNet?"
> "Global receptive field. If the needle tip is partially hidden, a CNN only sees that local region and fails. The Transformer's self-attention looks at the ENTIRE gauge - it can infer 'the tip must be here' based on center position and gauge structure. This improved occlusion robustness by 20%."

### 4. "What frameworks did you use and why?"
> "PyTorch for training (flexibility), timm for pre-trained ViT (saves weeks of training), Ultralytics for YOLO (production-ready), Albumentations for fast augmentation, scikit-learn for RANSAC (robust regression), and TensorRT for edge optimization (3x speedup)."

### 5. "Why use TWO OCR models?"
> "Redundancy. PaddleOCR is fast for 80% of cases. EasyOCR catches the hard 20%. Cascading them gives best of both: speed AND accuracy."

### 6. "What is RANSAC?"
> "It's like democracy for data. If 100 numbers are detected and 20 are wrong, RANSAC finds the line that fits the 80 correct ones and ignores the outliers."

### 7. "How do you handle decimal points?"
> "OCR engines miss tiny dots. I added a morphological filter that looks for small circular blobs between digits. If found, insert the decimal. This was a custom solution I developed."

### 8. "What's your accuracy?"
> "96% within ±2% of true value. The 4% failures trigger fallback to VLM (Gemini) for a second opinion."

### 9. "Why ellipse fitting?"
> "Cameras view gauges at angles, so circles appear as ellipses. Fitting the correct ellipse removes perspective distortion for accurate angle calculation."

### 10. "What augmentation techniques did you use?"
> "Albumentations pipeline with: RandomRotate (±45°), GaussianNoise (sensor noise), MotionBlur (robot movement), RandomBrightnessContrast (lighting changes), and CoarseDropout (simulating occlusions). This made the model robust to real-world conditions."

### 11. "Explain the complete flow from image to reading."
> "First, CLAHE enhances contrast. Second, ViTPose detects 4 keypoints defining gauge geometry. Third, YOLOv8-Seg creates a needle mask as backup. Fourth, dual OCR (Paddle + Easy) reads scale numbers. Fifth, RANSAC fits a robust line from angles to values. Finally, we calculate: Reading = Slope × NeedleAngle + Intercept."

### 12. "What's the most complex technique you implemented?"
> "The morphological decimal point detector. I had to threshold the image, find contours, filter by circularity and area, then validate spatial proximity to digits. It required tuning 5 hyperparameters but boosted accuracy 16%."
