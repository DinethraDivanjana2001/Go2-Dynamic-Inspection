# Object Detection & Edge Optimization

## **What I Built**
Real-time object detection on a Jetson Orin Nano using **YOLOv11-OBB** (Oriented Bounding Boxes) with **TensorRT acceleration**.

---

## **The Key Innovation: OBB (Oriented Boxes)**

### The Problem with Normal YOLO
Standard YOLO uses **horizontal boxes** → wastes space when objects are diagonal.

**Example:**
```
Diagonal Gauge:
❌ Normal Box: [████████]  (50% wasted space)
✅ Rotated Box:   [▰▰▰▰]   (perfect fit)
```

### Why This Matters
- Cleaner crops → Better gauge readings
- Less background noise → 15% accuracy boost

---

## **My Training Results**

| Model | Speed | Accuracy (mAP50) | Notes |
|-------|-------|------------------|-------|
| YOLOv8s-OBB | 35ms | 87.3% | Old baseline |
| **YOLOv11s-OBB** | **25ms** | **93.8%** | **My model** |

**Classes I Trained:**
- Pressure Gauges
- Fire Extinguishers  
- Doors (open/closed)

**Dataset:** 500 images → 2000 after augmentation

---

## **Edge Optimization: TensorRT**

### What I Did
Converted PyTorch model to **TensorRT FP16** for the Jetson.

### Results
- **Before:** 45ms per frame (PyTorch)
- **After:** 15ms per frame (TensorRT)
- **Speedup:** 3x faster ⚡

### How TensorRT Works
1. **Layer Fusion:** Combines operations (Conv + ReLU) into one step
2. **FP16 Precision:** Uses 16-bit instead of 32-bit numbers (2x faster, same accuracy)

**Why not INT8?** Too aggressive - loses accuracy on small objects like gauge needles.

---

## **Implementation Steps**

```bash
# 1. Export PyTorch to ONNX
model.export(format='onnx')

# 2. Convert ONNX to TensorRT
trtexec --onnx=model.onnx --saveEngine=model.engine --fp16

# 3. Run inference
python detect.py --engine model.engine
```

---

## **Interview Talking Points**

### 1. "Why YOLOv11 over v8?"
> "YOLOv11 has a better backbone architecture (C3k2 blocks) that captures finer details like gauge needles. Plus, mAP improved by 6.5%."

### 2. "What is OBB?"
> "Oriented Bounding Boxes - they rotate to match the object angle. Critical for diagonal gauges because normal boxes capture too much background."

### 3. "Explain TensorRT"
> "It's NVIDIA's model optimizer. It fuses layers and uses lower precision (FP16) to run 3x faster on the Jetson without losing accuracy."

### 4. "Why not use MobileNet?"
> "We process on the Edge for filtering only. For accuracy-critical tasks, we need full YOLO precision. The Jetson has enough power for it."

### 5. "How did you handle false positives?"
> "Two ways: (1) High confidence threshold (0.5+), (2) Size filter - reject tiny detections (<20x20 pixels) as noise."

---

## **Key Technologies**

- **Framework:** Ultralytics YOLOv11
- **Optimization:** NVIDIA TensorRT  
- **Training:** PyTorch + Albumentations (augmentation)
- **Hardware:** Jetson Orin Nano (8GB)
- **Annotation:** Roboflow (for OBB labels)
