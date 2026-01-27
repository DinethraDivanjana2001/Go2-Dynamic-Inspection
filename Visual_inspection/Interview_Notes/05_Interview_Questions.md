# Essential Interview Questions

## **Quick Project Pitch (30 seconds)**
> "I built an autonomous industrial inspection system using a Unitree Go2 quadruped robot. The robot uses TensorRT-optimized YOLOv11 for real-time object detection, offloads heavy analysis to a FastAPI server running a hybrid AI pipeline - combining Vision Transformers for keypoint detection with geometric algorithms for precision measurement. For unknown scenarios, I integrated Gemini 2.0 VLM with one-shot prompting. The system achieves 96% accuracy on gauge readings."

---

## **Technical Deep-Dive Questions**

### **ML/AI Questions**

**Q: Why hybrid DL + Geometry instead of end-to-end?**
> "Explainability. If an end-to-end model fails, you can't debug it. My modular approach lets me pinpoint failures: keypoints wrong? OCR issue? Angle calculation? This is critical for industrial applications."

**Q: What's the difference between your Vision Transformer and a CNN?**
> "Transformers use self-attention to see the whole image at once, while CNNs only see local patches. For gauges, this helps when the needle is partially hidden - the Transformer understands the full gauge context."

**Q: How does RANSAC work?**
> "It's robust regression. Say OCR gives 10 numbers and 3 are wrong. RANSAC randomly picks subsets, fits lines, and chooses the line with the most 'inliers'. It's democratic voting that ignores outliers automatically."

**Q: Why Wing Loss over MSE?**
> "Wing Loss heavily penalizes small errors (1-2 pixels) which MSE ignores. For sub-pixel accuracy in gauge reading, this forces the model to optimize even when error is already low."

**Q: Explain TensorRT optimization.**
> "It's NVIDIA's compiler that: (1) Fuses layers to reduce memory access, (2) Uses FP16 instead of FP32 for 2x speed, (3) Optimizes kernel execution for the specific GPU. Result: 3x faster inference with zero accuracy loss."

---

### **System Design Questions**

**Q: Why AsyncIO in your server?**
> "I/O-bound tasks. VLM calls take 2 seconds - a synchronous server would freeze. AsyncIO lets me handle 100 simultaneous robot uploads because while waiting for one API response, the CPU processes other requests."

**Q: Explain your producer-consumer pattern.**
> "Separates *receiving* work from *doing* work. The API (producer) accepts images instantly and returns a job ID. Background workers (consumers) process the queue at their own pace. This prevents traffic bursts from crashing the server."

**Q: Why HTTP over ROS2?**
> "Reliability over unreliable networks. Industrial WiFi is spotty - ROS2's DDS can lose packets and corrupt images. HTTP uses TCP for guaranteed delivery and is stateless, so failed requests just retry."

**Q: Why SQLite instead of PostgreSQL?**
> "Single-server deployment. SQLite handles 10,000+ writes/sec easily without network overhead, requires zero configuration, and 'backup' means copying a file. I'd upgrade to Postgres when scaling to multiple servers."

---

### **Problem-Solving Questions**

**Q: How did you handle the decimal point problem?**
> "OCR engines missed tiny dots. My solution: After OCR, I crop the space between digits and run a morphological filter looking for small circular blobs. If found, I inject the decimal. Accuracy jumped from 80% to 96%."

**Q: What if both OCR models fail?**
> "Fallback to VLM. If PaddleOCR and EasyOCR both have low confidence, the system automatically routes to Gemini for a 'human-like' reading."

**Q: How do you prevent VLM hallucinations?**
> "Three ways: (1) Prompt engineering with one-shot examples, (2) Self-evaluation - ask the model for confidence, (3) Cross-verification with the geometric pipeline - flag disagreements."

**Q: Biggest challenge you solved?**
> "OCR ensemble. Single models were 78% accurate. I built a cascading system: fast model first, slow model if needed, RANSAC to reject outliers. Final accuracy: 91% on OCR, 96% on final reading."

---

### **Trade-offs & Decisions**

**Q: Why OBB over standard bounding boxes?**
> "Diagonal objects. When a gauge is tilted 45°, a horizontal box captures 50% background noise. OBB rotates with the object for perfect crops, improving downstream accuracy by 15%."

**Q: Why not run everything on the robot?**
> "Battery and safety. Heavy AI drains battery and slows navigation. The Edge-Server split keeps the robot responsive for obstacle avoidance while offloading 'thinking' to a powerful server."

**Q: Gemini vs local LLaVA model?**
> "Accuracy vs latency. LLaVA hallucinates heavily on small text and industrial objects. Gemini is state-of-the-art. I accept the 1.5s latency for 20% better accuracy because safety inspections aren't time-critical."

---

### **Deployment & Production**

**Q: How do you handle network failures?**
> "The robot queues images locally. When WiFi reconnects, it flushes the queue. The server is stateless (HTTP), so retries are simple."

**Q: How would you scale to 100 robots?**
> "Horizontal scaling: (1) Load balancer (Nginx) in front, (2) Multiple FastAPI instances behind it, (3) Upgrade to Postgres for concurrent writes, (4) Use Redis for shared queue."

**Q: What's your inference latency breakdown?**
> "Edge (Jetson): 15ms YOLO + 2ms crop + 50ms transfer = 67ms. Server: 80ms geometric pipeline OR 2000ms VLM. Total: ~150ms for normal cases."

---

## **Behavioral/Project Management**

**Q: What would you do differently?**
> "Start with containerization (Docker) from day 1. I added it later, which meant migrating dependencies. Also, I'd use experiment tracking (MLFlow) earlier to avoid re-running ablation studies."

**Q: Most impactful technical decision?**
> "Using RANSAC for OCR noise. It turned an 80% accurate OCR into a 96% accurate system by automatically handling bad data. Simple algorithm, huge impact."

**Q: How did you validate your results?**
> "Ground truth dataset of 100 manually verified gauge readings. Measured MAE (Mean Absolute Error) and ensured 95% of predictions were within ±2% of true values."
