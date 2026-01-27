# VLM Reasoning Pipeline

## **What I Built**
A Vision-Language Model (VLM) system using **Gemini 2.0 Flash** for intelligent visual inspection of safety equipment and unknown objects.

**NOT used for gauge reading** - gauges use the hybrid Vision Transformer + Geometry pipeline.

---

## **What VLM Actually Does**

### **VLM Inspection Tasks:**
1. **Fire Extinguishers** - Check if present and accessible (not blocked)
2. **Emergency Exits** - Verify exit pathway is clear
3. **Doors** - Confirm doors are closed and secured
4. **Hydraulic Cylinders** - Detect oil leaks on equipment/floor
5. **Unknown Objects** - Identify object type and apply appropriate inspection rules

### **When VLM is Used:**
```
If YOLO detects "fire_extinguisher" → VLM inspection
If YOLO detects "door" → VLM inspection  
If YOLO detects "emergency_exit" → VLM inspection
If YOLO detects "unknown" → VLM identifies object + inspects
If YOLO detects "gauge" → HYBRID PIPELINE (NOT VLM)
```

---

## **The Smart Routing System**

### **Object Type → Pipeline Mapping**

| Object Detected | Pipeline Used | What It Does |
|----------------|---------------|---------------|
| **Gauge** | Vision Transformer + Geometry | Reads analog values (5.2 Bar) |
| **Fire Extinguisher** | VLM (Gemini) | Checks if blocked/accessible |
| **Door** | VLM (Gemini) | Checks if closed/secured |
| **Emergency Exit** | VLM (Gemini) | Checks if path clear |
| **Main Cylinder** | VLM (Gemini) | Detects oil leaks |
| **Unknown** | VLM (Gemini) | Identifies object → applies rules |

---

## **How VLM Works**

### 1. Prompt Templates (Task-Specific)
Each object type has a custom YAML prompt configuration.

**Example: Fire Extinguisher Prompt**
```yaml
TASK: Check fire extinguisher compliance

INSPECTION CRITERIA:
1. Is fire extinguisher visible?
2. Is it accessible (not blocked)?

DECISION RULES:
- PASS: Present AND accessible
- FAIL: Present BUT blocked
- FAIL: Expected but missing

EVIDENCE:
- present: true/false
- blocked: true/false
```

### 2. The "Unknown" Handler
When YOLO detects an unknown object, VLM runs a **two-step process**:

**Step 1:** Identify the object
- "This is a fire extinguisher"

**Step 2:** Apply appropriate inspection rules
- "Fire extinguisher detected → checking accessibility"
- "Path is clear → PASS"

This is **zero-shot intelligence** - the VLM automatically figures out what to inspect.

### 3. Response Format (Structured JSON)
```json
{
  "task": "fire_extinguisher",
  "decision": "FAIL",
  "confidence": 0.92,
  "summary": "Fire extinguisher blocked by boxes",
  "findings": [
    "Fire extinguisher visible on wall",
    "Stack of cardboard boxes blocking access",
    "Clearance distance < 1 meter"
  ],
  "evidence": {
    "present": true,
    "blocked": true
  },
  "extracted_objects": ["fire_extinguisher", "boxes"]
}
```

---

## **Why Gemini 2.0 Flash?**

| Model | Speed | Use Case | Our Choice |
|-------|-------|----------|------------|
| GPT-4 Vision | 4-6s | Complex reasoning | ❌ Too slow |
| **Gemini 2.0 Flash** | **1.5s** | Safety inspection | **✅ Perfect** |
| Gemini 1.5 Pro | 3s | Long context | ❌ Overkill |
| LLaVA (local) | Fast | General vision | ❌ Hallucinates |

**Why Flash?** Fast enough for real-time + accurate enough for safety decisions.

---

## **VLM vs Hybrid Pipeline**

### **Why NOT use VLM for gauges?**

| Method | Speed | Accuracy | Explainability |
|--------|-------|----------|----------------|
| **Hybrid (Vision Transformer + Geometry)** | 80ms | 96% | ✅ Debuggable |
| **VLM (Gemini)** | 2000ms | 90% | ❌ Black box |

**Decision:** Use precise math for measurements (gauges), use AI intelligence for safety checks (fire extinguishers, doors).

---

## **Prompt Engineering Strategy**

### Base Rules (Applied to ALL tasks)
```markdown
- Return ONLY valid JSON
- No markdown code blocks
- Confidence must be 0.0-1.0
- Decision must be PASS/FAIL/UNKNOWN
```

### Task-Specific Rules
Each YAML file adds specialized criteria on top of base rules.

**Example Tasks:**
- `fire_extinguisher.yaml` - Accessibility checks
- `door.yaml` - Open/closed status
- `emergency_exit.yaml` - Path clearance
- `main_cylinder.yaml` - Leak detection
- `unknown.yaml` - Smart object identification

---

## **Handling VLM Failures**

### The Problems
1. **Hallucination** - Makes up information
2. **Invalid JSON** - Returns markdown or broken syntax
3. **Low Confidence** - Unsure about decision

### My Solutions
1. **Structured Prompts** - Force JSON schema compliance
2. **Validation** - Parse with Pydantic schemas, reject invalid responses
3. **Confidence Thresholding** - Flag decisions with confidence < 0.8
4. **Retry Logic** - If JSON invalid, retry with "fix your JSON" prompt

---

## **Technical Implementation**

### Server Code Flow
```python
# In vlm_router.py
def run_vlm_task(object_type, roi_path, metadata):
    # 1. Load prompt template
    prompt_config = load_prompt_config(object_type)
    
    # 2. Render prompts
    system_prompt = prompt_config.system
    user_prompt = prompt_config.render_user_prompt()
    
    # 3. Call Gemini
    response = vlm_client.analyze_image(
        image_path=roi_path,
        system_prompt=system_prompt,
        user_prompt=user_prompt
    )
    
    # 4. Validate JSON
    result = validate_and_parse_vlm_response(response)
    
    return result
```

### Async Processing
VLM calls take ~2 seconds - we use `await` to avoid blocking:
```python
async def process_inspection():
    result = await vlm_client.analyze_image(...)
    # Server handles other requests during this wait
```

---

## **Interview Talking Points**

### 1. "What does the VLM pipeline do?"
> "It handles safety inspections: checking if fire extinguishers are blocked, doors are closed, emergency exits are clear, and detecting oil leaks. It's for 'yes/no' safety decisions, NOT for precise measurements like gauge reading."

### 2. "Why not use VLM for gauge reading?"
> "Precision vs intelligence trade-off. Gauge reading needs sub-pixel accuracy (96% within ±2%). The hybrid geometric method is faster (80ms vs 2s) and more accurate. VLM is better for semantic understanding like 'is this blocked?'"

### 3. "How does the 'unknown' handler work?"
> "It's two-step reasoning. First, the VLM identifies what the object is. Second, it applies the appropriate inspection rules automatically. This is zero-shot learning - no retraining needed for new object types."

### 4. "Why Gemini over GPT-4?"
> "Speed. Gemini 2.0 Flash responds in 1.5s vs GPT-4's 4-6s. For safety inspections, we need fast feedback to the robot operator."

### 5. "How do you prevent hallucinations?"
> "Three layers: (1) Strict JSON schema in prompts, (2) Pydantic validation rejects malformed responses, (3) Confidence thresholding - flag answers below 0.8 for human review."

### 6. "What's in the prompt templates?"
> "Each object type has a YAML file with: system prompt (role definition), user template (inspection criteria), decision rules (PASS/FAIL logic), and output schema. Base rules apply to all tasks."

### 7. "How is this integrated with the robot?"
> "YOLO on the Jetson detects objects and sends them to the server. The server routes each object to the right pipeline - gauges go to hybrid method, fire extinguishers go to VLM. It's transparent to the robot."

---

## **Tech Stack**

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **VLM Provider** | Google Gemini 2.0 Flash API | Visual reasoning |
| **Prompt Management** | YAML configs | Task-specific prompts |
| **Validation** | Pydantic schemas | JSON parsing |
| **Orchestration** | FastAPI + AsyncIO | Non-blocking processing |
| **Routing** | vlm_router.py | Task dispatcher |
