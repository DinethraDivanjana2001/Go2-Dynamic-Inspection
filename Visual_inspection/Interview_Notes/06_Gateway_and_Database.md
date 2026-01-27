# Database & API Communication

## **What I Built**
The backend communication layer connecting the robot to the server using **REST API** and **SQLite database**.

---

## **1. Communication: REST API**

### **What is an API?**
**Simple explanation:** A website for robots instead of humans.

**What is an Endpoint?**
A specific "function" you can call over the network.

**Example:**
```
POST /api/v1/jobs       → Upload new inspection image
GET  /api/v1/jobs/{id}  → Check status of a job
GET  /api/v1/health     → Is server alive?
```

### **Why REST over ROS2?**

| Feature | REST (HTTP) | ROS2 (DDS) |
|---------|------------|-----------|
| **Reliability** | TCP - guaranteed delivery | UDP - can lose packets |
| **Debugging** | Test with browser/Postman | Need ROS tools |
| **Network** | Works over WiFi | Flaky on unstable networks |
| **Flexibility** | Any device can connect | Needs ROS installation |

**My choice:** REST for reliability and simplicity.

---

## **2. Database: SQLite**

### **Schema Design**

**3 Main Tables:**

1. **Jobs** (Master Record)
```sql
id          UUID (Primary Key)
status      ENUM (QUEUED, PROCESSING, COMPLETED, FAILED)
created_at  TIMESTAMP
object_type STRING (gauge, fire_extinguisher, unknown)
```

2. **Results** (Inspection Outcomes)
```sql
job_id      UUID (Foreign Key)
value       FLOAT (e.g., 5.2)
unit        STRING (bar, psi)
confidence  FLOAT (0.98)
raw_json    TEXT (full pipeline output)
```

3. **Audit_Logs** (Debugging)
```sql
job_id      UUID
error_msg   TEXT (stack trace if failed)
latency_ms  INT (performance tracking)
```

### **Why SQLite?**
- **Zero config:** Just a file, no server to run
- **Fast:** 10,000+ writes/sec for our scale
- **Simple:** Backup = copy the file
- **WAL Mode:** Allows concurrent reads/writes

**When to upgrade:** Multiple servers writing simultaneously → switch to PostgreSQL.

---

## **3. The Async Workflow**

### **How a Request is Handled**

```
1. Robot sends image → POST /api/v1/jobs
2. Server validates image
3. Generate UUID job_id
4. Insert into Jobs table (status=QUEUED)
5. Put job in asyncio.Queue (RAM)
6. Return 202 Accepted {"job_id": "abc-123"}
   ← Robot gets immediate response

BACKGROUND (doesn't block the API):
7. Worker pulls job from queue
8. Update status → PROCESSING  
9. Run AI pipeline (80ms or 2000ms)
10. Update status → COMPLETED
11. Insert into Results table
```

### **Why This Design?**
- **Non-blocking:** API returns instantly (step 6)
- **Scalable:** Queue buffers traffic spikes
- **Tolerant:** If pipeline crashes, job stays in DB with PROCESSING status

---

## **4. VLM Integration**

### **API Call Flow**
```python
# Async to prevent blocking
async def analyze_with_vlm(image):
    response = await gemini_client.generate_content(
        prompt=build_prompt(),
        image=image
    )
    return parse_json(response)
```

### **Why `await`?**
While waiting for Gemini's API (2 seconds), the Python interpreter handles OTHER requests. This is non-blocking I/O.

**Without async:** Server freezes for 2 seconds.  
**With async:** Server handles 100+ concurrent uploads.

---

## **5. File Storage**

### **Why NOT Store Images in Database?**
**Anti-pattern:** BLOBs bloat the database, slow backups, waste RAM.

**My approach:**
- Save images to disk: `/data/jobs/abc-123/roi.jpg`
- Store only the **file path** in the database
- Database stays fast and lightweight

---

## **Interview Talking Points**

### 1. "What is a REST API?"
> "A standardized way for systems to communicate over HTTP. The robot sends a request to an endpoint like '/jobs', and the server responds with status and data."

### 2. "Why use UUIDs instead of integer IDs?"
> "Global uniqueness. If I scale to multiple servers, integer IDs can collide (both start at 1). UUIDs are generated on the robot BEFORE sending, enabling distributed tracing."

### 3. "How do you handle schema changes?"
> "Alembic for migrations. If I add a 'temperature' column, I create a migration script. This version-controls the database schema, keeping it in sync with code."

### 4. "What if the server crashes during processing?"
> "Jobs in the RAM queue are lost. On restart, a cleanup routine scans for jobs stuck in PROCESSING for >5 minutes and marks them FAILED or requeues them."

### 5. "How does async VLM work technically?"
> "Python's `await` keyword pauses THAT specific task, yielding control to the event loop. While waiting for Google's servers, the CPU executes OTHER tasks (like accepting new uploads). This is cooperative multitasking."

### 6. "Why SQLite over Postgres?"
> "Single-node deployment simplicity. No network overhead, no complex auth, instant backup by copying a file. I'd upgrade to Postgres when scaling to multiple servers writing concurrently."

### 7. "How do you monitor system health?"
> "Two ways: (1) Log every pipeline latency to Audit table - alert if average >200ms, (2) `/health` endpoint checks DB connectivity and queue size to detect bottlenecks."

---

## **Tech Stack Summary**

| Component | Technology | Purpose |
|-----------|-----------|---------|
| **API Framework** | FastAPI | Async HTTP server |
| **Database** | SQLite + SQLAlchemy | Data persistence |
| **Queue** | Python AsyncIO | Task buffer |
| **File Storage** | Local filesystem | Image artifacts |
| **VLM Client** | Google Gemini API | Vision-Language reasoning |
