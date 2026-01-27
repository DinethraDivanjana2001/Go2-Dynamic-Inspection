# System Architecture

## **What I Built**
A distributed robot inspection system where a **Unitree Go2 robot** (with Jetson Orin Nano) scans industrial equipment and sends images to a **server** for AI analysis.

---

## **Why This Architecture?**

**The Problem:** Running heavy AI on the robot drains battery and slows down navigation.

**My Solution:** Split the work
- **Robot (Edge):** Fast detection only → finds objects
- **Server (Cloud):** Heavy AI → analyzes objects in detail

Think of it like: Robot = Camera, Server = Photo Editor

---

## **How They Communicate**

### REST API (Simple HTTP)
The robot sends images like uploading a photo to a website.

**Example:**
```
Robot → POST http://server:8000/api/jobs
        Sends: gauge_image.jpg + metadata
Server → Returns: "Job #123 received"
```

### Why HTTP (not ROS)?
- **Reliable:** Works over unstable Wi-Fi
- **Simple:** I can test with a web browser
- **Flexible:** Any device can talk to the server

---

## **The Server Magic: AsyncIO**

### The Problem
If processing takes 3 seconds, a normal server **freezes** and can't accept new images.

### My Solution: Async Processing
**Restaurant Analogy:**
- ❌ **Bad waiter:** Takes order, goes to kitchen, waits there, ignores other customers
- ✅ **Good waiter:** Takes order, gives it to chef, serves other customers while food cooks

**Technical Terms:**
- **FastAPI:** The web framework I used
- **AsyncIO:** Non-blocking code that handles multiple requests at once
- **Queue System:** Jobs wait in line, workers process them one by one

---

## **Data Flow**

```
ROBOT                          SERVER
  |                              |
  | 1. Camera sees gauge         |
  | 2. YOLO detects it (15ms)    |
  | 3. Crops image               |
  |                              |
  | 4. Sends via WiFi ---------> | 5. Receives image
  |                              | 6. Puts in queue
  |                              | 7. Worker picks it up
  |                              | 8. Runs AI pipeline
  |                              | 9. Saves result to database
```

---

## **Tech Stack Summary**

| Component | Technology | Why I Chose It |
|-----------|-----------|----------------|
| **Robot Processing** | TensorRT + YOLOv11 | 3x faster than normal PyTorch |
| **Communication** | HTTP/REST | Reliable over Wi-Fi |
| **Server Framework** | FastAPI | Handles 100+ requests at once |
| **Database** | SQLite | Simple, fast, no setup needed |
| **Task Queue** | Python AsyncIO | Non-blocking processing |

---

## **Interview Talking Points**

### 1. "Why split robot and server?"
> "Battery life and safety. Running heavy AI on the robot would slow down navigation and risk collisions. This way, the robot stays fast and responsive."

### 2. "What is AsyncIO?"
> "It lets my server handle multiple robot requests simultaneously. While waiting for AI results, it can accept new uploads instead of freezing."

### 3. "Why REST API over ROS?"
> "Reliability. ROS can lose data packets over WiFi. HTTP guarantees delivery and is easier to debug."

### 4. "How does the queue work?"
> "Like a ticket system at a restaurant. The API takes orders instantly and gives a receipt. Background workers cook the orders at their own pace."

### 5. "What if the server crashes?"
> "Jobs in memory are lost, but on restart, I check the database for stuck jobs and requeue them. For production, I'd use Redis for persistent queues."
