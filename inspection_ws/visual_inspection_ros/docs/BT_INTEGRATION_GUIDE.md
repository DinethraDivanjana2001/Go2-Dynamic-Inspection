# Behaviour Tree Integration Guide
**Visual Inspection System — Jetson Orin Nano**

---

## 1. Overview
The visual inspection pipeline exposes a single **ROS2 Service** for the Behavior Tree (BT) to interact with. 

**IMPORTANT FOR BT DEVELOPERS:** The visual inspection module is a ROS2 **Service** (not an Action). The BT simply triggers the service and waits until the inspection completes. 

**Service Name**: `/visual_inspection/inspect`
**Service Type**: `visual_inspection_interfaces/srv/Inspect`

---

## 2. Input: From Behavior Tree to Inspection System

When the Behavior Tree reaches the Inspection Node, it must send the following request fields to the service:

| Input Field | Type | What BT should provide |
|---|---|---|
| `target_object` | `string` | What to inspect: `"fire_extinguisher"`, `"door"`, `"person"`, `"gauge"`, or `"unknown"` |
| `location_label` | `string` | The current location on the map (e.g. `"engine_room_A"`). Used for naming saved folders. |
| `max_objects` | `int32` | How many to inspect. Send `0` to inspect ALL objects of that type found in the area. |
| `return_home` | `bool` | Send `true` so the camera resets its pan/tilt to center after finishing. |

---

## 3. Output: From Inspection System back to Behavior Tree

When the inspection is complete, the Jetson will return the following response to the BT. 

**IMPORTANT:** The images themselves and their metadata (JSON files containing confidence, convergence, errors, etc.) are saved on the Jetson's disk. The BT receives the **paths** to these images so it knows exactly where to find them and their metadata.

| Output Field | Type | What BT receives |
|---|---|---|
| `success` | `bool` | `true` if at least 1 object was successfully inspected and captured. |
| `status` | `string` | Reason for completion: `"ok"`, `"no_detection"`, `"ibvs_timeout"`, `"all_in_back"`, `"no_frames"`, or `"busy"`. |
| `objects_found` | `int32` | How many were spotted by the wide Insta360 camera. |
| `objects_inspected` | `int32` | How many were actually focused on and successfully captured by the Logitech camera. |
| `object_in_back` | `bool` | `true` means the object is behind the robot. The BT should read this and trigger a 180° rotation of the robot body, then try again. |
| `image_paths` | `string[]` | **CRITICAL:** A list of absolute paths to the saved images. The BT or Server can use these paths to read the images. Every image folder also contains a `metadata.json` file with all the metrics (confidence, time taken, etc). |
| `info` | `string` | Human-readable log summary string. |

---

## Target Object Classes

| `target_object` | Mode | Pipeline |
|---|---|---|
| `"fire_extinguisher"` | **Full IBVS** | Insta360 detect → coarse pan-tilt → Logitech IBVS → 3 images |
| `"extinguisher"` | **Full IBVS** | Same (alias) |
| `"door"` | **Full IBVS** | Same |
| `"person"` | **Full IBVS** | Same |
| `"gauge"` | **Full IBVS + Sweep** | Same + serpentine sweep if Insta360 misses |
| `"unknown"` | **Overview only** | Servo home → Insta360 raw + 1 Logitech image |
| `"main_cylinder"` | **Overview only** | Same |
| `""` or `"any"` | **All classes** | Detect all YOLO classes, inspect each |

### YOLO Model Classes (yolov26s.engine)

| ID | YOLO Name | Accepted `target_object` values |
|---|---|---|
| 0 | `door` | `"door"` |
| 1 | `extinguisher` | `"fire_extinguisher"`, `"extinguisher"` |
| 2 | `gauge` | `"gauge"`, `"pressure_gauge"` |
| 3 | `person` | `"person"`, `"people"` |

*`unknown` and `main_cylinder` are NOT YOLO classes → overview-only mode.*

### Confidence Thresholds (per class)

| Class | Threshold | Reason |
|---|---|---|
| extinguisher | 0.5 | High confidence needed |
| door | 0.5 | |
| person | 0.5 | |
| gauge | **0.3** | Gauge is visually weak — lower to avoid missing it |

---

## Status Values

| `status` | BT Action |
|---|---|
| `"ok"` | Success — continue |
| `"no_detection"` | Nothing found → navigate to next position |
| `"ibvs_timeout"` | IBVS didn't converge → move closer |
| `"all_in_back"` | Objects in back half → rotate robot 180° |
| `"no_frames"` | Camera failure |
| `"busy"` | Another inspection running — retry |

---

## Gauge Special Case: Sweep Scan

If gauge is **not detected** on Insta360 after 8s:
1. Servo does a **serpentine sweep**: tilt=[20, 50, 80]°, pan=[20→160]° (10° steps)
2. Logitech checks for gauge at every position
3. If found → **IBVS fires immediately** from that position (no coarse step)
4. If not found after full sweep → `status="no_detection"`

---

## Object Numbering (Multiple Instances)

When multiple same-class objects are detected (e.g. 2 gauges):
- Numbered by **confidence** (highest = #1)
- Inspected in order: #1 → #2 → ...
- Each saved to its own folder with `metadata.json`

### Capture Folder Structure
```
captures/
├── inspection/20260420_120000/        ← full IBVS session
│   ├── extinguisher/
│   │   └── instance_1/
│   │       ├── img_01_conf0.85.jpg   ← 3 Logitech ROI images
│   │       ├── img_02_conf0.85.jpg
│   │       ├── img_03_conf0.85.jpg
│   │       ├── overview_engine_room_A_01.jpg  ← Insta360 raw
│   │       └── metadata.json
│   └── gauge/
│       ├── instance_1/               ← highest confidence
│       └── instance_2/               ← lower confidence
└── overview/20260420_120100/         ← overview-only session
    ├── overview_unknown_01.jpg       ← Insta360 raw
    └── unknown/instance_1/
        ├── img_01_conf0.00.jpg       ← 1 Logitech from home
        └── metadata.json
```

---

## 4. Why Paths Instead of Images? (Architecture Note)

You might wonder why the service returns `image_paths` instead of a `sensor_msgs/Image` array. 
* **DDS Payload Limits**: Sending multiple uncompressed high-res images over a synchronous ROS2 Service response will often exceed DDS payload limits or cause severe network lag.
* **BT Stability**: To ensure the Behavior Tree never freezes waiting for massive payloads, the heavy images are written instantly to disk. The BT only receives the lightweight file paths, allowing it to read them safely on its own time without blocking the ROS2 middleware.

---

## 5. C++ BehaviorTree Node Example (BehaviorTree.CPP)

If you are using **BehaviorTree.CPP** (standard with Nav2), here is a complete C++ Leaf Node that wraps the ROS2 Service. You can drop this directly into your BT architecture.

```cpp
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "visual_inspection_interfaces/srv/inspect.hpp"

class InspectObjectNode : public BT::CoroActionNode
{
public:
    InspectObjectNode(const std::string& name, const BT::NodeConfiguration& config)
      : BT::CoroActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_inspect_client");
        client_ = node_->create_client<visual_inspection_interfaces::srv::Inspect>("/visual_inspection/inspect");
    }

    // Define the inputs the BT XML will provide to this node
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("target_object"),
            BT::InputPort<std::string>("location_label")
        };
    }

    BT::NodeStatus tick() override
    {
        std::string target_obj;
        std::string location;
        getInput("target_object", target_obj);
        getInput("location_label", location);

        auto request = std::make_shared<visual_inspection_interfaces::srv::Inspect::Request>();
        request->target_object = target_obj;
        request->location_label = location;
        request->max_objects = 0;
        request->return_home = true;

        auto future = client_->async_send_request(request);
        
        // Yield to prevent BT from freezing while waiting
        while (rclcpp::ok() && future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
            setStatusRunningAndYield(); 
        }

        auto response = future.get();

        if (response->success) {
            // Success: inspection complete. Images are at response->image_paths
            return BT::NodeStatus::SUCCESS;
        } else if (response->object_in_back) {
            // Signal BT to rotate robot 180!
            return BT::NodeStatus::FAILURE; 
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<visual_inspection_interfaces::srv::Inspect>::SharedPtr client_;
};
```

---

## 6. BT XML Example (Navigation + Inspection)

This is how the XML looks when hooking the inspection up to the navigation sequence:

```xml
<root main_tree_to_execute="MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="NavigateAndInspect">
            <!-- 1. Nav2 drives the robot to the gauge -->
            <NavigateToPose pose="engine_room_A_waypoint" />
            
            <!-- 2. Trigger the Visual Inspection -->
            <InspectObjectNode target_object="gauge" location_label="engine_room_A" />
        </Sequence>
    </BehaviorTree>
</root>
```

---

## Running the Service

```bash
# ── Every terminal needs this ──────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# Terminal 1 — Cameras
ros2 run visual_inspection_ros camera_node

# Terminal 2 — Servo
ros2 run visual_inspection_ros servo_node

# Terminal 3 — Inspection service (replaces ibvs_action_server)
ros2 run visual_inspection_ros inspection_service
```

---

## Test Commands (Simulate BT — 7 classes)

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/Visual_Inspection_ws/inspection_ws/install/setup.bash

# 1. Fire Extinguisher
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object fire_extinguisher

# 2. Door
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object door

# 3. Person
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object person

# 4. Gauge (with sweep fallback if not found on Insta360)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object gauge

# 5. Unknown (overview only — Insta360 + 1 Logitech at home)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object unknown

# 6. Main Cylinder (overview only)
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py --object main_cylinder

# 7. All classes
python3 ~/Documents/Visual_Inspection_ws/test_scripts/test_inspection_service.py
```

---

## Monitoring Topics (Optional — for debugging only)

| Topic | Type | Description |
|---|---|---|
| `/visual_inspection/debug` | `Image` | Side-by-side Insta360 + Logitech (RViz2) |
| `/visual_inspection/status` | `String` | `IDLE` / `DETECTING` / `COARSE` / `IBVS` / `CAPTURING` / `OVERVIEW` / `SWEEP` |
| `/visual_inspection/ibvs_error` | `Point` | x/y pixel error during IBVS |
| `/visual_inspection/detections` | `String` | JSON of detected objects |
| `/servo/pan_tilt` | `Int16MultiArray` | `[tilt, pan]` servo angles |

> **Note:** These topics are optional — only subscribe when debugging. During real deployment, skip RViz2 to save GPU/CPU resources.

---

## File Locations

| File | Path |
|---|---|
| Service definition | `inspection_ws/visual_inspection_interfaces/srv/Inspect.srv` |
| Service server | `inspection_ws/visual_inspection_ros/visual_inspection_ros/inspection_service.py` |
| Test client | `test_scripts/test_inspection_service.py` |
| This doc | `inspection_ws/visual_inspection_ros/docs/BT_INTEGRATION_GUIDE.md` |
