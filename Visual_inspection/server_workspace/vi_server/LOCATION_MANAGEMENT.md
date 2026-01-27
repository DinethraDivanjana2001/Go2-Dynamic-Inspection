# Location Management System

## Overview

The location management system enables two modes of robot inspection:

1. **Known Locations (Predefined)**: Save inspection points once, reuse them repeatedly
2. **Ad-hoc Locations**: Inspect any arbitrary coordinate on demand

## Architecture

### Database Schema

```
areas
├── id (PK)
├── name (unique)
├── description
├── floor_number
├── map_image_path
├── map_width, map_height
├── origin_x, origin_y
└── timestamps

inspection_points
├── id (PK)
├── area_id (FK → areas)
├── name
├── object_type
├── x, y, z, orientation
├── is_active
├── notes
├── last_inspected_at
└── timestamps

jobs (updated)
├── ... (existing fields)
└── inspection_point_id (FK → inspection_points, optional)
```

## API Endpoints

### Area Management

```
POST   /api/v1/areas                    Create new area
GET    /api/v1/areas                    List all areas
GET    /api/v1/areas/{id}               Get area details
PUT    /api/v1/areas/{id}               Update area
DELETE /api/v1/areas/{id}               Delete area
```

### Inspection Point Management

```
POST   /api/v1/areas/{area_id}/points                Create inspection point
GET    /api/v1/areas/{area_id}/points                List points in area
GET    /api/v1/areas/{area_id}/points/{point_id}     Get point details
PUT    /api/v1/areas/{area_id}/points/{point_id}     Update point
DELETE /api/v1/areas/{area_id}/points/{point_id}     Delete point
```

### Coordinate Setup (First-time calibration)

```
POST   /api/v1/areas/{area_id}/request-coordinates   Request robot position
POST   /api/v1/areas/{area_id}/save-coordinates      Save robot's coordinates
```

### Inspection Execution

```
POST   /api/v1/inspections/known    Inspect known location
POST   /api/v1/inspections/adhoc    Inspect arbitrary coordinates
```

## Usage Workflows

### Workflow 1: Setting Up Known Locations (First Time)

1. **Create Area** (from UI or API):
```bash
POST /api/v1/areas
{
  "name": "Floor 1 - Main Hall",
  "floor_number": 1,
  "map_width": 20.0,
  "map_height": 15.0
}
```

2. **Request Coordinates** (UI sends):
```bash
POST /api/v1/areas/1/request-coordinates
```

3. **Robot Navigates Manually** to desired inspection point

4. **Robot Sends Coordinates** (or UI saves them):
```bash
POST /api/v1/areas/1/save-coordinates
{
  "point_name": "Gauge 1",
  "object_type": "gauge",
  "x": 5.2,
  "y": 3.8,
  "z": 0.0,
  "orientation": 1.57
}
```

5. **Repeat** for all inspection points in the area

### Workflow 2: Inspecting Known Locations

1. **UI selects area and point** (e.g., "Floor 1" → "Gauge 1")

2. **UI sends inspection request**:
```bash
POST /api/v1/inspections/known
{
  "area_id": 1,
  "point_id": 5
}
```

3. **Server responds with coordinates**:
```json
{
  "job_id": "pending",
  "status": "NAVIGATION_REQUESTED",
  "message": "Navigate to Gauge 1 and capture ROI",
  "coordinates": {
    "x": 5.2,
    "y": 3.8,
    "z": 0.0,
    "orientation": 1.57,
    "point_id": 5,
    "point_name": "Gauge 1",
    "object_type": "gauge"
  }
}
```

4. **Robot navigates** to coordinates

5. **Robot captures ROI** and sends to existing job endpoint:
```bash
POST /api/v1/jobs
{
  "file": <image>,
  "object_type": "gauge",
  "metadata_json": {
    "robot_id": "robot_01",
    "location": "Floor 1 - Gauge 1",
    "inspection_point_id": 5
  }
}
```

### Workflow 3: Ad-hoc Inspection

1. **UI user clicks on 2D map** to select coordinates (x: 7.5, y: 4.2)

2. **UI sends ad-hoc inspection request**:
```bash
POST /api/v1/inspections/adhoc
{
  "area_id": 1,
  "x": 7.5,
  "y": 4.2,
  "z": 0.0,
  "orientation": 0.0,
  "object_type": "unknown"
}
```

3. **Server responds with coordinates** (same format as known location)

4. **Robot navigates and captures** (same as workflow 2, step 4-5)

5. **If position needs adjustment**, user can click nearby and retry

## Test UI

Open `location_ui.html` in a browser to test the system:

```bash
# Start server
python -m uvicorn app.main:app --host 0.0.0.0 --port 8001

# Open in browser
location_ui.html
```

### UI Features:

- **Area Selection**: Dropdown to select floor/area
- **2D Coordinate Map**: Visual representation with clickable points
- **Inspection Points List**: Shows all saved points in selected area
- **Request Position**: Simulates requesting robot's current coordinates
- **Ad-hoc Inspection**: Click map to select arbitrary coordinates
- **Visual Feedback**: Status messages and coordinate display

## Database Migration

Run migration to add new tables:

```bash
python migrate_add_locations.py
```

## Integration with Existing System

### Job Submission (Updated)

When robot sends inspection job, it can now include `inspection_point_id`:

```python
metadata = {
    "robot_id": "robot_01",
    "location": "Floor 1 - Gauge 1",
    "timestamp": "2026-01-21T12:00:00",
    "inspection_point_id": 5  # Optional - links job to known point
}
```

### Benefits:

1. **Traceability**: Track which jobs came from which inspection points
2. **Analytics**: Analyze inspection history per location
3. **Maintenance**: Know when each point was last inspected
4. **Reporting**: Generate location-based inspection reports

## ROS Service Compatibility

The current REST API design is compatible with future ROS service conversion:

### Current (REST):
```
UI → HTTP POST → Server → HTTP Response → UI
Robot → HTTP POST → Server → Process → Store
```

### Future (ROS Services):
```
UI → ROS Service Call → Server Node → Response → UI
Robot → ROS Service Call → Server Node → Process → Store
```

**Migration Path:**
1. Create ROS service definitions (.srv files) matching current schemas
2. Wrap existing route handlers in ROS service callbacks
3. Keep database and processing logic unchanged
4. Both REST and ROS can coexist during transition

## Example ROS Service Definitions (Future)

```
# InspectKnownLocation.srv
int32 area_id
int32 point_id
---
string job_id
string status
string message
float64 x
float64 y
float64 z
float64 orientation
```

```
# InspectAdhocLocation.srv
int32 area_id
float64 x
float64 y
float64 z
float64 orientation
string object_type
---
string job_id
string status
string message
```

## Testing

### 1. Create Test Area:
```bash
curl -X POST http://localhost:8001/api/v1/areas \
  -H "Content-Type: application/json" \
  -d '{"name": "Test Floor", "floor_number": 1}'
```

### 2. Add Inspection Point:
```bash
curl -X POST http://localhost:8001/api/v1/areas/1/points \
  -H "Content-Type: application/json" \
  -d '{
    "name": "Test Gauge",
    "object_type": "gauge",
    "x": 5.0,
    "y": 3.0,
    "z": 0.0,
    "orientation": 0.0
  }'
```

### 3. Request Known Location Inspection:
```bash
curl -X POST http://localhost:8001/api/v1/inspections/known \
  -H "Content-Type: application/json" \
  -d '{"area_id": 1, "point_id": 1}'
```

### 4. Request Ad-hoc Inspection:
```bash
curl -X POST http://localhost:8001/api/v1/inspections/adhoc \
  -H "Content-Type: application/json" \
  -d '{
    "area_id": 1,
    "x": 7.5,
    "y": 4.2,
    "object_type": "unknown"
  }'
```

## Summary

✅ Database schema with Areas and InspectionPoints
✅ Complete REST API for location management
✅ Coordinate setup workflow for first-time calibration
✅ Known location inspection (precise, repeatable)
✅ Ad-hoc location inspection (flexible, on-demand)
✅ Test UI with 2D coordinate map
✅ Integration with existing job system
✅ ROS service compatible design
✅ Migration script for database updates
