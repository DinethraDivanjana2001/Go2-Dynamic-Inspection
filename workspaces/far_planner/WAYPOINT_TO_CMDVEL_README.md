# Waypoint to Cmd_Vel Converter

## Overview
This document describes the `waypoint_to_cmd_vel` node that converts waypoint goals from `far_planner` into velocity commands (`cmd_vel`) for robot control.

## What Was Added

### 1. New Node: `waypoint_to_cmd_vel.cpp`
- **Location**: `src/far_planner/src/waypoint_to_cmd_vel.cpp`
- **Purpose**: Converts waypoint goals to velocity commands for differential drive robots
- **Functionality**:
  - Subscribes to `/way_point` (geometry_msgs/PointStamped) from far_planner
  - Subscribes to `/state_estimation` (nav_msgs/Odometry) for robot pose
  - Publishes to `/cmd_vel` (geometry_msgs/Twist) for robot control
  - Implements simple proportional control for both linear and angular velocities

### 2. Control Strategy
The node uses a simple but effective control strategy:
- **Rotation First**: If the angle error is large, the robot rotates in place
- **Move and Adjust**: Once roughly aligned, the robot moves forward while adjusting heading
- **Goal Reached**: Stops when within goal tolerance distance

### 3. Configurable Parameters
You can adjust these parameters in the launch file or via command line:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 0.5 m/s | Maximum forward/backward speed |
| `max_angular_speed` | 1.0 rad/s | Maximum rotation speed |
| `linear_kp` | 1.0 | Proportional gain for linear velocity |
| `angular_kp` | 2.0 | Proportional gain for angular velocity |
| `goal_tolerance` | 0.3 m | Distance to consider goal reached |
| `angle_tolerance` | 0.1 rad | Angle error threshold to start moving forward |

### 4. Safety Features
- **Timeout Protection**: Stops robot if waypoint is older than 2 seconds
- **Velocity Clamping**: All velocities are clamped to configured maximum values
- **Zero Velocity on Stop**: Publishes explicit zero velocities when stopping

## How to Use

### Building
The node is automatically built when you build the far_planner package:

```bash
cd /home/yasiru/Documents/Far_planner_test/workspaces/far_planner
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Launching
The node is automatically started when you launch far_planner:

```bash
ros2 launch far_planner far_planner.launch
```

### Manual Launch (Optional)
You can also run the node separately:

```bash
ros2 run far_planner waypoint_to_cmd_vel
```

With custom parameters:

```bash
ros2 run far_planner waypoint_to_cmd_vel --ros-args \
  -p max_linear_speed:=0.8 \
  -p max_angular_speed:=1.5 \
  -p goal_tolerance:=0.2
```

## Topics

### Subscribed Topics
- `/way_point` (geometry_msgs/PointStamped): Target waypoint from far_planner
- `/state_estimation` (nav_msgs/Odometry): Current robot pose

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot

## Tuning Tips

### If the robot is too slow:
- Increase `max_linear_speed` and `max_angular_speed`
- Increase `linear_kp` and `angular_kp`

### If the robot oscillates or overshoots:
- Decrease `linear_kp` and `angular_kp`
- Increase `goal_tolerance`

### If the robot doesn't align well before moving:
- Increase `angle_tolerance` (more alignment before forward motion)
- Increase `angular_kp` (faster rotation)

### If the robot stops too far from waypoints:
- Decrease `goal_tolerance`

## Integration with Pipeline

The waypoint_to_cmd_vel node is now automatically launched as part of the far_planner launch file. The complete pipeline flow is:

1. **Open3D SLAM / DLIO** → Provides odometry and mapping
2. **Vehicle Simulator / Terrain Analysis** → Processes terrain data
3. **FAR Planner** → Plans path and publishes waypoints to `/way_point`
4. **Waypoint to Cmd_Vel** → Converts waypoints to velocity commands on `/cmd_vel`
5. **Robot** → Executes velocity commands

## Troubleshooting

### Robot doesn't move
- Check that `/way_point` is being published: `ros2 topic echo /way_point`
- Check that `/state_estimation` is being published: `ros2 topic echo /state_estimation`
- Verify the node is running: `ros2 node list | grep waypoint_to_cmd_vel`

### Robot moves erratically
- Check the control gains are not too high
- Verify the odometry quality
- Check for topic remapping issues

### Robot stops unexpectedly
- Check console for "Waypoint is too old" warnings
- Verify far_planner is continuously publishing waypoints

## Advanced Configuration

To modify the behavior further, you can edit the source file at:
```
/home/yasiru/Documents/Far_planner_test/workspaces/far_planner/src/far_planner/src/waypoint_to_cmd_vel.cpp
```

Key areas to modify:
- Control algorithm in `controlLoop()` function
- Safety timeout in `controlLoop()` (currently 2.0 seconds)
- Velocity calculation formulas

After modifications, rebuild the package:
```bash
cd /home/yasiru/Documents/Far_planner_test/workspaces/far_planner
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select far_planner
```
