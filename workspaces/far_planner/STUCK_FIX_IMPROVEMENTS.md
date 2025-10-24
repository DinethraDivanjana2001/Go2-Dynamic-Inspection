# Waypoint to Cmd_Vel - Stuck Robot Fix

## Problem
The robot was getting stuck and unable to move even when a path was given. It would try to move but couldn't make progress, often getting stuck in a rotation loop or oscillating without forward movement.

## Root Causes Identified

1. **Too Strict Angle Tolerance** (0.1 rad / ~5.7°)
   - Robot waited for near-perfect alignment before moving
   - Small angle errors from odometry noise caused constant re-alignment
   - Led to oscillation and no forward progress

2. **No Minimum Speed Threshold**
   - Commanded velocities could be very small (< 0.05 m/s)
   - Insufficient to overcome friction and motor dead zones
   - Robot appeared to be trying but not moving

3. **No Stuck Detection**
   - No mechanism to detect when robot wasn't making progress
   - No recovery behavior when stuck

4. **Binary Control Strategy**
   - Either rotate in place OR move forward
   - No gradual transition between behaviors
   - Caused jerky, inefficient motion

## Solutions Implemented

### 1. Relaxed Angle Tolerance
- **Old**: `angle_tolerance = 0.1 rad` (~5.7°)
- **New**: `angle_tolerance = 0.5 rad` (~28.6°)
- **Benefit**: Robot starts moving with reasonable alignment instead of waiting for perfection

### 2. Three-Tier Control Strategy

#### a) Large Angle Error (> 90°)
```cpp
if (angle_error > 1.57 rad) {
    // Rotate in place only
    linear_velocity = 0
    angular_velocity = with minimum threshold
}
```

#### b) Moderate Angle Error (28° - 90°)
```cpp
else if (angle_error > 0.5 rad) {
    // Slow forward + correction
    linear_velocity = reduced_speed (30-100% based on angle)
    angular_velocity = proportional correction
}
```

#### c) Small Angle Error (< 28°)
```cpp
else {
    // Full speed + gentle correction
    linear_velocity = full_speed
    angular_velocity = gentle_correction (50% gain)
}
```

### 3. Minimum Speed Thresholds
- **Min Linear Speed**: 0.15 m/s (overcomes friction)
- **Min Angular Speed**: 0.2 rad/s (ensures actual rotation)
- If calculated speed is below minimum, uses minimum value
- Prevents "trying but not moving" behavior

### 4. Stuck Detection & Recovery
```cpp
- Track position history (last 1 second)
- If total movement < 0.1m in 1 second → STUCK
- If stuck for > 3 seconds → Recovery mode
- Recovery: Push forward at 1.5x min_speed regardless of angle
```

### 5. Faster Update Rate
- **Old**: 100ms (10 Hz)
- **New**: 50ms (20 Hz)
- **Benefit**: More responsive, smoother control

### 6. Adaptive Speed Scaling
- Speed reduces smoothly as angle error increases
- Prevents aggressive deceleration/acceleration
- Maintains momentum while correcting heading

## New Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 0.8 m/s | Maximum forward speed |
| `min_linear_speed` | 0.15 m/s | Minimum speed to overcome friction |
| `max_angular_speed` | 1.5 rad/s | Maximum rotation speed |
| `min_angular_speed` | 0.2 rad/s | Minimum rotation speed |
| `linear_kp` | 0.8 | Linear velocity gain |
| `angular_kp` | 3.0 | Angular velocity gain |
| `goal_tolerance` | 0.3 m | Distance to goal |
| `angle_tolerance` | 0.5 rad | Angle error to start moving |
| `rotate_in_place_threshold` | 1.57 rad | Large angle → rotate only |
| `stuck_timeout` | 3.0 s | Time before stuck recovery |
| `stuck_distance_threshold` | 0.1 m | Movement threshold for stuck detection |

## Tuning Guide

### Robot Still Gets Stuck?
1. **Decrease** `min_linear_speed` (try 0.1 m/s)
2. **Increase** `stuck_timeout` to 5.0s (give more time before recovery)
3. **Increase** `angle_tolerance` to 0.8 rad (be less strict about alignment)

### Robot Too Aggressive / Overshoots?
1. **Decrease** `max_linear_speed` to 0.5 m/s
2. **Decrease** `angular_kp` to 2.0
3. **Increase** `goal_tolerance` to 0.5 m

### Robot Too Slow?
1. **Increase** `max_linear_speed` to 1.0 m/s
2. **Increase** `linear_kp` to 1.2
3. **Decrease** `angle_tolerance` to 0.3 rad

### Robot Rotates Too Much Before Moving?
1. **Decrease** `rotate_in_place_threshold` to 1.0 rad
2. **Increase** `angle_tolerance` to 0.7 rad

## Testing Results Expected

### Before Fix:
- Robot hangs at waypoints
- Constant rotation without forward motion
- Very slow progress
- Frequent stopping and starting

### After Fix:
- Smooth approach to waypoints
- Continuous forward motion with heading correction
- Faster overall navigation
- Automatic recovery from stuck situations
- More efficient paths

## Monitoring

Watch the console output for these messages:

```bash
# Normal operation
[INFO] New waypoint: (x, y)
[INFO] Reached waypoint (dist: 0.2m)

# Stuck detection
[WARN] Robot appears stuck! Distance: 2.5m, Angle error: 45 deg

# Rotation mode
[INFO] Rotating in place: angle_error=120 deg
```

## Advanced Debugging

Enable debug output:
```bash
ros2 run far_planner waypoint_to_cmd_vel --ros-args --log-level debug
```

This shows every control loop:
```
Distance: 2.5m, Angle: 45deg, Linear: 0.3m/s, Angular: 0.8rad/s
```

## Quick Test

After launching, monitor:
```bash
# Watch waypoints
ros2 topic echo /way_point

# Watch commands
ros2 topic echo /cmd_vel

# Watch odometry
ros2 topic echo /state_estimation
```

The robot should now:
1. Accept waypoints with moderate angle errors
2. Move forward while correcting heading
3. Maintain minimum speeds to overcome friction
4. Detect and recover from stuck situations
5. Navigate smoothly to goals

## Rollback (if needed)

If the new behavior is problematic, you can adjust parameters in the launch file or revert to more conservative settings:

```python
parameters=[
    {'max_linear_speed': 0.3},      # Very conservative
    {'min_linear_speed': 0.05},     # Lower minimum
    {'angle_tolerance': 0.2},       # Stricter alignment
]
```
