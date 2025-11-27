#!/bin/bash

# Pipeline Launch Script
# This script sources the required setup files and launches the complete pipeline

echo "=== Pipeline Launch Script ==="
echo "Sourcing required setup files..."

# Source the autonomous exploration workspace
echo "Sourcing autonomous_exploration workspace setup..."
source ~/Documents/Far_planner_test/workspaces/autonomous_exploration/install/setup.sh

# Source the far_planner workspace
echo "Sourcing far_planner workspace setup..."
source ~/Documents/Far_planner_test/workspaces/far_planner/install/setup.sh

# Source the dlio workspace 
echo "Sourcing dlio workspace setup..."
source ~/Documents/Far_planner_test/workspaces/dlio/install/setup.sh
source ~/Documents/Far_planner_test/workspaces/dlio/setup_cuda12.sh

echo "Sourcing open3d slam workspace setup..."
source ~/Documents/Far_planner_test/workspaces/open3d_slam_ws/install/setup.sh


# Source the pipeline_launcher workspace
echo "Sourcing pipeline_launcher workspace setup..."
source ~/Documents/Far_planner_test/workspaces/pipeline_launcher/install/setup.sh

echo "All setup files sourced successfully!"
echo ""
echo "=== Launching Pipeline ==="
echo "Starting the complete pipeline with the following sequence:"
echo "  T=0s: fast_lio mapping starts"
echo "  T=3s: vehicle_simulator starts"  
echo "  T=6s: far_planner starts"
echo ""

# Launch the pipeline
# Step 1: Start Gazebo simulation first and wait for it to be ready
echo "Starting Gazebo simulation..."
ros2 launch go2_config gazebo_velodyne.launch.py world:=/home/yasiru/world.world &
GAZEBO_PID=$!

# Wait for Gazebo and robot to be ready (controller_manager needs time)
echo "Waiting for Gazebo and robot to initialize (15 seconds)..."
sleep 15

# Check if Gazebo is still running
if ! kill -0 $GAZEBO_PID 2>/dev/null; then
    echo "ERROR: Gazebo failed to start. Check for errors above."
    exit 1
fi

# Step 2: Start the pipeline launcher
echo "Starting pipeline launcher..."
ros2 launch pipeline_launcher pipeline_gazebo.launch.py &
PIPELINE_PID=$!

# --- Publish static transforms ---
sleep 3  # Small delay to ensure ROS nodes are up

echo "Publishing static transforms..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 livox_frame sensor &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 map_o3d map &

echo "All nodes launched. Press Ctrl+C to stop."
wait  # Wait for both background processes to finish
