#!/bin/bash

# Trap SIGINT to kill background processes when the script is terminated
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# 1. Launch Voxel Publisher Node
echo "Starting Voxel Publisher Node..."
ros2 run voxel_grid_filter voxel_node --ros-args -p voxel_size:=0.5 &
PID1=$!

# 2. Launch Backend (Python Server)
echo "Starting Backend Server..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$ROOT_DIR/workspaces/web_viz/backend" || { echo "Backend directory not found"; exit 1; }
source /opt/ros/humble/setup.bash
python3 server.py &
PID2=$!

# 3. Launch Mission Planner UI (Frontend)
echo "Starting Mission Planner UI..."
cd "$ROOT_DIR/workspaces/web_viz/frontend" || { echo "Frontend directory not found"; exit 1; }
npm run dev -- --host &
PID3=$!

# Wait for all processes
wait $PID1 $PID2 $PID3
