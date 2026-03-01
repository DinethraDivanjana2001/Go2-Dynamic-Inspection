#!/bin/bash

# Trap SIGINT to kill background processes when the script is terminated
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT


# Determine the root directory of the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

source "$WORKSPACE_DIR/workspaces/voxel_ws/install/setup.sh"

# 1. Launch Voxel Publisher Node
echo "Starting Voxel Publisher Node..."
ros2 run voxel_grid_filter voxel_node --ros-args -p voxel_size:=0.5 &
PID1=$!

# Start Backend
echo "Starting Backend Server..."
cd "$WORKSPACE_DIR/workspaces/mission_planner_ui/backend" || { echo "Backend directory not found"; exit 1; }
source /opt/ros/humble/setup.bash
python3 server.py &
BACKEND_PID=$!

# Wait for backend to initialize
sleep 2

# Start Frontend
echo "Starting Frontend..."
cd "$WORKSPACE_DIR/workspaces/mission_planner_ui/frontend" || { echo "Frontend directory not found"; kill $BACKEND_PID; exit 1; }
npm run dev -- --host &
FRONTEND_PID=$!

# Wait for all processes
wait $PID1 $BACKEND_PID $FRONTEND_PID
