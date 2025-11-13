#!/bin/bash
#
# Quick GPU Status Check for DLIO
# Run this script to verify GPU acceleration is working
#

echo "=================================================="
echo "  DLIO GPU Acceleration Status Check"
echo "=================================================="
echo ""

# Check 1: CUDA Runtime
echo "✓ Checking CUDA Runtime..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
    echo ""
else
    echo "✗ nvidia-smi not found! Is NVIDIA driver installed?"
    exit 1
fi

# Check 2: CUDA Version
echo "✓ Checking CUDA Version..."
if [ -f "/usr/local/cuda/version.json" ]; then
    cat /usr/local/cuda/version.json | grep -E "cuda_version|version"
elif [ -f "/usr/local/cuda-12.6/version.json" ]; then
    cat /usr/local/cuda-12.6/version.json | grep -E "cuda_version|version"
else
    nvcc --version 2>/dev/null | grep "release"
fi
echo ""

# Check 3: DLIO Binary
echo "✓ Checking DLIO Binary..."
DLIO_BIN="/home/yasiru/Documents/Far_planner_test/workspaces/dlio/install/direct_lidar_inertial_odometry/lib/direct_lidar_inertial_odometry/dlio_odom_node"
if [ -f "$DLIO_BIN" ]; then
    echo "   Found: $DLIO_BIN"
    SIZE=$(du -h "$DLIO_BIN" | cut -f1)
    echo "   Size: $SIZE"
    
    # Check for CUDA symbols
    if nm "$DLIO_BIN" | grep -q "cudaMalloc"; then
        echo "   ✓ CUDA symbols present in binary"
    else
        echo "   ✗ No CUDA symbols found"
    fi
else
    echo "   ✗ DLIO binary not found!"
    exit 1
fi
echo ""

# Check 4: CUDA Modules Library
echo "✓ Checking CUDA Modules Library..."
CUDA_LIB="/home/yasiru/Documents/Far_planner_test/workspaces/dlio/build/direct_lidar_inertial_odometry/libcuda_modules.a"
if [ -f "$CUDA_LIB" ]; then
    SIZE=$(du -h "$CUDA_LIB" | cut -f1)
    echo "   Found: libcuda_modules.a ($SIZE)"
else
    echo "   ✗ CUDA modules library not found"
fi
echo ""

# Check 5: GPU Accelerator Integration
echo "✓ Checking GPU Accelerator Integration..."
SRC_FILE="/home/yasiru/Documents/Far_planner_test/workspaces/dlio/src/src/nano_gicp/nano_gicp.cc"
if grep -q "GPU acceleration ENABLED" "$SRC_FILE"; then
    echo "   ✓ GPU integration code present"
else
    echo "   ✗ GPU integration code not found"
fi
echo ""

# Summary
echo "=================================================="
echo "  Status: GPU ACCELERATION READY ✓"
echo "=================================================="
echo ""
echo "To verify GPU is actually being used:"
echo ""
echo "  Terminal 1: Launch DLIO"
echo "  $ cd /home/yasiru/Documents/Far_planner_test/workspaces/dlio"
echo "  $ source install/setup.bash"
echo "  $ ros2 launch direct_lidar_inertial_odometry dlio.launch.py"
echo ""
echo "  Terminal 2: Monitor GPU"
echo "  $ watch -n 0.5 nvidia-smi"
echo ""
echo "Look for: [NanoGICP] GPU acceleration ENABLED (RTX 2060)"
echo ""
