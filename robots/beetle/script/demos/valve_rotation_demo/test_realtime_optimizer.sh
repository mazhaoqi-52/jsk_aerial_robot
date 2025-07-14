#!/bin/bash

# Test script for real-time insertion optimizer
# This script tests the real-time data capability of the insertion optimizer

echo "=== Testing Real-time Insertion Optimizer ==="
echo "Date: $(date)"
echo "Test objectives:"
echo "1. Verify optimizer can subscribe to real-time UAV and valve data"
echo "2. Verify automatic strategy selection based on current positions"
echo "3. Verify fallback to manual data when real-time data unavailable"
echo "4. Test both simulation and real modes"

echo ""
echo "=== Key Improvements ==="
echo "1. Added real-time ROS topic subscriptions"
echo "2. Automatic UAV position reading from /beetle{id}/mocap/pose"
echo "3. Automatic valve position reading from /valve/odom (sim) or /valve/mocap/pose (real)"
echo "4. Fallback to manual data when real-time data unavailable"
echo "5. Thread-safe data synchronization"
echo "6. Configurable module ID and simulation mode"

echo ""
echo "=== Test 1: Manual data test (no ROS required) ==="
cd /home/ma/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/beetle/script/demos/valve_rotation_demo/
python3 insertion_optimizer.py --manual

echo ""
echo "=== Test 2: ROS integration test ==="
echo "This test requires ROS to be running with proper topics"
echo "Testing with default parameters..."

# Test with simulation mode
export ROS_MASTER_URI=http://localhost:11311
python3 insertion_optimizer.py

echo ""
echo "=== Test 3: Main program integration ==="
echo "Testing integration with main valve rotation program..."

# Test syntax and import
python3 -c "
import sys
sys.path.append('.')
try:
    from insertion_optimizer import InsertionOptimizer
    from valve_rotation_fang_single import DescendAndContactState
    print('✓ All imports successful')
    print('✓ Real-time optimizer ready for integration')
except Exception as e:
    print(f'✗ Import error: {e}')
"

echo ""
echo "=== Expected Behavior ==="
echo "1. Manual test should work without ROS"
echo "2. ROS test should subscribe to topics and wait for data"
echo "3. Optimizer should automatically select best strategy based on real positions"
echo "4. Fallback should work when real-time data unavailable"
echo "5. Main program should integrate seamlessly"

echo ""
echo "=== Log Analysis Points ==="
echo "Look for these log messages:"
echo "- 'Insertion optimizer initialized with real-time data'"
echo "- 'Subscribing to UAV pose: /beetle{id}/mocap/pose'"
echo "- 'Subscribing to valve pose: /valve/odom or /valve/mocap/pose'"
echo "- 'Using real-time data for insertion strategy evaluation'"
echo "- 'REAL-TIME DATA FOR OPTIMIZATION'"
echo "- 'Real-time optimization: UAV->Valve distance optimized'"

echo ""
echo "=== Test completed at $(date) ==="
