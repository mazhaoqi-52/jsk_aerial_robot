#!/bin/bash

echo "=== Checking IMU Status ==="
echo "1. Check if IMU topic is publishing:"
timeout 5 rostopic hz /ninja1/imu 2>/dev/null || echo "No IMU topic found"

echo ""
echo "2. Check for timestamp warnings in recent logs:"
grep -i "bad timestamp" ~/.ros/log/latest/*.log 2>/dev/null | tail -5 || echo "No recent timestamp warnings found"

echo ""
echo "3. Check IMU message timestamps:"
echo "Capturing 10 IMU messages to check timestamp consistency..."
timeout 5 rostopic echo -n 10 /ninja1/imu/stamp 2>/dev/null || echo "Could not capture IMU timestamps"

echo ""
echo "4. Check system time vs ROS time:"
echo "System time: $(date +%s.%N)"
echo "ROS time: $(rostopic echo -n 1 /clock/clock/secs 2>/dev/null || echo 'No /clock topic')"

echo ""
echo "5. Check if using simulation time:"
rosparam get /use_sim_time 2>/dev/null || echo "use_sim_time parameter not set"
