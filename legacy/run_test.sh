#!/bin/bash
# Script to run Gate 2 data collection test inside container

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /home/orin/dev/ws/colcon_ws/install/setup.bash

echo "Starting ROS2 launch in background..."
ros2 launch pius_control pius_launch.py > /tmp/pius_launch.log 2>&1 &
LAUNCH_PID=$!

echo "Nodes started with PID: $LAUNCH_PID"
echo "Collecting data for 15 seconds..."
sleep 15

echo "Stopping nodes gracefully..."
kill -INT $LAUNCH_PID

# Wait for process to terminate and flush HDF5
sleep 3

echo "=== Launch Log Output ==="
cat /tmp/pius_launch.log | tail -n 30
echo "========================="

echo "Verifying generated HDF5 files..."
python3 /home/orin/dev/ws/colcon_ws/verify_h5.py
