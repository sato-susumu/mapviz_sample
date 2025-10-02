#!/bin/bash
# Stop all mapviz and GPS publisher processes

echo "Stopping all mapviz and GPS publisher processes..."

# Kill only ROS2-related processes by pattern, not killall
pkill -f "ros2 launch"
pkill -f "ros2 run"
pkill -f "ros2 topic pub"
pkill -f "mapviz"
pkill -f "static_transform_publisher"
pkill -f "initialize_origin.py"
pkill -f "publish_osaka.sh"
pkill -f "publish_tokyo.sh"
pkill -f "publish_newyork.sh"

# Wait for graceful shutdown
sleep 2

# Force kill any remaining processes
pkill -9 -f "ros2 launch"
pkill -9 -f "ros2 run"
pkill -9 -f "ros2 topic pub"
pkill -9 -f "mapviz"
pkill -9 -f "static_transform_publisher"
pkill -9 -f "initialize_origin.py"

# Clean up shared memory only (keep ~/.mapviz_config)
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

echo "All processes stopped and cache cleaned."
