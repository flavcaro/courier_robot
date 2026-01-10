#!/bin/bash

echo "=== AprilTag Diagnostics ==="
echo ""

echo "1. Checking ROS2 topics..."
echo "   Camera topics:"
ros2 topic list | grep -E "camera|image"
echo ""

echo "2. Checking topic rates (5 seconds)..."
timeout 5 ros2 topic hz /camera 2>&1 | head -5 &
timeout 5 ros2 topic hz /camera_info 2>&1 | head -5 &
timeout 5 ros2 topic hz /apriltag_pose 2>&1 | head -5 &
wait
echo ""

echo "3. Checking camera output (1 message)..."
timeout 2 ros2 topic echo /camera --once 2>&1 | head -10
echo ""

echo "4. Checking apriltag_pose output (1 message)..."
timeout 2 ros2 topic echo /apriltag_pose --once 2>&1 | head -10
echo ""

echo "5. Checking for AprilTag images..."
if [ -d "$PWD/src/courier_nav/courier_nav/apriltag_images" ]; then
    echo "   Found: $(ls $PWD/src/courier_nav/courier_nav/apriltag_images/*.png 2>/dev/null | wc -l) PNG files"
    ls -lh $PWD/src/courier_nav/courier_nav/apriltag_images/ | head -10
else
    echo "   ❌ AprilTag images directory not found!"
fi
echo ""

echo "6. Checking Gazebo world entities..."
gz model list 2>&1 | grep -E "apriltag|courier" | head -10
echo ""

echo "=== Diagnostics Complete ==="
