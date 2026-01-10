#!/bin/bash

echo "=== Courier Robot Mission - Ubuntu Version ==="
echo ""

# Ottieni il percorso dello script e della workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="$SCRIPT_DIR"

cd "$ROS2_WS" || exit

# Verifica e installa dipendenze Python
echo "📦 Checking Python dependencies..."
if ! python3 -c "import py_trees" 2>/dev/null; then
    echo "⚠️  py_trees not found. Installing..."
    pip3 install --user py_trees>=2.2.0
    echo "✅ py_trees installed!"
fi

# Verifica OpenCV per rilevamento AprilTag
echo "📦 Checking OpenCV for AprilTag..."
if ! python3 -c "import cv2" 2>/dev/null; then
    echo "⚠️  OpenCV not found. Installing..."
    pip3 install --user "numpy>=1.21.6,<1.28.0" opencv-python
    echo "✅ OpenCV installed!"
fi

# Installa cv_bridge per ROS2
if ! ros2 pkg list 2>/dev/null | grep -q "cv_bridge"; then
    echo "⚠️  cv_bridge not found. Installing..."
    sudo apt update
    sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport
    echo "✅ cv_bridge installed!"
fi

# Genera immagini AprilTag se non esistono
echo "📦 Checking AprilTag images..."
APRILTAG_DIR="$ROS2_WS/src/courier_nav/courier_nav/apriltag_images"
if [ ! -d "$APRILTAG_DIR" ] || [ -z "$(ls -A $APRILTAG_DIR 2>/dev/null)" ]; then
    echo "⚠️  AprilTag images not found. Generating..."
    python3 "$ROS2_WS/src/courier_nav/courier_nav/generate_apriltags.py"
    echo "✅ AprilTag images generated!"
else
    echo "✅ AprilTag images present ($(ls $APRILTAG_DIR/*.png 2>/dev/null | wc -l) files)"
fi

# Source ROS2
echo "📦 Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash

# Pulisci build artifacts precedenti
echo "🧹 Cleaning previous build artifacts..."
rm -rf build/ install/ log/
echo "✅ Clean complete!"

# Build workspace
echo "🔨 Building workspace..."
colcon build

if [ $? -ne 0 ]; then
    echo "❌ Build failed! Check the errors above."
    exit 1
fi

# Copy AprilTag images (fix symlink issue for Gazebo)
echo "📋 Copying AprilTag images for Gazebo..."
APRILTAG_SRC="$ROS2_WS/src/courier_nav/courier_nav/apriltag_images"
APRILTAG_INSTALL="$ROS2_WS/install/courier_nav/share/courier_nav/apriltag_images"
if [ -d "$APRILTAG_SRC" ] && [ -d "$APRILTAG_INSTALL" ]; then
    # Remove symlinks and copy real files
    rm -f "$APRILTAG_INSTALL"/*.png
    cp "$APRILTAG_SRC"/*.png "$APRILTAG_INSTALL"/
    echo "✅ AprilTag images copied ($(ls $APRILTAG_INSTALL/*.png 2>/dev/null | wc -l) files)"
fi

source install/setup.bash

# 1. Avvia Gazebo + Bridge
echo ""
echo "1️⃣  Starting Gazebo and ROS Bridge..."
ros2 launch courier_description sim.launch.py &
SIM_PID=$!
sleep 6

# 2. Spawn il mondo griglia
echo "2️⃣  Spawning grid world (green=start, blue=goal, red=obstacle)..."
ros2 run courier_nav spawner
echo "    Grid complete!"
sleep 2

# 3. Spawn il robot
echo "3️⃣  Spawning robot..."
ros2 run ros_gz_sim create -world empty -file "$ROS2_WS/robot.sdf" -name courier_robot -x 0.5 -y 0.5 -z 0.15
sleep 3

# 4. Pubblica trasformazioni statiche
echo "4️⃣  Publishing static TF transforms..."
# map -> odom (identity)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
TF_MAP_PID=$!
# base_link -> base_footprint
ros2 run tf2_ros static_transform_publisher 0 0 0.1 0 0 0 base_link base_footprint &
TF_BASE_PID=$!
# base_link -> lidar_link (lidar è 20cm sopra base_link)
ros2 run tf2_ros static_transform_publisher 0 0 0.2 0 0 0 base_link lidar_link &
TF_LIDAR_PID=$!
# base_link -> camera_link (camera per AprilTag)
ros2 run tf2_ros static_transform_publisher 0.15 0 0.15 0 0 0 base_link camera_link &
TF_CAMERA_PID=$!
sleep 2

# 5. Avvia AprilTag Localizer
echo "5️⃣  Starting AprilTag Localizer..."
ros2 run courier_nav apriltag_localizer --ros-args -p use_sim_time:=true &
APRILTAG_PID=$!
sleep 2

# 6. Avvia il Behavior Tree Mission Controller
echo "6️⃣  Starting Behavior Tree Mission Controller..."
ros2 run courier_nav mission_controller --ros-args -p use_sim_time:=true &
MISSION_PID=$!

echo ""
echo "=== 🚀 Courier Robot Mission Started! ==="
echo ""
echo "🌲 Architecture: Hierarchical Behavior Tree (py_trees)"
echo "📦 Modular structure:"
echo "   • behaviors/navigation.py  - Cell centering and rotation"
echo "   • behaviors/mission.py     - Pickup and delivery simulation"
echo "   • behaviors/obstacle.py    - Obstacle handling with replanning"
echo "   • behaviors/battery.py     - Battery management"
echo "   • controller.py            - Main behavior tree orchestration"
echo ""
echo "📍 Mission:"
echo "   Start:  Cell (0,0) → World (0.5, 0.5)"
echo "   Goal:   Cell (4,2) → World (4.5, 2.5)"
echo "   Return: Cell (0,0) → World (0.5, 0.5)"
echo ""
echo "🔧 Features:"
echo "   • BFS pathfinding on 5x5 grid"
echo "   • LIDAR obstacle detection"
echo "   • AprilTag localization corrections"
echo "   • Cell-by-cell navigation with centering"
echo "   • Simulated pickup and delivery actions"
echo ""
echo "📊 Monitoring commands:"
echo "   ros2 topic echo /cmd_vel          # Velocity commands"
echo "   ros2 topic echo /odom             # Robot odometry"
echo "   ros2 topic echo /scan             # LIDAR data"
echo "   ros2 topic echo /apriltag_pose    # AprilTag corrections"
echo "   ros2 topic echo /path_markers     # Visualization markers"
echo ""
echo "Press Ctrl+C to stop all processes"
echo ""

# Funzione di cleanup
cleanup() {
    echo ""
    echo "🛑 Stopping all processes..."
    kill $MISSION_PID $APRILTAG_PID $TF_CAMERA_PID $TF_LIDAR_PID $TF_BASE_PID $TF_MAP_PID $SIM_PID 2>/dev/null
    pkill -P $$
    wait 2>/dev/null
    echo "✅ All processes stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Attendi il completamento della missione
wait $MISSION_PID
