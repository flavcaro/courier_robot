#!/bin/bash

echo "=== Avvio Missione Courier Robot con Behavior Tree ==="
echo ""

# Determina la directory corrente dello script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit

# Verifica e installa dipendenze Python
echo "📦 Verifica dipendenze Python..."
if ! python3 -c "import py_trees" 2>/dev/null; then
    echo "⚠️  py_trees non trovato. Installazione in corso..."
    pip install --user py_trees>=2.2.0
    echo "✅ py_trees installato!"
fi

# Verifica e installa OpenCV e cv_bridge per AprilTag
echo "📦 Verifica OpenCV per AprilTag..."
if ! python3 -c "import cv2" 2>/dev/null; then
    echo "⚠️  OpenCV non trovato. Installazione in corso..."
    pip install --user "numpy>=1.21.6,<1.28.0" opencv-python
    echo "✅ OpenCV installato!"
fi

# Installa cv_bridge per ROS2
if ! ros2 pkg list 2>/dev/null | grep -q "cv_bridge"; then
    echo "⚠️  cv_bridge non trovato. Installazione in corso..."
    sudo apt update
    sudo apt install -y ros-jazzy-cv-bridge ros-jazzy-image-transport
    echo "✅ cv_bridge installato!"
fi

# Genera AprilTag images se non esistono
echo "📦 Verifica AprilTag images..."
APRILTAG_DIR="$SCRIPT_DIR/src/courier_nav/courier_nav/apriltag_images"
if [ ! -d "$APRILTAG_DIR" ] || [ -z "$(ls -A $APRILTAG_DIR 2>/dev/null)" ]; then
    echo "⚠️  AprilTag images non trovate. Generazione in corso..."
    python3 "$SCRIPT_DIR/src/courier_nav/courier_nav/generate_apriltags.py"
    echo "✅ AprilTag images generate!"
else
    echo "✅ AprilTag images già presenti ($(ls $APRILTAG_DIR/*.png 2>/dev/null | wc -l) files)"
fi

# Sourcing ROS2
echo "📦 Sourcing ROS2..."
source /opt/ros/jazzy/setup.bash

# Build del workspace (se necessario)
echo "🔨 Building workspace..."
colcon build --symlink-install
source install/setup.bash

# 1. Avvia Gazebo + Bridge
echo ""
echo "1️⃣  Avvio Gazebo e Bridge..."
ros2 launch courier_description sim.launch.py &
SIM_PID=$!
sleep 6

# 2. Spawna la griglia di celle
echo "2️⃣  Spawning griglia (green=start, blue=goal, red=obstacle)..."
ros2 run courier_nav spawner
echo "    Griglia completata!"
sleep 2

# 3. Spawna il robot
echo "3️⃣  Spawning robot..."
ros2 run ros_gz_sim create -world empty -file "$SCRIPT_DIR/robot.sdf" -name courier_robot -x 0.5 -y 0.5 -z 0.15
sleep 3

# 4. Pubblica static transforms
echo "4️⃣  Pubblicazione TF statiche..."
# map -> odom
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
echo "5️⃣  Avvio AprilTag Localizer..."
ros2 run courier_nav apriltag_localizer --ros-args -p use_sim_time:=true &
APRILTAG_PID=$!
sleep 2

# 6. Avvia il mission controller con Behavior Tree
echo "6️⃣  Avvio Mission Controller (Behavior Tree)..."
ros2 run courier_nav nav2_mission_controller_bt --ros-args -p use_sim_time:=true &
MISSION_PID=$!

echo ""
echo "=== 🚀 Sistema avviato con Behavior Tree Controller! ==="
echo ""
echo "🌲 Architettura: Hierarchical Behavior Tree"
echo "📦 Struttura modulare:"
echo "   • behaviors/navigation.py  - Movimento e rotazione"
echo "   • behaviors/mission.py     - Raccolta e consegna"
echo "   • behaviors/obstacle.py    - Gestione ostacoli"
echo "   • controller.py            - Orchestrazione principale"
echo ""
echo "📍 Missione:"
echo "   Start:  Cella (0,0) → Coordinate (0.5, 0.5)"
echo "   Goal:   Cella (4,2) → Coordinate (4.5, 2.5)"
echo "   Return: Cella (0,0) → Coordinate (0.5, 0.5)"
echo ""
echo "🔍 Per vedere i comandi di velocità:"
echo "   ros2 topic echo /cmd_vel"
echo ""
echo "📷 Per monitorare AprilTag detection:"
echo "   ros2 topic echo /apriltag_pose"
echo ""
echo "📊 Per vedere lo stato odometry:"
echo "   ros2 topic echo /odom"
echo ""

# Attendi che tutti i processi terminino
wait $MISSION_PID
