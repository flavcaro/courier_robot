#!/bin/bash

echo "=== Avvio Missione Courier Robot con Nav2 ==="
echo ""

# Usa la path corretta dentro il container
cd /home/ubuntu/ros2_ws || exit

# Verifica e installa dipendenze Python
echo "📦 Verifica dipendenze Python..."
if ! python3 -c "import py_trees" 2>/dev/null; then
    echo "⚠️  py_trees non trovato. Installazione in corso..."
    pip install --break-system-packages py_trees>=2.2.0
    echo "✅ py_trees installato!"
fi

# Verifica e installa OpenCV e cv_bridge per AprilTag
echo "📦 Verifica OpenCV per AprilTag..."
if ! python3 -c "import cv2" 2>/dev/null; then
    echo "⚠️  OpenCV non trovato. Installazione in corso..."
    pip install --break-system-packages "numpy>=1.21.6,<1.28.0" opencv-python
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
APRILTAG_DIR="/home/ubuntu/ros2_ws/src/courier_nav/courier_nav/apriltag_images"
if [ ! -d "$APRILTAG_DIR" ] || [ -z "$(ls -A $APRILTAG_DIR 2>/dev/null)" ]; then
    echo "⚠️  AprilTag images non trovate. Generazione in corso..."
    python3 /home/ubuntu/ros2_ws/src/courier_nav/courier_nav/generate_apriltags.py
    echo "✅ AprilTag images generate!"
else
    echo "✅ AprilTag images già presenti ($(ls $APRILTAG_DIR/*.png 2>/dev/null | wc -l) files)"
fi

# Verifica e installa Nav2
echo "📦 Verifica Nav2..."
NAV2_CHECK=$(ros2 pkg list 2>/dev/null | grep "nav2_bringup" || true)
if [ -z "$NAV2_CHECK" ]; then
    echo "⚠️  Nav2 non trovato. Installazione in corso..."
    sudo apt update
    sudo apt install -y \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-nav2-simple-commander \
        ros-jazzy-nav2-msgs \
        ros-jazzy-nav2-lifecycle-manager \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-amcl \
        ros-jazzy-nav2-controller \
        ros-jazzy-nav2-planner \
        ros-jazzy-nav2-behaviors \
        ros-jazzy-nav2-bt-navigator \
        ros-jazzy-nav2-waypoint-follower \
        ros-jazzy-nav2-smoother \
        ros-jazzy-nav2-velocity-smoother \
        ros-jazzy-nav2-costmap-2d \
        ros-jazzy-nav2-core \
        ros-jazzy-nav2-util \
        ros-jazzy-nav2-common \
        ros-jazzy-dwb-core \
        ros-jazzy-dwb-plugins \
        ros-jazzy-dwb-critics \
        ros-jazzy-nav2-navfn-planner \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-tf2-ros
    echo "✅ Nav2 installato!"
else
    echo "✅ Nav2 già installato"
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
ros2 run ros_gz_sim create -world empty -file /home/ubuntu/ros2_ws/robot.sdf -name courier_robot -x 0.5 -y 0.5 -z 0.15
sleep 3

# 4. Pubblica static transforms
# Nota: Robot State Publisher non è necessario perché Gazebo già pubblica i TF
echo "4️⃣  Pubblicazione TF statiche..."
# map -> odom (AMCL lo sovrascriverà quando localizzato)
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

# 5. Avvia Nav2 stack
echo "5️⃣  Avvio Nav2 Navigation Stack..."
ros2 launch courier_nav nav2_bringup.launch.py use_sim_time:=true autostart:=true &
NAV2_PID=$!
sleep 10

# 6. Avvia AprilTag Localizer
echo "6️⃣  Avvio AprilTag Localizer..."
ros2 run courier_nav apriltag_localizer --ros-args -p use_sim_time:=true &
APRILTAG_PID=$!
sleep 2

# 7. Avvia il mission controller (scegli versione)
echo "7️⃣  Avvio Mission Controller..."
echo ""
echo "   📌 Versioni disponibili:"
echo "   • nav2_mission_controller     = State Machine (legacy, monolitico)"
echo "   • nav2_mission_controller_bt  = Behavior Tree (modulare ✨)"
echo ""
echo "   ▶️  Avvio versione: Behavior Tree (Modular Architecture)"
ros2 run courier_nav nav2_mission_controller_bt --ros-args -p use_sim_time:=true &
MISSION_PID=$!

echo ""
echo "=== 🚀 Sistema avviato con Modular Behavior Tree Controller! ==="
echo ""
echo "🌲 Architettura: Hierarchical Behavior Tree (Refactored)"
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
echo "💡 Per usare la versione State Machine (legacy):"
echo "   Modifica: nav2_mission_controller_bt → nav2_mission_controller"
echo ""
echo "📚 Documentazione:"
echo "   • REFACTORING_SUMMARY.md - Panoramica completa"
echo "   • ARCHITECTURE_DIAGRAM.md - Diagrammi e struttura"
echo "   • DEVELOPER_GUIDE.md - Guida rapida sviluppatore"
echo ""
echo "📊 Per monitorare Nav2:"
echo "   ros2 topic echo /navigate_to_pose/_action/status"
echo ""
echo "📷 Per monitorare AprilTag detection:"
echo "   ros2 topic echo /apriltag_pose"
echo ""
echo "🗺️  Per visualizzare la costmap:"
echo "   ros2 topic echo /global_costmap/costmap"
echo ""
echo "🔍 Per vedere i comandi di velocità:"
echo "   ros2 topic echo /cmd_vel"
echo ""

# Attendi che tutti i processi terminino
wait $MISSION_PID
