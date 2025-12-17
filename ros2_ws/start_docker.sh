#!/bin/bash

echo "=== Avvio Missione Courier Robot ==="

# Usa la path corretta dentro il container
cd /home/ubuntu/ros2_ws || exit

# Verifica e installa dipendenze Python
echo "📦 Verifica dipendenze Python..."
if ! python3 -c "import py_trees" 2>/dev/null; then
    echo "⚠️  py_trees non trovato. Installazione in corso..."
    pip install --break-system-packages py_trees>=2.2.0
    echo "✅ py_trees installato!"
fi

# Sourcing ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Avvio nodi in background
ros2 launch courier_description sim.launch.py &
sleep 5

ros2 run courier_nav spawner &
sleep 3

echo "🤖 Spawning robot..."
ros2 run ros_gz_sim create -world empty -file /home/ubuntu/ros2_ws/robot.sdf -name courier_robot -x 0.5 -y 0.5 -z 0.15
sleep 2
 
ros2 run courier_nav mission_controller &

echo "Missione avviata!"
wait
