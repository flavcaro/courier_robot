#!/bin/bash

echo "=== Avvio Simulazione Courier Robot ==="
echo ""

# Path del workspace
WS_PATH="$HOME/ros2_ws"

# 1. Avvia Gazebo in background
echo "1. Avvio Gazebo..."
source /opt/ros/jazzy/setup.bash
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 launch ros_gz_sim gz_sim.launch.py gz_args:='-r empty.sdf'; exec bash"

# Aspetta che Gazebo sia pronto
sleep 5

# 2. Spawna la griglia
echo "2. Spawno la griglia..."
gnome-terminal -- bash -c "cd $WS_PATH; source install/setup.bash; ros2 run courier_nav spawner; sleep 3; exec bash"

# Aspetta che la griglia sia spawnata
sleep 3

# 3. Avvia il bridge
echo "3. Avvio il bridge ROS2-Gazebo..."
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist /odom@nav_msgs/msg/Odometry@gz.msgs.Odometry /imu@sensor_msgs/msg/Imu@gz.msgs.IMU /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V; exec bash"

# Aspetta che il bridge sia attivo
sleep 2

# 4. Spawna il robot sulla posizione START (0.5, 0.5)
echo "4. Spawno il robot..."
cd $WS_PATH
ros2 run ros_gz_sim create -world empty -file robot.sdf -name courier_robot -x 0.5 -y 0.5 -z 0.2

# Aspetta che il robot sia pronto
sleep 2

# 5. Avvia il controller
echo "5. Avvio il controller..."
gnome-terminal -- bash -c "cd $WS_PATH; source install/setup.bash; ros2 run courier_nav controller --ros-args -p use_sim_time:=true; exec bash"

echo ""
echo "=== Sistema avviato! ==="
echo "- Il robot (grigio) parte dal quadrato VERDE (0,0)"
echo "- E va verso il quadrato BLU (4,2)"
echo "- Evitando i cubi ROSSI (ostacoli)"
echo ""
echo "Per fermare tutto: pkill -f 'ros2|gz sim'"
