# Courier Robot - ROS2 Jazzy + Gazebo Harmonic

Sistema di navigazione autonoma con Behavior Tree per robot courier.

## 🚀 Setup Iniziale (Prima Esecuzione)

Nel container Docker, eseguire:

```bash
cd ~/ros2_ws

# 1. Installa dipendenze Python
chmod +x setup_dependencies.sh
./setup_dependencies.sh

# 2. Compila i pacchetti
colcon build

# 3. Source del workspace
source install/setup.bash
```

## ▶️ Avvio Simulazione

```bash
cd ~/ros2_ws
./start_docker.sh
```

## 📦 Dipendenze

- ROS2 Jazzy
- Gazebo Harmonic
- py_trees >= 2.2.0

## 📁 Struttura

- `courier_description/` - URDF/SDF del robot
- `courier_control/` - Controllore PID
- `courier_nav/` - Behavior Tree e navigazione
- `courier_robot/` - Metapackage