# Courier Robot Docker Image with Nav2
# Based on tiryoh/ros2-desktop-vnc:jazzy

FROM tiryoh/ros2-desktop-vnc:jazzy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Fix for apt mirror sync issues - clean and retry
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get update --fix-missing || apt-get update || true

# Install Nav2 and dependencies (with retry logic)
RUN apt-get update && apt-get install -y --no-install-recommends \
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
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    || (apt-get update --fix-missing && apt-get install -y --no-install-recommends \
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
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport) \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (with compatible numpy version)
RUN pip3 install --break-system-packages \
    "numpy>=1.21.6,<1.28.0" \
    opencv-python \
    py_trees>=2.2.0

# Set working directory
WORKDIR /home/ubuntu/ros2_ws

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then source /home/ubuntu/ros2_ws/install/setup.bash; fi" >> /home/ubuntu/.bashrc

# Reset frontend
ENV DEBIAN_FRONTEND=dialog

# Default command
CMD ["/bin/bash"]
