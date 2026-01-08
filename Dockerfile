# Courier Robot Docker Image with Behavior Tree Navigation
# Based on tiryoh/ros2-desktop-vnc:jazzy

FROM tiryoh/ros2-desktop-vnc:jazzy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Fix for apt mirror sync issues - clean and retry
RUN rm -rf /var/lib/apt/lists/* && \
    apt-get clean && \
    apt-get update --fix-missing || apt-get update || true

# Install ROS2 dependencies (with retry logic)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    || (apt-get update --fix-missing && apt-get install -y --no-install-recommends \
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
