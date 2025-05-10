# Use NVIDIA CUDA base image (Ubuntu 20.04)
FROM nvcr.io/nvidia/pytorch:23.04-py3

# Set up environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install ROS Noetic
RUN apt-get update && \
    apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool && \
    rosdep init && \
    rosdep update && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Install GUI dependencies (for RViz/Gazebo)
RUN apt-get install -y \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri && \
    rm -rf /var/lib/apt/lists/*

# Configure X11 forwarding
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV QT_X11_NO_MITSHM=1

# Create a workspace directory
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Entrypoint
CMD ["bash"]
