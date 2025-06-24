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
    iputils-ping \
    gnupg2 \
    lsb-release \
    sudo \
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
    rosdep update

# Install GUI/X11 dependencies
RUN apt-get install -y \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    sshpass && \
    rm -rf /var/lib/apt/lists/*

# Create a non-root user and add to sudoers (matching host user)
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

# Configure X11 and GPU
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV QT_X11_NO_MITSHM=1

# Switch to the user
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS in .bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
CMD ["bash"]
