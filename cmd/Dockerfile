# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM arm64v8/ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*


# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install venv
RUN apt-get update && apt-get install -y \
    python3-venv \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
    
# create venv environment
RUN python3 -m venv --system-site-packages /opt/pc_venv

# install numpy>2.0.0 in venv
RUN /opt/pc_venv/bin/pip install --upgrade pip && \
    /opt/pc_venv/bin/pip install numpy==2.2.6 open3d scikit-learn dbscan
    
# install vim
RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    && rm -rf /var/lib/apt/lists/*
    
# install openGL
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
