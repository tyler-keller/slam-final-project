# following prereqs listed:
# 1. Prerequisites
# 1.1 Ubuntu and ROS
# Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic. ROS Installation
# 1.2. Ceres Solver
# 1.3. PCL
FROM ros:melodic

RUN echo "new run"

SHELL ["/bin/bash", "-c"]
RUN bash -c "source ~/.bashrc"

ENV DEBIAN_FRONTEND=noninteractive

RUN sudo apt-get update
RUN sudo apt-get -y install build-essential
RUN sudo apt-get -y install curl
RUN sudo apt-get -y install git

# ceres and pcl libraries
# CMake
RUN sudo apt-get -y install cmake
# google-glog + gflags
RUN sudo apt-get -y install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
RUN sudo apt-get -y install libatlas-base-dev
# Eigen3
RUN sudo apt-get -y install libeigen3-dev
# SuiteSparse (optional)
RUN sudo apt-get -y install libsuitesparse-dev
RUN sudo apt-get -y install libceres-dev
RUN sudo apt-get -y install libpcl-dev

# ros dependencies
RUN sudo apt-get -y install python-catkin-tools
RUN sudo apt-get -y install ros-${ROS_DISTRO}-catkin
RUN sudo apt-get -y install ros-${ROS_DISTRO}-image-transport
RUN sudo apt-get -y install ros-${ROS_DISTRO}-cv-bridge
RUN sudo apt-get -y install ros-${ROS_DISTRO}-message-filters
RUN sudo apt-get -y install ros-${ROS_DISTRO}-tf
RUN sudo apt-get -y install ros-${ROS_DISTRO}-pcl-conversions
RUN sudo apt-get -y install ros-${ROS_DISTRO}-rviz
RUN sudo apt-get -y install ros-${ROS_DISTRO}-perception
RUN sudo apt-get -y install ros-${ROS_DISTRO}-rqt-graph

# mesa for x11 stuff
RUN sudo apt-get -y install mesa-utils libgl1-mesa-glx libgl1-mesa-dri

# evo and python
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa -y && apt-get update
RUN apt-get install -y python3.7 python3-pip
RUN python3.7 -m pip install --upgrade pip
RUN python3.7 -m pip install evo

# tmux for better terminal windows
RUN apt-get install tmux -y

WORKDIR "/root/catkin_ws/src"
RUN git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
WORKDIR "/root/catkin_ws"
RUN source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make -j1
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc 
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc