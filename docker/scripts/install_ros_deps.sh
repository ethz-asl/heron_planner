#!/bin/bash
set -x
# set -o pipefail

DOCKER_WS=/root/hlp_deps_ws/

# Install ROS packages
apt-get update && apt-get install -y \
	python3-catkin-tools \
	python3-osrf-pycommon \
	ros-noetic-catkin \
    ros-noetic-py-trees \
    ros-noetic-py-trees-ros \
    ros-noetic-rqt-py-trees \
    ros-noetic-tf-conversions \
    ros-noetic-rviz-visual-tools

# Set up the workspace
mkdir -p $DOCKER_WS/src
cd $DOCKER_WS
source /opt/ros/noetic/setup.bash

catkin init
catkin config --extend /root/hlp_ws/devel # extend the host workspace
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Fetch repositories using vcs
cd $DOCKER_WS/src || exit 1
vcs import --recursive --input $SCRIPTS_PATH/ros_deps.repos

# Install Python dependencies
pip3 install kafka-python scipy

# Clean up
rm -rf /var/lib/apt/lists/*
