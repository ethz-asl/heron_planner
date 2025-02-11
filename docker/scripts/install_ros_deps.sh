#!/bin/bash
set -o pipefail

# set up a ROS workspace
mkdir -p $HERON_DEP_WS/src
cd $HERON_DEP_WS
source /opt/ros/noetic/setup.bash
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# install ROS packages from apt
apt-get update && apt-get install -y \
	ros-noetic-py-trees \
	ros-noetic-py-trees-ros \
	ros-noetic-rqt-py-trees \
	ros-noetic-tf-conversions \
	ros-noetic-rviz-visual-tools \

# Install all the other dependencies in the moma_dep_ws
cd $HERON_DEP_WS/src || exit 1
vcs import --recursive --input $SCRIPTS_PATH/ros_deps.repos

# pip install some stuff.
pip3 install kafka-python

# add a rundemo alias:
# echo 'alias rundemo="roslaunch grasp_demo grasp_demo.launch launch_rviz:=true"' >> ~/.bashrc

# Clear cache to keep layer size down
rm -rf /var/lib/apt/lists/*
