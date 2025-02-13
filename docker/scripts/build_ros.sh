#!/bin/bash
set -o pipefail

# Source ROS
source /opt/ros/noetic/setup.bash

# Build the workspace
cd $DOCKER_WS
catkin build -c

# Source both workspaces
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /root/hlp_ws/devel/setup.bash --extend" >> ~/.bashrc
echo "source /root/hlp_deps_ws/devel/setup.bash --extend" >> ~/.bashrc
