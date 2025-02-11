#!/bin/bash
set -o pipefail

# this script builds all the installed ROS packages and sets up the bashrc.
cd $HERON_DEP_WS
catkin build -c

# add sourcing of the repo to the ~/.bashrc
echo "source $HERON_DEP_WS/devel/setup.bash" >> ~/.bashrc
echo 'ROSBASH=/root/moma_ws/devel/setup.bash && [ -e "$ROSBASH" ] && source $ROSBASH' >> ~/.bashrc
