# offical ROS noetic base image
FROM osrf/ros:noetic-desktop

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV ROS_WS=/root/hlp_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general dep installation
RUN ${SCRIPTS_PATH}/install_sys_deps.sh

# run the ROS workspace set-up and dep installation
RUN ${SCRIPTS_PATH}/install_ros_deps.sh

# finally, build all the stuff we downloaded.
RUN ${SCRIPTS_PATH}/build_ros.sh
