# offical ROS noetic base image
FROM osrf/ros:noetic-desktop

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV HERON_DEP_WS=/root/heron_dep_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general dep installation
RUN scripts/install_sys_deps.sh

# run the ROS workspace set-up and dep installation
RUN scripts/install_ros_deps.sh

# finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
