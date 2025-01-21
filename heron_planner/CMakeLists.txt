cmake_minimum_required(VERSION 3.1)
project(heron_planner)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  # heron_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
