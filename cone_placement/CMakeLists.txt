cmake_minimum_required(VERSION 3.0.0)
project(cone_placement)

find_package(catkin REQUIRED message_generation)
catkin_python_setup()

# add_compile_options(-std=c++17 -Wall -Wextra)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  std_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  rostest
  py_trees

)
#############
# LIBRARIES #
# #############
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

############
# BINARIES #
############
# add_executable(${PROJECT_NAME}_node, src/node.cpp)
# target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})


##########
# EXPORT #
##########
