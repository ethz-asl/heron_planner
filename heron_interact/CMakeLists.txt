cmake_minimum_required(VERSION 3.1)
project(heron_interact)

find_package(catkin REQUIRED COMPONENTS
  rqt_gui
  rqt_gui_py
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()
# catkin_python_setup()

# catkin_install_python(PROGRAMS
#   src/heron_interact/image_path_selector.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
catkin_install_python(PROGRAMS
  src/heron_interact/heron_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/mymodule
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
