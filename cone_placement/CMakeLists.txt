cmake_minimum_required(VERSION 3.0.0)
project(cone_placement)

find_package(catkin_simple REQUIRED message_generation)
catkin_simple(ALL_DEPS_REQUIRED)

# add_compile_options(-std=c++17 -Wall -Wextra)

# catkin_python_setup()

#############
# LIBRARIES #
#############
# cs_add_library(${PROJECT_NAME}
  # src/simple_navigation_goals.cpp
# )

############
# BINARIES #
############

# cs_add_executable(move_base_action
  # src/move_base_action.cpp
# )
# target_link_libraries(move_base_action ${catkin_LIBRARIES})


##########
# EXPORT #
##########
cs_install()
cs_export()
