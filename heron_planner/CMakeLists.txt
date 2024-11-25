cmake_minimum_required(VERSION 2.8.3)
project(heron_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib_msgs geometry_msgs std_msgs)

catkin_python_setup()

catkin_package()
