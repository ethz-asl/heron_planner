cmake_minimum_required(VERSION 3.0.2)
project(heron_msgs)

find_package(catkin_simple REQUIRED) 
catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
