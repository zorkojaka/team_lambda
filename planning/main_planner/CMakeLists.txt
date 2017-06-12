cmake_minimum_required(VERSION 2.8.3)
project(main_planner)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(maze_navigator REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${maze_navigator_INCLUDE_DIRS}
)
#add_executable(main_planner src/main_planner.cpp)
