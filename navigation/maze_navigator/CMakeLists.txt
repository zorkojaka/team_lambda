cmake_minimum_required(VERSION 2.8.3)
project(maze_navigator)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  move_base_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cv_bridge
  tf
)

catkin_package()

find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(maze_exploration src/costmap.cpp src/expl_map.cpp src/expl_planner.cpp 
  src/robot_pose.cpp src/visualizer.cpp)


add_executable(maze_navigator src/maze_navigator.cpp)

target_link_libraries(
  maze_navigator
  maze_exploration
  ${catkin_LIBRARIES}
)

