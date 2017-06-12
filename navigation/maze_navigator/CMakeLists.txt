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
  tf
)

find_package(Boost REQUIRED COMPONENTS system)

#add_action_files(
#  DIRECTORY action
#  FILES Navigate.action
#)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(maze_navigator src/ring_pickup.cpp)

target_link_libraries(
  maze_navigator
  ${catkin_LIBRARIES}
)

#add_dependencies(
#  navigation_server
#  ${maze_navigator_EXPORTED_TARGETS}
#)

## Install project namespaced headers
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)
