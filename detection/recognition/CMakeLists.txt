cmake_minimum_required(VERSION 2.8.3)
project(recognition)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  roscpp
  cv_bridge
  detection_msgs
)

## Add include directories
include_directories(
  ${Boost_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
)
catkin_package()



link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_executable (descriptorMaker src/descriptorMaker.cpp)
add_executable (distanceCalculator src/distanceCalculator.cpp)
target_link_libraries(descriptorMaker ${catkin_LIBRARIES} dlib)

