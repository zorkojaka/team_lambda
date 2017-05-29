cmake_minimum_required(VERSION 2.8.3)
project(exercise6)

## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
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
  nav_msgs
  geometry_msgs
)

## Add include directories
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    sensor_msgs
    std_msgs
  DEPENDS
    Boost
    Eigen
    PCL
)

add_executable (find_red src/find_red.cpp)
add_executable (find_grn src/find_grn.cpp)
add_executable (find_blu src/find_blu.cpp)
add_executable (seperate src/seperateByColorAndDistance.cpp)
add_executable (cloud_forwarder src/forwarder.cpp)
add_executable (cloud_voxelgrid src/voxelgrid.cpp)
add_executable (find_plane src/find_plane.cpp)
add_executable (find_cyl src/find_cyl.cpp)
add_executable (mark_on_map src/mark_on_map.cpp)
#add_executable (rand_goal src/rand_goal.cpp)
add_executable (map_goalz src/map_goals.cpp)
#add_executable (goal_loop src/goal_loop.cpp)

