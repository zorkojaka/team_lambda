#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "ros/ros.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <maze_navigator/costmap.h>
#include <maze_navigator/expl_map.h>
#include <maze_navigator/expl_planner.h>

void visualizeLayer(const ros::Publisher &vis_pub, const Explmap &expl_map, Costmap& costmap, int layer_flag,
                    visualization_msgs::Marker point_cloud);

void visualizeMultipleLayers(const ros::Publisher &vis_pub, const Explmap &expl_map, Costmap& costmap);

void visualizeSinglePoint(const ros::Publisher &vis_pub, RobotPose& r_pos);
#endif