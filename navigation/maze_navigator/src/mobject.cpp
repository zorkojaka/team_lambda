#include "ros/ros.h"

#include <maze_navigator/mobject.h>

void MObject::facePositionCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    ROS_INFO("%d face poses", (int)msg->poses.size());
    for(int i = 0;i < msg->poses.size();i++){
        ROS_INFO("\t(%lf %lf)",msg->poses[i].position.x,msg->poses[i].position.y);
    }
}