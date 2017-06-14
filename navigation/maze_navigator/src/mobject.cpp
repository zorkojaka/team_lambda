#include "ros/ros.h"

#include <maze_navigator/mobject.h>


bool MObject::hasNewFace(){
    return (last_call_size_ < face_pos_.size());
}

std::pair<double,double> MObject::getLastFacePos(){
    last_call_size_ = face_pos_.size();
    return face_pos_[face_pos_.size() - 1];
}
        
void MObject::facePositionCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    ROS_INFO("%d face poses", (int)msg->poses.size());

    if(msg->poses.size() > face_pos_.size()){
        for(int i = face_pos_.size(); i < msg->poses.size();i++){
            face_pos_.push_back(std::make_pair(msg->poses[i].position.y, msg->poses[i].position.x));
        }
    }

    for(int i = 0;i < msg->poses.size();i++){
        ROS_INFO("\t(%lf %lf)",msg->poses[i].position.x,msg->poses[i].position.y);
    }
}