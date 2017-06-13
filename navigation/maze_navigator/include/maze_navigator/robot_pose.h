#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <maze_navigator/costmap.h>

class RobotPose
{
  public:
    int x_, y_;
    double wx_, wy_, wz_;

    tf::Vector3 rot_;
    tf::Quaternion rot_quat_;

    bool init_;

    RobotPose() { init_ = false; }
    RobotPose(int py_, int px_, Costmap &costmap);
    RobotPose(int py_, int px_, tf::Vector3 rotvec, Costmap &costmap);
    RobotPose(int py_, int px_, tf::Vector3 rotvec, double angle_2d, Costmap &costmap);

    void update(tf::StampedTransform &transform, Costmap &costmap);
};

#endif