#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <maze_navigator/costmap>

class RobotPose
{
  public:
    int x_, y_;
    double wx_, wy_, wz_;

    tf::Vector3 rot_;
    tf::Quaternion rot_quat_;

    bool init_;
    RobotPose() { init_ = false; }
    RobotPose(int py_, int px_, CostMap &costmap)
    {
        init_ = true;
        x_ = px_;
        y_ = py_;
        costmap.fromPixel(y_, x_, wy_, wx_);
    }

    RobotPose(int py_, int px_, tf::Vector3 rotvec, CostMap &costmap)
    {
        init_ = true;
        x_ = px_;
        y_ = py_;
        rot_ = rotvec;
        costmap.fromPixel(y_, x_, wy_, wx_);
    }

    RobotPose(int py_, int px_, tf::Vector3 rotvec, double angle_2d, CostMap &costmap)
    {
        init_ = true;
        x_ = px_;
        y_ = py_;
        rot_ = rotvec;

        costmap.fromPixel(y_, x_, wy_, wx_);
        rot_quat_ = constructRotation(angle_2d);
    }

    void update(tf::StampedTransform &transform, CostMap &costmap)
    {
        rot_quat_ = transform.getRotation();

        tf::Vector3 y_axis(0, 1, 0);
        tf::Vector3 z_axis(0, 0, 1);

        rot_ = tf::quatRotate(rot_quat_, y_axis);
        rot_ = rot_.rotate(z_axis, -M_PI / 2.0);

        wy_ = transform.getOrigin().y();
        wx_ = transform.getOrigin().x();
        wz_ = transform.getOrigin().z();

        costmap.fromWorld(wy_, wx_, y_, x_);
        init_ = true;
    }
};

#endif