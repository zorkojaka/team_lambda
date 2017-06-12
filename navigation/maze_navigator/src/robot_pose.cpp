#include <maze_navigator/geom_util>
#include <maze_navigator/robot_pose>

RobotPose::RobotPose() { init_ = false; }
RobotPose::RobotPose(int py_, int px_, Costmap &costmap)
{
    init_ = true;
    x_ = px_;
    y_ = py_;
    costmap.fromPixel(y_, x_, wy_, wx_);
}

RobotPose::RobotPose(int py_, int px_, tf::Vector3 rotvec, Costmap &costmap)
{
    init_ = true;
    x_ = px_;
    y_ = py_;
    rot_ = rotvec;
    costmap.fromPixel(y_, x_, wy_, wx_);
}

RobotPose::RobotPose(int py_, int px_, tf::Vector3 rotvec, double angle_2d, Costmap &costmap)
{
    init_ = true;
    x_ = px_;
    y_ = py_;
    rot_ = rotvec;

    costmap.fromPixel(y_, x_, wy_, wx_);
    rot_quat_ = constructRotation(angle_2d);
}

void RobotPose::update(tf::StampedTransform &transform, Costmap &costmap)
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