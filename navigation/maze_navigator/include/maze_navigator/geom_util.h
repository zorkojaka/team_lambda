#ifndef GEOM_UTIL_H
#define GEOM_UTIL_H

#include <cmath>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

/*
    Constructs rotation quaternion from angle between rotation vector
    and map's y axis.
*/
tf::Quaternion constructRotation(double angle_2d){
    angle_2d += M_PI / 2.0;
    double mag = sqrt(sin(angle_2d  / 2.0) * sin(angle_2d  / 2.0) +
          cos(angle_2d  / 2.0) * cos(angle_2d  / 2.0));
    tf::Quaternion rot_quat_ = tf::Quaternion(0.0, 0.0, 
            sin(angle_2d  / 2.0) / mag, cos(angle_2d / 2.0) / mag);
    return rot_quat_;
}

/*
    Rotation vector to rotation angle.
    @returns Angle in radians.
*/
double rotVecToAngle(tf::Vector3 rot_vec){
    tf::Vector3 y_axis(0,1,0);
    return 0.0;
}

#endif