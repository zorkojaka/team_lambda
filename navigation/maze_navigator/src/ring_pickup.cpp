
#include "ros/ros.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Action goals
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <maze_navigator/geom_util.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

const double OFFSET_X = 0.0;
const double OFFSET_Y = 0.0;
const double OFFSET_Z = 0.0;
/*
    Small correction relative to robot.
*/
void constructGoal(MoveBaseClient &ac, tf::Vector3 &trans, double angle_2d)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/base_link";
    goal.target_pose.header.stamp = ros::Time::now();

   
    // translation
    goal.target_pose.pose.position.x = trans.x();
    goal.target_pose.pose.position.y = trans.y();
    goal.target_pose.pose.position.z = trans.z();

    // rotation
    tf::Quaternion rot_quat = constructRotation(angle_2d);
    goal.target_pose.pose.orientation.x = rot_quat.x();
    goal.target_pose.pose.orientation.y = rot_quat.y();
    goal.target_pose.pose.orientation.z = rot_quat.z();
    goal.target_pose.pose.orientation.w = rot_quat.w();

    ROS_INFO("Sending goal (%f, %f, %f) [%f, %f, %f, %f]\n",
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y,
             goal.target_pose.pose.position.z,
             goal.target_pose.pose.orientation.x,
             goal.target_pose.pose.orientation.y,
             goal.target_pose.pose.orientation.z,
             goal.target_pose.pose.orientation.w);

    ac.sendGoal(goal);
}

/*
    Get ring position
*/
void ringPoseCallback(const geometry_msgs::PointStamped::ConstPtr& ring_pose)
{
    ROS_INFO("RING DETECTED %f %f %f\n", ring_pose->point.x, ring_pose->point.y, ring_pose->point.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ring_pickup");

    ROS_INFO("HELLO WORLD!\n");
    // initialize handlers
    ros::NodeHandle n;

    // subscribers
    ros::Subscriber ringpose_sub;

    // publishers
    ros::Publisher vis_pub;

    // frame transformer
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ringpose_sub = n.subscribe("input", 1, ringPoseCallback);
    vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // main loop
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        ros::spinOnce();
    }
    return 0;
}
