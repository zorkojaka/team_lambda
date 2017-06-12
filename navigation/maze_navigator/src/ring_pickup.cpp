#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define mp make_pair
#define pb push_back

using namespace std;
using namespace cv;


/*
    Small correction relative to robot.
*/
void constructGoal(MoveBaseClient &ac, tf::Vector3& trans, double angle_2d){
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/base_link";
    goal.target_pose.header.stamp = ros::Time::now();
  
    ROS_INFO("Moving ROBOT -> (%f %f) to (%f %f)\n",r_pos.wy_, r_pos.wx_, r_goal.wy_, r_goal.wx_);

    // translation
    goal.target_pose.pose.position.x = trans.x();
    goal.target_pose.pose.position.y = trans.y();
    goal.target_pose.pose.position.z = trans.z();

    // rotation
    tf::Quaternion rot_quat = constructRotation(angle_2d);
    goal.target_pose.pose.orientation.x = rot_quat_.x();
    goal.target_pose.pose.orientation.y = rot_quat_.y();
    goal.target_pose.pose.orientation.z = rot_quat_.z();
    goal.target_pose.pose.orientation.w = rot_quat_.w();
    
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ring_pickup");

    // initialize handlers
    ros::NodeHandle n;

    // subscribers
    ros::Subscriber costmap_sub;
    ros::Subscriber simplemap_sub;

    // publishers
    ros::Publisher goal_pub;
    ros::Publisher vis_pub;

     // frame transformer
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
    // vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // goal sender
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    RobotPose r_goal; 
    RobotPose r_start;


    // main loop
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        waitKey(100);
        ros::spinOnce();
    }
    return 0;
}
