#include <iostream>
#include <vector>
#include <deque>
#include <cmath>
#include <set>

#include "ros/ros.h"

#include <nav_msgs/GetMap.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

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

// project includes
#include <maze_navigator/robot_pose.h>
#include <maze_navigator/exploration_planner.h>
#include <maze_navigator/costmap.h>

#define mp make_pair
#define pb push_back

using namespace std;
using namespace cv;

Costmap costmap;          // maze costmap
ExplPlanner expl_planner; // planner for autonomous exploration
RobotPose robot_pose;     // current robot position

// map callbacks
void simplemapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    costmap.storeSimpleMap(msg_map);
}
void costmapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    costmap.storeCostMap(msg_map);
}

struct TargetPosition
{
    int py_, px_;
    double angle_;
    vector<int> ccomp_;
    vector<pair<int, int>> pts_;
};

/*
    Action client interaction.
*/

bool goal_sent = false;
void sendGoal(MoveBaseClient &ac, RobotPose &r_goal, RobotPose &r_pos, CostMap &costmap)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    ROS_INFO("Moving ROBOT -> (%f %f) to (%f %f)\n", r_pos.wy_, r_pos.wx_, r_goal.wy_, r_goal.wx_);

    // translation
    goal.target_pose.pose.position.x = r_goal.wx_;
    goal.target_pose.pose.position.y = r_goal.wy_;
    goal.target_pose.pose.position.z = r_goal.wz_;

    goal.target_pose.pose.orientation.x = r_goal.rot_quat_.x();
    goal.target_pose.pose.orientation.y = r_goal.rot_quat_.y();
    goal.target_pose.pose.orientation.z = r_goal.rot_quat_.z();
    goal.target_pose.pose.orientation.w = r_goal.rot_quat_.w();

    ROS_INFO("Sending goal (%f, %f, %f) [%f, %f, %f, %f]\n",
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y,
             goal.target_pose.pose.position.z,
             goal.target_pose.pose.orientation.x,
             goal.target_pose.pose.orientation.y,
             goal.target_pose.pose.orientation.z,
             goal.target_pose.pose.orientation.w);

    ac.sendGoal(goal);

    goal_sent = true;
}

void checkGoal(MoveBaseClient &ac, Planner &planner, const RobotPose &r_pos)
{
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached (%f, %f, %f) [%f, %f, %f, %f]\n",
                 r_pos.wx_,
                 r_pos.wy_,
                 r_pos.wz_,
                 r_pos.rot_quat_.x(),
                 r_pos.rot_quat_.y(),
                 r_pos.rot_quat_.z(),
                 r_pos.rot_quat_.w());

        planner.goalReached(r_pos, costmap);
        goal_sent = false;
    }
    else
    {
        //   ROS_INFO("GOAL IN PROGRESS...");
    }
}

/*
    Updates current robot position.
    @returns True if success
*/
bool updateRobotPose(tf::TransformListener &listener,
                     ros::Time &now)
{
    try
    {
        listener.waitForTransform("/map", "/base_link",
                                  now, ros::Duration(2.0));
        listener.lookupTransform("/map", "/base_link",
                                 ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }

    robot_pose.update(transform, costmap);
}


/*
    Loads map layers - target cells, etc..
    @returns true if success
*/
bool loadExplPlannerLayers(){
    

}

/*
    Prepare costmap and maze costmap.
    @returns true if ready
*/
bool plannerAndCostmapReady()
{
    if (!expl_planner.init_)
    {
        if (costmap.init_){
            loadExplPlannerLayers();
            expl_planner.init(robot_pose, costmap);
        }
        else
            costmap.layerMaps();
    }
    else return true;    
}


/*
    Main planning node.
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_navigator");

    // initialize handlers
    ros::NodeHandle n;

    // subscribers
    ros::Subscriber costmap_sub;
    ros::Subscriber simplemap_sub;

    // publishers
    ros::Publisher goal_pub;
    ros::Publisher vis_pub;

    // maze map
    costmap_sub = n.subscribe("/move_base/global_costmap/costmap", 10, &costmapCallback);
    simplemap_sub = n.subscribe("/map", 10, &simplemapCallback);

    // frame transformer
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
    // vis_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
    // vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
        if (!updateRobotPose(listener, now))
            continue;
        if (!plannerAndCostmapReady())
            continue;

        /* else
        {
            if (goal_sent)
            {
                checkGoal(ac, planner, r_pos);
            }
            else
            {
                bool goal_found = false;
                r_goal = planner.getNextGoal(r_pos, costmap, goal_found);
                visualizeSinglePoint(vis_pub, r_goal);

                if(!goal_found){
                    r_goal = r_start;
                    ROS_INFO("RETURNING BACK TO STARTING POINT\n");
                }

                sendGoal(ac, r_goal, r_pos, costmap);
            }
        }

        if(goal_sent && vis_counter == 20){
            FoV fov = calcFieldOfView(r_goal,costmap);
     //       visualizeFoV(vis_pub, fov);
            visualizePlanner(vis_pub, planner);
     
       //     visualizeRobotOrientation(vis_pub, r_pos, r_goal);
            vis_counter = 0;
        }
        */

//        ROS_INFO("VISUALIZING PLANNER");
//        visualizePlanner(vis_pub, planner);
//        vis_counter++;
        waitKey(100);
        ros::spinOnce();
    }
    return 0;
}
