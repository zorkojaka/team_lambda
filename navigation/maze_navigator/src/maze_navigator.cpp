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
#include <maze_navigator/expl_planner.h>
#include <maze_navigator/costmap.h>
#include <maze_navigator/visualizer.h>

#define mp make_pair
#define pb push_back

using namespace std;
using namespace cv;

Costmap costmap;          // maze costmap
ExplPlanner expl_planner; // planner for autonomous exploration
RobotPose robot_pose;     // current robot position
MObject maze_objects;     // all detected objects in the maze

bool goal_in_progress;

// layer paths
char *simple_layer_path;
char *north_south_layer_path;
char *south_north_layer_path;
char *east_west_layer_path;
char *west_east_layer_path;

// logic



// map callbacks
void simplemapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    ROS_INFO("COSTMAP SIMPLE MAP");
    costmap.storeSimple(msg_map);
}
void costmapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    ROS_INFO("COSTMAP COSTMAP REVEICEV");
    costmap.store(msg_map);
}

/*  
    Sends goal to move_base
*/
void sendGoal(MoveBaseClient &ac, RobotPose &r_goal)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    //ROS_INFO("Moving ROBOT -> (%f %f) to (%f %f)\n", r_pos.wy_, r_pos.wx_, r_goal.wy_, r_goal.wx_);

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

    goal_in_progress = true;
}

/*
    Checks state of currently executing goal.
*/
void checkGoalState(MoveBaseClient &ac)
{
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal reached (%f, %f, %f) [%f, %f, %f, %f]\n",
                 robot_pose.wx_,
                 robot_pose.wy_,
                 robot_pose.wz_,
                 robot_pose.rot_quat_.x(),
                 robot_pose.rot_quat_.y(),
                 robot_pose.rot_quat_.z(),
                 robot_pose.rot_quat_.w());

        expl_planner.goalReachedCb(robot_pose, costmap);
        goal_in_progress = false;
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
                     ros::Time &now, tf::StampedTransform &transform)
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
    return true;
}

/*
    Reads individual layer from file and adds it.
*/
bool loadAndAddLayer(char *layer_path, int layer_flag)
{
    Mat image;
    image = imread(layer_path, IMREAD_UNCHANGED); // Read the file
    if (!image.data)                              // Check for invalid input
    {
        cout << "Could not open or find the image" << std::endl;
        return false;
    }

    vector<vector<int>> l_mat;
    for (int y = 0; y < image.rows; y++)
    {
        l_mat.pb(vector<int>());
        for (int x = 0; x < image.cols; x++)
        {
            int val = image.data[(image.rows - y - 1) * image.cols + x];
            l_mat[y].pb((val == 0 ? 1 : 0));
        }
    }
    expl_planner.addLayer(l_mat, l_mat.size(), l_mat[0].size(), layer_flag);
    return true;
}

/*
    Loads map layers - target cells, etc..
    @returns true if success
*/
bool loadExplPlannerLayers()
{
    ROS_INFO("Adding simple layer...");
    loadAndAddLayer(simple_layer_path, LAYER_SIMPLE);
    /*loadAndAddLayer(north_south_layer_path, LAYER_NORTH_SOUTH);
    loadAndAddLayer(south_north_layer_path, LAYER_SOUTH_NORTH);
    loadAndAddLayer(east_west_layer_path, LAYER_EAST_WEST);
    loadAndAddLayer(west_east_layer_path, LAYER_WEST_EAST);*/
    return true;
}

/*
    Prepare costmap and maze costmap.
    @returns true if ready
*/
bool plannerAndCostmapReady()
{
    if (!expl_planner.init_)
    {
        if (costmap.init_)
        {
            expl_planner.init(robot_pose, costmap);
            loadExplPlannerLayers();
        }
        else
            costmap.layerMaps();
        return false;
    }
    else
        return true;
}

bool readArgs(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR(" Usage: maze_navigator simple_layer_path north_south_layer_path ... \n");
        return false;
    }

    simple_layer_path = argv[1];
    /*north_south_layer_path = argv[2];
    south_north_layer_path = argv[3];
    east_west_layer_path = argv[4];
    west_east_layer_path = argv[5];*/
    return true;
}

/*
    Main planning node.

    args1: target_layer pgm file
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "maze_navigator");
    if (!readArgs(argc, argv))
        return -1;

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

    goal_in_progress = false;

    // main loop
    ROS_INFO("Starting main loop...");
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        if (!updateRobotPose(listener, now, transform))
        {
            waitKey(100);
            ros::spinOnce();
            continue;
        }
        if (!plannerAndCostmapReady())
        {
            waitKey(100);
            ros::spinOnce();
            continue;
        }

        if (goal_in_progress)
        {
            checkGoalState(ac);
        }
        else
        {
            bool goal_found = false;
            r_goal = expl_planner.getNextGoal(robot_pose, costmap, goal_found);

            visualizeSinglePoint(vis_pub, r_goal);

            if (!goal_found)
            {
                break;
            }
            sendGoal(ac, r_goal);
        }

        visualizeMultipleLayers(vis_pub, expl_planner.expl_map_, costmap);

        waitKey(100);
        ros::spinOnce();
    }
    return 0;
}
