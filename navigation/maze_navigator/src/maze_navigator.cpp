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

#define mp make_pair
#define pb push_back

using namespace std;
using namespace cv;

// map callbacks
void simplemapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    costmap.storeSimpleMap(msg_map);
}
void costmapCallback(const nav_msgs::OccupancyGridConstPtr &msg_map)
{
    costmap.storeCostMap(msg_map);
}

// Planner
const int VIS = 1;
const int ROBOT_REACHABLE = 2;
const int COND_REACHABLE = 4;
const int TARGET_CELL = 8;
const int ENLIGHT = 16;
const int TARGET_REACHED = 32;
const int PLANNER_CALC = 64;

const int VISIBLE_TARGET_CELL = 128;


const double VIEW_ANGLE = 0.5;
const int ANGLE_PARTS = 50;
const double VIEW_RANGE = 0.7;

const int ROBOT_ANGLE_PARTS = 4;

const int PLANNER_NEW_POINTS = 2;

struct FoV
{
    double wy_;
    double wx_;
    vector<tf::Vector3> vecs_;
    vector<double> mag_;
    vector<pair<int, int> > pts_;
};

FoV calcFieldOfView(const RobotPose &r_pos, CostMap &costmap)
{
    tf::Vector3 z_axis(0, 0, 1); // z-axis rotation
    FoV field_of_view;

    field_of_view.wx_ = r_pos.wx_;
    field_of_view.wy_ = r_pos.wy_;

    double step = 2.0 * VIEW_ANGLE / ANGLE_PARTS;

    set< pair<int,int> > pts_unique;
    for (double rot = -VIEW_ANGLE; rot <= VIEW_ANGLE; rot += step)
    {
        tf::Vector3 curr_rot = r_pos.rot_.rotate(z_axis, rot);

        field_of_view.vecs_.pb(curr_rot);
        field_of_view.mag_.pb(VIEW_RANGE);

        // naive endpoint
        int py, px;
        for (double mult = 0.01; mult <= VIEW_RANGE; mult += 0.01)
        {
            costmap.fromWorld(r_pos.wy_ + curr_rot.y() * mult, r_pos.wx_ + curr_rot.x() * mult, py, px);
            if (costmap.isObstacle(py, px))
            {
                field_of_view.mag_[field_of_view.mag_.size() - 1] = mult;
                if(pts_unique.find(mp(py, px)) == pts_unique.end()){
                    field_of_view.pts_.pb(mp(py, px));
                    pts_unique.insert(mp(py, px));
                }
                break;
            }
        }
    }
    return field_of_view;
}



struct TargetPosition{
    int py_, px_;
    double angle_;
    vector<int> ccomp_;
    vector<pair<int,int> > pts_;
};





visualization_msgs::Marker createPointCloud(int id, float r, float g, float b)
{
    visualization_msgs::Marker point_cloud;

    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.frame_id = "/map";
    point_cloud.id = id;

    point_cloud.type = visualization_msgs::Marker::POINTS;
    point_cloud.action = visualization_msgs::Marker::ADD;

    point_cloud.scale.x = 0.02; // point width
    point_cloud.scale.y = 0.02; // point height

    // violet?
    point_cloud.color.r = r;
    point_cloud.color.g = g;
    point_cloud.color.b = b;

    point_cloud.color.a = 1.0;

    return point_cloud;
}

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

/*
    Visualize costmap layer.
*/
void visualizeLayer(const ros::Publisher &vis_pub, const Planner &planner, int layer_flag,
                    visualization_msgs::Marker point_cloud)
{
    for (int y = 0; y < costmap.height_; y++)
    {
        for (int x = 0; x < costmap.width_; x++)
        {
            if (planner.aux_map_[y][x] & layer_flag)
            {
                double wx, wy;
                costmap.fromPixel(y, x, wy, wx);

                geometry_msgs::Point p;
                p.x = wx;
                p.y = wy;
                p.z = 0;
                point_cloud.points.push_back(p);
            }
        }
    }
    vis_pub.publish(point_cloud);
}

/*
    Visualize given field of view.
*/
void visualizeFoV(const ros::Publisher &vis_pub, FoV &fov)
{
    // visualize view rays
    visualization_msgs::Marker line_list;
    line_list.header.stamp = ros::Time::now();
    line_list.header.frame_id = "/map";
    line_list.id = 1;

    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.scale.x = 0.005;

    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    for (int i = 0; i < fov.vecs_.size(); i++)
    {
        line_list.points.push_back(createPoint(fov.wx_, fov.wy_, 0.0));
        line_list.points.push_back(createPoint(fov.wx_ + fov.vecs_[i].x() * fov.mag_[i],
                                               fov.wy_ + fov.vecs_[i].y() * fov.mag_[i], 0.0));
    }
    vis_pub.publish(line_list);
    

    // visualize "seen" points
    visualization_msgs::Marker point_cloud = createPointCloud(7, 1.0, 1.0, 0.0);
    for (int i = 0; i < fov.pts_.size(); i++)
    {
        double wx, wy;
        costmap.fromPixel(fov.pts_[i].first, fov.pts_[i].second, wy, wx);
        point_cloud.points.push_back(createPoint(wx, wy, 0.0));
    }
    vis_pub.publish(point_cloud);
}

void debug(const ros::Publisher &vis_pub,
           visualization_msgs::Marker point_cloud)
{
    for (int y = 0; y < costmap.height_; y++)
    {
        for (int x = 0; x < costmap.width_; x++)
        {
            if (costmap.simple_map_[y][x] == 0)
            {
                double wx, wy;
                costmap.fromPixel(y, x, wy, wx);

                geometry_msgs::Point p;
                p.x = wx;
                p.y = wy;
                p.z = 0;
                point_cloud.points.push_back(p);
            }
        }
    }
    vis_pub.publish(point_cloud);
}

void visualizePlanner(const ros::Publisher &vis_pub, const Planner &planner)
{
    //debug(vis_pub, createPointCloud(2, 0, 1, 0));
    //visualizeLayer(vis_pub, planner, COND_REACHABLE, createPointCloud(1, 0.5, 0, 0.5));
    visualizeLayer(vis_pub, planner, ROBOT_REACHABLE, createPointCloud(2, 0, 1, 0));
    visualizeLayer(vis_pub, planner, TARGET_CELL, createPointCloud(3, 1.0, 0, 0));
    //   visualizeLayer(vis_pub, planner, ENLIGHT, createPointCloud(4, 0.5, 0.8, 0.0));
   // visualizeLayer(vis_pub, planner, TARGET_REACHED, createPointCloud(4, 0.5, 0.8, 0.0));
    // visualizeFoV(vis_pub, planner);
}

void visualizeSinglePoint(const ros::Publisher &vis_pub, RobotPose& r_pos){
    visualization_msgs::Marker point_cloud = createPointCloud(11, 1.0, 0.0, 0.0);
    point_cloud.scale.x = 0.1; // point width
    point_cloud.scale.y = 0.1; // point height

    geometry_msgs::Point p;
    p.x = r_pos.wx_;
    p.y = r_pos.wy_;
    p.z = 0;
    point_cloud.points.push_back(p);
    
    vis_pub.publish(point_cloud);
}


visualization_msgs::Marker createArrow(int id, double r, double g, double b){
    visualization_msgs::Marker arrow;
    arrow.header.stamp = ros::Time::now();
    arrow.header.frame_id = "/map";
    arrow.id = id;

    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.02;

    arrow.color.r = r;
    arrow.color.g = g;
    arrow.color.b = b;
    
    arrow.color.a = 1.0;

    return arrow;
}

float rgb[4][3] = {{1,0,0}, {0,1,0}, {0,0,1}, {1,1,1}};
    
void visualizeRobotOrientation(const ros::Publisher &vis_pub, RobotPose& r_pos, RobotPose& r_goal){
    ROS_INFO("VISUALIZE ROBOT ORIENTATION");
    int arrow_cnt = 0;
    visualization_msgs::Marker arrow;
    for (double angle = 0.0; angle < 2 * M_PI && arrow_cnt < 4; angle += (2 * M_PI) / 4.0)
    {
        arrow = createArrow(arrow_cnt, rgb[arrow_cnt][0],
            rgb[arrow_cnt][1],rgb[arrow_cnt][2]);
        arrow.pose.position = createPoint(r_pos.wx_, r_pos.wy_, 0.0);

        tf::Quaternion quat = constructRotation(angle);
        arrow.pose.orientation.x = quat.x();
        arrow.pose.orientation.y = quat.y();
        arrow.pose.orientation.z = quat.z();
        arrow.pose.orientation.w = quat.w();


        vis_pub.publish(arrow);

        arrow_cnt++;
    }

    arrow = createArrow(arrow_cnt, 0.7,
            0.0, 0.4);
    arrow.pose.position = createPoint(r_goal.wx_, r_goal.wy_, 0.0);

    arrow.pose.orientation.x = r_goal.rot_quat_.x();
    arrow.pose.orientation.y = r_goal.rot_quat_.y();
    arrow.pose.orientation.z = r_goal.rot_quat_.z();
    arrow.pose.orientation.w = r_goal.rot_quat_.w();
    vis_pub.publish(arrow);
    arrow_cnt++;


    arrow = createArrow(arrow_cnt, 0.0,
            0.0, 0.0);
    arrow.pose.position = createPoint(r_pos.wx_, r_pos.wy_, 0.0);

    arrow.pose.orientation.x = r_pos.rot_quat_.x();
    arrow.pose.orientation.y = r_pos.rot_quat_.y();
    arrow.pose.orientation.z = r_pos.rot_quat_.z();
    arrow.pose.orientation.w = r_pos.rot_quat_.w();
    vis_pub.publish(arrow);
    arrow_cnt++;
}

bool goal_sent = false;
void sendGoal(MoveBaseClient &ac, RobotPose &r_goal, RobotPose &r_pos, CostMap &costmap)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
  
    ROS_INFO("Moving ROBOT -> (%f %f) to (%f %f)\n",r_pos.wy_, r_pos.wx_, r_goal.wy_, r_goal.wx_);

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
    else{
     //   ROS_INFO("GOAL IN PROGRESS...");
    }
}

// path planner
Planner planner;
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

    // current robot position
    RobotPose r_pos;

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

    int vis_counter = 0;
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();

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
            continue;
        }

        r_pos.update(transform, costmap);
        if(!r_start.init_){
            r_start = r_pos;
        }

        // initialize planner and costmap
        if (!planner.init_)
        {
            if (costmap.init_)
                planner.init(r_pos, costmap);
            else
                costmap.layerMaps();
        }
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

        ROS_INFO("VISUALIZING PLANNER");
        visualizePlanner(vis_pub, planner);
        vis_counter++;
        waitKey(100);
        ros::spinOnce();
    }
    return 0;
}
