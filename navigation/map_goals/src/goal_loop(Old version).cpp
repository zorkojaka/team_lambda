#include "ros/ros.h"

#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <time.h>

using namespace std;

ros::Publisher goal_pub;
ros::Publisher marker_pub;
ros::Subscriber map_sub;
ros::Subscriber red_sub;
ros::Subscriber grn_sub;
ros::Subscriber blu_sub;

float map_resolution = 0;
tf::Transform map_transform;
visualization_msgs::Marker redMarker;
bool redFound=false;


Mat cv_map;
int size_x;
int size_y;


void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;

    map_resolution = msg_map->info.resolution;
	tf::poseMsgToTF(msg_map->info.origin, map_transform);
	
	
	const std::vector<int8_t>& map_msg_data (msg_map->data);

    unsigned char *cv_map_data = (unsigned char*) cv_map.data;

    //We have to flip around the y axis, y for image starts at the top and y for map at the bottom
    int size_y_rev = size_y-1;

    for (int y = size_y_rev; y >= 0; --y) {

        int idx_map_y = size_x * (size_y -y);
        int idx_img_y = size_x * y;

        for (int x = 0; x < size_x; ++x) {

            int idx = idx_img_y + x;

            switch (map_msg_data[idx_map_y + x])
            {
            case -1:
                cv_map_data[idx] = 127;
                break;

            case 0:
                cv_map_data[idx] = 255;
                break;

            case 100:
                cv_map_data[idx] = 0;
                break;
            }
        }
    }
}

int getMapAt(int x, int y){
	return cv_map_data[size_x*y+x];
}


void redCallback(const visualization_msgs::Marker marker){
	if(redFound==true) return;
	redMarker=marker;
	redFound=true;
}




int main(int argc, char** argv) {
	
	ros::init(argc, argv, "goal_loop");
    ros::NodeHandle n;
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
	 
	 
		 
	tf::TransformListener map2BaseLinkListner;
	tf::StampedTransform map2BaseLink;
	
	map2BaseLinkListner.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0));
	map2BaseLinkListner.lookupTransform("/map", "/base_link", ros::Time(0), map2BaseLink);
	
	marker_pub = n.advertise<visualization_msgs::Marker> ("points", 1);
	
	
	
	
	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
    map_sub = n.subscribe("map", 10, &mapCallback);
    //faceDetect_sub = n.subscribe("detectionFace", 10, &faceDetector);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);

	
    while(ros::ok()) {
	int x=0,y=100;
	int faceCounter=0;
		//
		do{
    		x=rand()%size_x;
    		y=rand()%size_y;
    	}while(getValAt(x,y)==0);
		tf::Point pt((float)x * 0.05, (float)y * 0.05, 0.0);
		tf::Point transformed = map_transform * pt;
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "map";
		goal.target_pose.pose.orientation.w = 1;
		goal.target_pose.pose.position.x = transformed.x();
		goal.target_pose.pose.position.y = transformed.y();
		goal.target_pose.header.stamp = ros::Time::now();


  		ROS_INFO("Sending goal location ...");
   		ac.sendGoal(goal);

		ac.waitForResult();
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Hooray, the base moved!!!");
			
		if(redFound){
			//publish red marker and goal underneath
			printf("RED found\n");
			
			
			

			tf::Stamped<tf::Pose> transformed_red_marker;
			map2BaseLinkListner.transformPose("map", redMarker.pose, transformed_red_marker);

			
			marker_pub.publish(redMarker);
		}


		ros::spinOnce();

	}
	
    return 0;

}
