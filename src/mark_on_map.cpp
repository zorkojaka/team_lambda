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


ros::Publisher marker_pub;
ros::Subscriber red_sub;
ros::Subscriber grn_sub;
ros::Subscriber blu_sub;

bool redFound=false;
bool grnFound=false;
bool bluFound=false;

void redCallback(const visualization_msgs::Marker marker){
	if(redFound==true) return;
	redFound=true;
	printf("RED FOUND");
	marker_pub.publish(marker);
}
void grnCallback(const visualization_msgs::Marker marker){
	if(grnFound==true) return;
	bluFound=true;
	printf("RED FOUND");
	marker_pub.publish(marker);
}
void bluCallback(const visualization_msgs::Marker marker){
	if(bluFound==true) return;
	bluFound=true;
	printf("RED FOUND");
	marker_pub.publish(marker);
}

int main(int argc, char** argv) {
	
	ros::init(argc, argv, "mark_on_map");
    	ros::NodeHandle n;
    
    	red_sub = n.subscribe("/exercise6/find_red", 1, redCallback);
    	grn_sub = n.subscribe("/exercise6/find_grn", 1, grnCallback);
    	blu_sub = n.subscribe("/exercise6/find_blu", 1, bluCallback);
	marker_pub = n.advertise<visualization_msgs::Marker> ("points", 1);
    
    
	while(ros::ok()) {
		ros::spinOnce();

	}

    return 0;
 }
