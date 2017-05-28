#include "ros/ros.h"

#include <stdlib.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>


using namespace std;
using namespace cv;

void searchForCircles();
Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;
ros::Subscriber red_sub;
ros::Subscriber grn_sub;
ros::Subscriber blu_sub;
int sizeY;

int minX[6]={105,134,163,190,160,134};
int maxX[6]={113,143,169,194,172,142};
int minY[6]={112,122,122,100,90,92};
int maxY[6]={123,124,124,120,100,98};

bool canDetect=false;	

bool redDetected=false;
bool bluDetected=false;
bool grnDetected=false;

float zDir[6]={-0.684288290471, 0.0372137819999, 0.757220047624, 0.915187144694, 0.996540233763, -0.861617006523};
float wDir[6]={0.72921158488, 0.999307327317, 0.653159857521, 0.403029143098, -0.0831117470074, 0.507558995655};



void redCallback(const visualization_msgs::Marker marker){
	if(canDetect == false){
		return;
	}
	if(redDetected == true)
	 	return;
	redDetected=true;
}
void grnCallback(const visualization_msgs::Marker marker){
	if(canDetect == false){
		return;
	}
	if(grnDetected == true)
	 	return;
	grnDetected=true;
}
void bluCallback(const visualization_msgs::Marker marker){
	if(canDetect == false){
		return;
	}
	if(grnDetected == true)
	 	return;
	bluDetected=true;
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg_map) {
    int size_x = msg_map->info.width;
    int size_y = msg_map->info.height;
	sizeY=size_y;
	printf("Veselo nastavljeno!\n");
    if ((size_x < 3) || (size_y < 3) ) {
        ROS_INFO("Map size is only x: %d,  y: %d . Not running map to image conversion", size_x, size_y);
        return;
    }

    // resize cv image if it doesn't have the same dimensions as the map
    if ( (cv_map.rows != size_y) && (cv_map.cols != size_x)) {
        cv_map = cv::Mat(size_y, size_x, CV_8U);
    }

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

int main(int argc, char** argv) {

    ros::init(argc, argv, "map_goalz");
    ros::NodeHandle n;
	
	
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    map_sub = n.subscribe("/map", 10, &mapCallback);
    red_sub = n.subscribe("/exercise6/find_red", 10, &redCallback);
    grn_sub = n.subscribe("/exercise6/find_grn", 10, &grnCallback);
    blu_sub = n.subscribe("/exercise6/find_blu", 10, &bluCallback);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("goal", 10);
	
	
    while(ros::ok()) {
    
		
		//CIS RANDOM
		int x,y;
    
		
		int minx, maxx, miny, maxy;
		//x 100 +25
		//y 90 +15 *3
		
		int i=0;
		for(i=0;i<6;i++){
	
			minx=minX[i];
			maxx=maxX[i];
			miny=minY[i];
			maxy=maxY[i];
		
			//min + (rand() * (int)(max-min) / RAND_MAX)
				x= (rand()%(maxx-minx)+minx);
				y= (rand()%(maxy-miny)+miny);
			if(cv_map.empty()){
				i--;
				printf("Bappitty boopitty! no mapy mapson!\n");
				ros::spinOnce();
				sleep(1);
				continue;
			}
			printf("Map found!\n");
			int v = (int)cv_map.at<unsigned char>(y,x);
			int idxTry=0;
			while (v != 255 && idxTry<100) {
				ROS_WARN("Unable to move to i:%d (x: %d, y: %d), not reachable", i, x, y);
				x= (rand()%(maxx-minx)+minx);
				y= (rand()%(maxy-miny)+miny);
				
				v = (int)cv_map.at<unsigned char>(y,x);
				ros::spinOnce();
				idxTry++;
				
			}
			printf("X:%d,	Y:%d!\n",x,y);sleep(1);
		
			
			tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
			tf::Point transformed = map_transform * pt;
	
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.pose.orientation.z = zDir[0];
			goal.target_pose.pose.orientation.w = wDir[0];
			goal.target_pose.pose.position.x = transformed.x();
			goal.target_pose.pose.position.y = transformed.y();
			goal.target_pose.header.stamp = ros::Time::now();

			ac.sendGoal(goal);

			ac.waitForResult();
			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				//na cilju!
				canDetect=true;
				ROS_INFO("Started detecting");
				int rotator=1;
				while(rotator<6){
					move_base_msgs::MoveBaseGoal goal1;
					goal1.target_pose.header.frame_id = "map";
					goal1.target_pose.pose.orientation.z = zDir[rotator];
					goal1.target_pose.pose.orientation.w = wDir[rotator];
					goal1.target_pose.pose.position.x = transformed.x();
					goal1.target_pose.pose.position.y = transformed.y();
					goal1.target_pose.header.stamp = ros::Time::now();
					ac.sendGoal(goal1);

					ac.waitForResult();
					rotator++;
					ros::spinOnce();
				}
				
				ROS_INFO("Ended detecting");
				canDetect=false;
				
				searchForCircles();
				ROS_INFO("Hooray, the base moved!!!");
			}
			else
				ROS_INFO("The base failed to move :(");

			ros::spinOnce();
		}
    }
    return 0;

}

void searchForCircles(){
	if(redDetected){
		ROS_INFO("RED detected in location!!!");
	}
	
	if(grnDetected){
		ROS_INFO("GREEN detected in location!!!");
	}
	
	if(bluDetected){
		ROS_INFO("BLUE detected in location!!!");
	}
}
