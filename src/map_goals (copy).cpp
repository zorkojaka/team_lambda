#include "ros/ros.h"

#include <stdlib.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

Mat cv_map;
float map_resolution = 0;
tf::Transform map_transform;

ros::Publisher goal_pub;
ros::Subscriber map_sub;
int sizeY;

int minX[6]={105,134,163,190,162,134};
int maxX[6]={113,143,169,195,168,142};
int minY[6]={112,122,122,102,95,92};
int maxY[6]={123,124,124,118,97,98};
		

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

    map_sub = n.subscribe("/map", 10, &mapCallback);
	goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	
	
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
		
			printf("X:%d,	Y:%d!\n",x,y);sleep(1);
			while (v != 255) {
				//ROS_WARN("Unable to move to i:%d (x: %d, y: %d), not reachable", i, x, y);
				x= (rand()%(maxx-minx)+minx);
				y= (rand()%(maxy-miny)+miny);
				ros::spinOnce();
				
			}
			//konc random
		
			
			tf::Point pt((float)x * map_resolution, (float)y * map_resolution, 0.0);
			tf::Point transformed = map_transform * pt;
	
			geometry_msgs::PoseStamped goal;
			goal.header.frame_id = "/map_goal";
			goal.pose.orientation.w = 0.0;
			goal.pose.orientation.z = 0.0;
			goal.pose.position.x = transformed.x();
			goal.pose.position.y = transformed.y();
			goal.header.stamp = ros::Time::now();

			goal_pub.publish(goal);

			ros::spinOnce();
		}
    }
    return 0;

}
