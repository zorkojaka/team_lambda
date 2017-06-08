#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h> 

std::vector<std_msgs::Float64MultiArray> faces (0);
bool knownFace(std_msgs::Float64MultiArray newF){
	int len = newF.data.size();
	double minDist = 300000;
	int minIdx=-1;
	for(int j=0; j<faces.size();j++){
		double dist =0;
		std_msgs::Float64MultiArray learnedFace =faces.at(j);
		for(int i=0; i<len;i++)
			dist+=(learnedFace.data[i]-newF.data[i])*(learnedFace.data[i]-newF.data[i]);
		dist=sqrt(dist);
		printf("Dists:%f\n",dist);
		if(dist<minDist){
			minIdx=j;
			minDist=dist;
		}
		if(dist<0.6){
			printf("I already know this face!\n\n");
			return true;
		}
	}
	printf("Min distance is between i=%d and this:%f\n",minIdx,minDist);
	if(minDist>1.6){
		printf("Adding new face!\n");
		faces.push_back(newF);
	}
	return false;
}
void callback(const std_msgs::Float64MultiArray::ConstPtr&  descPtr) {
	std_msgs::Float64MultiArray desc = *descPtr;
	knownFace(desc);
}
int main (int argc, char** argv) {

	// Initialize ROS

	ros::init (argc, argv, "distanceCalculator");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("faceDesc", 1, callback);
	ros::Rate r(5);
	// Spin
	ros::spin ();


	}
