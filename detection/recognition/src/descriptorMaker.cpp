#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"


int siz=50;
int sts=5;
double stpX;
double stpY;

double[][] createSmaller(int[][] orgF, int orgX, int orgY){
	smlF=new double[siz][siz];
	int ctr[][] = new int[siz][siz];
	double x=0;
	for(int i=0;i<orgX; i++){
		double y=0;
		for(int j=0;j<orgY;j++){
			smlF[(int)x][(int)y]+=orgF[i][j];
			ctr[(int)x][(int)y]++;
			y+=stpY;
		}
		x+=stpX;
	}
	for(int i=0;i<siz; i++)
		for(int j=0;j<siz;j++)
			smlF[i][j]/=(double)(ctr[i][j]+1);
}

void normalize(double[][] smlF){
	double min=256;
	double max=0;
	for(int i=0;i<siz; i++){
		for(int j=0;j<siz;j++){
			smlF[i][j]/=255;
			if(smlF[i][j]<min)min=smlF[i][j];
			if(smlF[i][j]>max)max=smlF[i][j];
		}
	}
	double diff=(max-min);
	for(int i=0;i<siz; i++){
		for(int j=0;j<siz;j++){
			smlF[i][j]=((smlF[i][j]-min)/diff);
		}
	}
}

std_msgs::Float64MultiArray getDescriptorVector(double[][] smlF){
	std_msgs::Float64MultiArray array;
	array.data.clear();
	int stp=siz/sts;
	for(int i=0;i<sts; i++){
		for(int j=0;j<sts;j++){
			double avg=0;
			double dx=0;
			double dy=0;
			for(int k=i*stp+1;k<(i+1)*stp;k++){
				for(int l=j*stp+1;l<(j+1)*stp;l++){
					avg+=smlF[k][l];
					dx+=smlF[k][l]-smlF[k-1][l];
					dy+=smlF[k][l]-smlF[k][l-1];
				}
			}
			avg/=((stp-1)*(stp-1));
			dx/=((stp-1)*(stp-1));
			dy/=((stp-1)*(stp-1));
			array.data.push_back(avg);
			array.data.push_back(dx);
			array.data.push_back(dy);
			
		}
	}
	return array;
}


ros::Publisher pub;
void callback(const Image face) {
	double[][] smlF=createSmaller(face);
	normalize(smlF);
	std_msgs::Float64MultiArray desc=getDescriptorVector(smlF);

	pub.publish(desc);
}


int main (int argc, char** argv) {

	// Initialize ROS

	ros::init (argc, argv, "descriptorMaker");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("faceImg", 1, callback);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<std_msgs::Float64MultiArray> ("faceDesc", 1);


	ros::Rate r(5);
	// Spin
	ros::spin ();


	}