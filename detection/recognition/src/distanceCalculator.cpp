#include <ros/ros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <math.h> 

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

std::vector<std_msgs::Float64MultiArray> faces[12];
std::vector<string> names; 
string descriptorLocation = "/home/team_lambda/ROS/src/team_lambda/detection/recognition/learnedFaces/";
void writeObject(int n);
bool readFaces(int n);
double minDists[2];
int learningIdx=-1;
void addDist(double thisDist){
	if(thisDist>minDists[1])
		return;
	else if(thisDist>minDists[0]){
		minDists[1]=thisDist;
	}
	else{
		minDists[1]=minDists[0];
		minDists[0]=thisDist;
	}
}
double getAvgMinDist(){
	int numDists=0;
	double avgDist=0;
	for(int i=0;i<3;i++){
		if(minDists[i]!=300000){
			numDists++;
			avgDist+=minDists[i];
		}
		else
			break;
	}
	if(numDists==0)return 300000;
	return avgDist/(double)numDists;
}
double faceSimilarity(std::vector<std_msgs::Float64MultiArray> face, std_msgs::Float64MultiArray newF){
	minDists[0]=300000;
	minDists[1]=300000;
	minDists[2]=300000;
	int minIdx=-1;
	int len = newF.data.size();
	double minDist = 300000;
	for(int j=0; j<face.size();j++){
		double dist =0;
		std_msgs::Float64MultiArray learnedFace =face.at(j);
		for(int i=0; i<len;i++)
			dist+=(learnedFace.data[i]-newF.data[i])*(learnedFace.data[i]-newF.data[i]);
		dist=sqrt(dist);
		addDist(dist);
		if(dist<minDist){
			minIdx=j;
			minDist=dist;
		}
	}
	if(minIdx == -1)
		return -1;
	return getAvgMinDist();
}

void learnFace(std_msgs::Float64MultiArray newF){
	int len = newF.data.size();
	double minDist = 300000;
	int minIdx=-1;
	printf("\n\nStarting matching for this face!\n");
	for(int j=0; j<faces[learningIdx].size();j++){
		double dist =0;
		std_msgs::Float64MultiArray learnedFace =faces[learningIdx].at(j);
		for(int i=0; i<len;i++)
			dist+=(learnedFace.data[i]-newF.data[i])*(learnedFace.data[i]-newF.data[i]);
		dist=sqrt(dist);
		printf("Dists:%f\n",dist);
		if(dist<minDist){
			minIdx=j;
			minDist=dist;
		}
	}
	printf("Min distance is between i=%d\n",minIdx);
	if(minDist>1){
		printf("Adding new face!\n");
		faces[learningIdx].push_back(newF);
		writeObject(learningIdx);
	}
	else if(minDist<0.5){
		printf("This face I know already!\n");
	}
	else{
		printf("Can't decide if I know that guy or.. Fuck it!\n");
	}
}

int getFaceIndex(std_msgs::Float64MultiArray desc){
	printf("Starting face estimation!\n");
	int minIdx;
	double minVal= 30000;
	for(int i=0;i<12;i++){
		double minDistTo_i = faceSimilarity(faces[i], desc);
		if(minDistTo_i <= 0)
			printf("this face %d has not yet been memorized\n", i);
		else{
			printf("Dist to %s is:%f\n",names.at(i).c_str() , minDistTo_i);
			if(minDistTo_i < minVal){
				minIdx=i;
				minVal = minDistTo_i;
			}
		}
	}
	if(minVal < 0.9){
		printf("Concluded that this face is %s\n", names.at(minIdx).c_str());
		return minIdx;
	}
	else{
		printf("Cannot estimate who this face is.\n");
		return -1;
	}

}

void callback(const std_msgs::Float64MultiArray::ConstPtr&  descPtr) {
	std_msgs::Float64MultiArray desc = *descPtr;
	if(learningIdx!=-1)
		learnFace(desc);
	else{
		
		int faceIdx=getFaceIndex(desc);
		// publish face idx?
	}
	
}
int main (int argc, char** argv) {
	
	// Initialize ROS
	ros::init (argc, argv, "distanceCalculator");
	ros::NodeHandle nh;
	
	if(argc > 1){
		learningIdx=atoi(argv[1]);
	}
	else{
		for(int i=0;i<12;i++)
			readFaces(i);
	}

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("faceDesc", 1, callback);
	ros::Rate r(5);
	// Spin
	ros::spin ();


}


void writeObject(int n){
	stringstream stream;
	stream<<descriptorLocation<<n;
	string file = stream.str();
	std::ofstream myHuman;
	myHuman.open(file.c_str());
  	for(int j=0; j<faces[n].size();j++){
  		for(int i=0; i<faces[n].at(j).data.size();i++)
  			myHuman << faces[n].at(j).data[i]<< " ";
  		myHuman<<"\n";
  	}
  	
  	myHuman.close();
}
bool readFaces(int n){
	stringstream stream;
	stream<<descriptorLocation<<n;
	string file = stream.str();
	std::ifstream myHuman;
	myHuman.open(file.c_str());

	if (!myHuman.is_open()){
		printf("Face descriptors for %d non existant in library!\n", n);
		names.push_back("noName");
		return false;
	}
	string name;
	getline(myHuman, name);
	names.push_back(name);
	printf("Reading all face descriptors for %d! Name:%s\n", n, name.c_str());
	for(string line; getline(myHuman, line); ){
		istringstream in(line);
		std_msgs::Float64MultiArray newSavedDesc;
		double val;
		while(in>>val)
			newSavedDesc.data.push_back(val);
		faces[n].push_back(newSavedDesc);
	}
	myHuman.close();
	return true;
}
