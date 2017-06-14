#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>

#include <std_msgs/Bool.h>
void callback(const pcl::PCLPointCloud2ConstPtr &cloud_blob);
void toggleCallback(const std_msgs::Bool &toggle);

char *namespaces[] = {"undefined", "red", "green", "blue", "yellow", "black"};
double mrkR[] = {1.0, 1.0, 0.1, 0.1, 1.0, 0.1};
double mrkG[] = {1.0, 0.1, 1.0, 0.1, 1.0, 0.1};
double mrkB[] = {1.0, 0.1, 0.1, 1.0, 0.1, 0.1};
double radMin[] = {0.0, 0.045, 0.07, 0.035, 0.0, 0.015};
double radMax[] = {1.0, 0.09, 0.11, 0.06, 1.0, 0.038};

int thisNode = 0;
int minNumInliers[] = {40, 30, 30,25, 40, 8};


ros::Publisher rvizPub;
ros::Publisher posePub;

// frame transformer
tf::TransformListener* listener;

//tf::StampedTransform transform;

int main(int argc, char **argv)
{
	// Initialize ROS
	if (argc > 1)
	{
		if (strcmp(argv[1], "red") == 0)
			thisNode = 1;
		else if (strcmp(argv[1], "green") == 0)
			thisNode = 2;
		else if (strcmp(argv[1], "blue") == 0)
			thisNode = 3;
		else if (strcmp(argv[1], "yellow") == 0)
			thisNode = 4;
		else if (strcmp(argv[1], "black") == 0)
			thisNode = 5;
		else
			thisNode = 0;
		printf("Started circle node! Its color is:%s\n", namespaces[thisNode]);
	}
	else
		thisNode = 0;

	ros::init(argc, argv, "find_circ");
	ros::NodeHandle nh;

	listener = new tf::TransformListener();


	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, callback);

	// Create a ROS publisher for the output point cloud
	rvizPub = nh.advertise<visualization_msgs::Marker>("marker", 1);
	posePub = nh.advertise<geometry_msgs::PointStamped>("pose", 1);

	ros::Rate r(5);
	
	// Spin
	while(ros::ok()){
		ros::spinOnce();
	}
}

void callback(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_CIRCLE3D);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.02);
	seg.setRadiusLimits(radMin[thisNode], radMax[thisNode]);
	seg.setInputCloud(cloud_filtered);

	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() < minNumInliers[thisNode])
		return;
	double xLoc = (double)coefficients->values[2] - 0.1;
	double yLoc = (double)-coefficients->values[0];
	double zLoc = (double)-coefficients->values[1] + 0.3;
	//if(thisNode == 3)
	//	printf("%s numInliers:%d\n", namespaces[thisNode], inliers->indices.size());
	if(zLoc<0.45 || zLoc>0.65)
		return;
	printf("CIRC %s Center: %f, %f, %f\n", namespaces[thisNode], xLoc, yLoc, zLoc);

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	//remove the plane
	//extract.setNegative(true);
	//show the plane only
	extract.setNegative(false);
	extract.filter(*cloud_outliers);

	// Detected point publisher
	geometry_msgs::PointStamped poseBaseLink;
	poseBaseLink.header.frame_id = "/base_link";
	poseBaseLink.header.stamp = ros::Time(0);
	poseBaseLink.point.x = xLoc; //KOLIKO PRED NJIM
	poseBaseLink.point.y = yLoc; //KOLIKO LEVO OD NJEGA
	poseBaseLink.point.z = zLoc; //KOLIKO NAD NJIM
	
	geometry_msgs::PointStamped poseMap;

	// transform to /map frame
	try{
      listener->transformPoint("/map", poseBaseLink, poseMap);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }

	posePub.publish(poseMap);

	// RViz visualization
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = namespaces[thisNode];
	marker.id = thisNode;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = poseMap.point.x; //KOLIKO PRED NJIM
	marker.pose.position.y = poseMap.point.y; //KOLIKO LEVO OD NJEGA
	marker.pose.position.z = poseMap.point.z; //KOLIKO NAD NJIM
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = mrkR[thisNode];
	marker.color.g = mrkG[thisNode];
	marker.color.b = mrkB[thisNode];
	rvizPub.publish(marker);
}

/*
	 Recognition on/off

	void toggleCallback(const std_msgs::Bool &toggle)
{
	active = toggle.data;
}
*/


