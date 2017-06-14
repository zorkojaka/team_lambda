#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

#include <geometry_msgs/PointStamped.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


ros::Publisher pub;

typedef pcl::PointXYZRGB PointT;
tf::TransformListener* listener;


double rAvg = 0, gAvg = 0, bAvg = 0;
int i = 0;
char *namespaces[] = {"undefined", "red", "green", "blue", "yellow", "black"};
double mrkR[] = {1.0, 1.0, 0.1, 0.1, 1.0, 0.1};
double mrkG[] = {1.0, 0.1, 1.0, 0.1, 1.0, 0.1};
double mrkB[] = {1.0, 0.1, 0.1, 1.0, 0.1, 0.1};
int thisNode = 0;
int minNumInliers = 800;
int numInliers = 0;
pcl::ModelCoefficients::Ptr doRANSAC(pcl::PointCloud<PointT>::Ptr cylinderPoints)
{

	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;

	pcl::ExtractIndices<PointT> extract;

	// Estimate point normals
	ne.setSearchMethod(tree);
	ne.setInputCloud(cylinderPoints);
	ne.setKSearch(50);
	ne.compute(*cloud_normals);

	pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_CYLINDER);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.12);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.04);
	seg.setRadiusLimits(0.08, 0.15);
	seg.setInputCloud(cylinderPoints);
	seg.setInputNormals(cloud_normals);
	pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
	seg.segment(*inliers_cylinder, *coeff);

	extract.setInputCloud(cylinderPoints);
	extract.setIndices(inliers_cylinder);
	extract.setNegative(false);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());

	extract.filter(*cloud_filtered);
	numInliers = cloud_filtered->size();
	return coeff;
}

void callback(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients = doRANSAC(cloud_filtered);

	if (numInliers < minNumInliers)
	{
		return;
	}
	//if(coefficients->values[1]<-0.28 || coefficients->values[1]>-0.08) return;
	if (sqrt(coefficients->values[1] * coefficients->values[1] + coefficients->values[0] * coefficients->values[0] + coefficients->values[2] * coefficients->values[2]) > 2)
	{
		return; //so far away from me
	}

	printf("CYLINDER %s Center: %f, %f, %f\n", namespaces[thisNode], coefficients->values[0], coefficients->values[1], coefficients->values[2]);

	geometry_msgs::PointStamped poseBaseLink;
	poseBaseLink.header.frame_id = "/base_link";
	poseBaseLink.header.stamp = ros::Time(0);
	poseBaseLink.point.x = (double)coefficients->values[2] - 0.1; //KOLIKO PRED NJIM
	poseBaseLink.point.y = (double)-coefficients->values[0]; //KOLIKO LEVO OD NJEGA
	poseBaseLink.point.z = 0.0; //KOLIKO NAD NJIM
	
	geometry_msgs::PointStamped poseMap;

	// transform to /map frame
	try{
      listener->transformPoint("/map", poseBaseLink, poseMap);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }


	visualization_msgs::Marker marker;
	marker.header.frame_id = "/map";
	marker.header.stamp = ros::Time::now();
	marker.ns = namespaces[thisNode];
	marker.id = 3;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = poseMap.point.x; //KOLIKO PRED NJIM
	marker.pose.position.y = poseMap.point.y; //KOLIKO LEVO OD NJEGA
	marker.pose.position.z = 0.2; //KOLIKO NAD NJIM
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.40;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = mrkR[thisNode];
	marker.color.g = mrkG[thisNode];
	marker.color.b = mrkB[thisNode];

	pub.publish(marker);
}

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
		printf("Started node! Its color is:%s\n", namespaces[thisNode]);
	}
	else
		thisNode = 0;
	ros::init(argc, argv, "find_cyl");
	listener = new tf::TransformListener();

	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe("input", 1, callback);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<visualization_msgs::Marker>("cylinder", 1);

	// Spin
	ros::spin();
}
