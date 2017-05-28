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

ros::Publisher pub;

typedef pcl::PointXYZRGB PointT;

double rAvg=0,gAvg=0,bAvg=0;
int i=0;

pcl::ModelCoefficients::Ptr doRANSAC ( pcl::PointCloud<PointT>::Ptr cylinderPoints ) {
	
 	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<PointT, pcl::Normal> ne;

	pcl::ExtractIndices<PointT> extract;
	
	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cylinderPoints);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	pcl::ModelCoefficients::Ptr coeff ( new pcl::ModelCoefficients );
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
	seg.setOptimizeCoefficients ( true );
	seg.setModelType ( pcl::SACMODEL_CYLINDER );
	seg.setMethodType ( pcl::SAC_RANSAC );
	seg.setNormalDistanceWeight ( 0.12 );
	seg.setMaxIterations ( 1000 );
	seg.setDistanceThreshold ( 0.04 );
	seg.setRadiusLimits (0.08, 0.15 );
	seg.setInputCloud ( cylinderPoints );
	seg.setInputNormals ( cloud_normals );
	pcl::PointIndices::Ptr inliers_cylinder ( new pcl::PointIndices );
	seg.segment ( *inliers_cylinder, *coeff );
	extract.setInputCloud (cylinderPoints);
	extract.setIndices (inliers_cylinder);
	extract.setNegative (false);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
	
	extract.filter (*cloud_filtered);
	rAvg=0;
	gAvg=0;
	bAvg=0;
	i=0;
	
	pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
	for (b1 = cloud_filtered->points.begin(); b1 < cloud_filtered->points.end(); b1++){
		rAvg+=b1->r;
		gAvg+=b1->g;
		bAvg+=b1->b;
		i++;
	}
	rAvg/=i*255;
	gAvg/=i*255;
	bAvg/=i*255;
	return coeff;
} 

void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_outliers (new pcl::PointCloud<PointT>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

	pcl::ModelCoefficients::Ptr coefficients =doRANSAC(cloud_filtered);

	//if(coefficients->values[1]<-0.28 || coefficients->values[1]>-0.08) return;
	if(sqrt(coefficients->values[1]*coefficients->values[1]+coefficients->values[0]*coefficients->values[0]+coefficients->values[2]*coefficients->values[2])>2) return;//so far away from me
	printf("CYLINDER Center: %f, %f, %f\n",coefficients->values[0],coefficients->values[1],coefficients->values[2]);


	
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "CylinderMarker";
	marker.id = 3;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = (double)coefficients->values[2]-0.1;//KOLIKO PRED NJIM
	marker.pose.position.y = (double)-coefficients->values[0];//KOLIKO LEVO OD NJEGA
	marker.pose.position.z = 0.0;//(double)-coefficients->values[1]+0.3;//KOLIKO NAD NJIM
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = rAvg;
	marker.color.g = gAvg;
	marker.color.b = bAvg;
	
	pub.publish(marker);
}


int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "find_cyl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<visualization_msgs::Marker> ("cylinder", 1);

  // Spin
  ros::spin ();
}
