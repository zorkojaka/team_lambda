#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pubCont;

double contFact=1.3;

int truncationing(double x){
	if(x>255.0)return 255;
	if(x<0.0)return 0;
	return (int)x;
}


void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
	pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorK (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr cloud_colorK (new pcl::PCLPointCloud2);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm

	pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
	for (b1 = cloud_filtered->points.begin(); b1 < cloud_filtered->points.end(); b1++){
		int r=b1->r;
		int g=b1->g;
		int b=b1->b;
		
		b1->r=truncationing((((double)r-128.0)*contFact)+128.0);
		b1->g=truncationing((((double)g-128.0)*contFact)+128.0);
		b1->b=truncationing((((double)b-128.0)*contFact)+128.0);
	}
	pcl::toPCLPointCloud2(*cloud_filtered, *cloud_colorK);
	pubCont.publish(*cloud_colorK);

}

int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "contraster");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pubCont = nh.advertise<sensor_msgs::PointCloud2> ("contrastedPcl", 1);

  
  ros::Rate r(5);
  // Spin
  ros::spin ();
  
 
}
