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

ros::Publisher pub;

int getCol(int r, int g, int b){
	int lab;
	if(r*0.5>g && r*0.5>b){
		//RED OBJECT
		lab=1;
	}else if(g*0.8>r && g*0.8>b){
		//GREEN OBJECT
		lab=2;
	}else if(b*0.6>r && b*0.6>g){
		//BLUE OBJECT
		lab=3;
	/*}else if(r*0.5>b && g*0.5>b){
		//YELLOW OBJECT
		lab=4;*/
	}else if(r<10 && g<10 && b<10){
		//BLACK OBJECT
		lab=0;
	}else{
		lab=-1;
	}
	return lab;
}


void callback(const pcl::PCLPointCloud2ConstPtr& cloud_blob) {
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outliers (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
	
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
	
	pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
	int num=0;
	for (b1 = cloud_filtered->points.begin(); b1 < cloud_filtered->points.end(); b1++){
		pcl::PointXYZRGB newpoint;
		newpoint.x = b1->x;
    	newpoint.y = b1->y;
    	newpoint.z = b1->z;
		newpoint.r = b1->r;
    	newpoint.g = b1->g;
    	newpoint.b = b1->b;
		int r, g, b;
		r=newpoint.r;
		g=newpoint.g;
		b=newpoint.b;
		int label=-1;
		if(b*0.5>r && b*0.5>g){
			num++;
			//printf("BLUE: %d, %d, %d\n",r,g,b);
			color_filtered->push_back(newpoint);
		}
	}
	cloud_filtered->points=color_filtered->points;
	cloud_filtered->width=num;
	
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_CIRCLE3D);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (1000);
	seg.setDistanceThreshold (0.02);
	seg.setRadiusLimits(0.04,0.1);
	seg.setInputCloud (cloud_filtered);

	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size () == 0) return;
	if(coefficients->values[1]<-0.28 || coefficients->values[1]>-0.08) return;
	if(sqrt(coefficients->values[1]*coefficients->values[1]+coefficients->values[0]*coefficients->values[0]+coefficients->values[2]*coefficients->values[2])>2) return;//so far away from me
	printf("BLUE Center: %f, %f, %f\n",coefficients->values[0],coefficients->values[1],coefficients->values[2]);
	
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	//remove the plane
	//extract.setNegative(true);
	//show the plane only
	extract.setNegative(false);
	extract.filter(*cloud_outliers);

	
	
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/base_link";
	marker.header.stamp = ros::Time::now();
	marker.ns = "BlueMarker";
	marker.id = 3;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = (double)coefficients->values[2]-0.1;//KOLIKO PRED NJIM
	marker.pose.position.y = (double)-coefficients->values[0];//KOLIKO LEVO OD NJEGA
	marker.pose.position.z = (double)-coefficients->values[1]+0.3;//KOLIKO NAD NJIM
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.1;
	marker.color.g = 0.1;
	marker.color.b = 1.0;
	
	pub.publish(marker);
	
}



int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "find_blu");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<visualization_msgs::Marker> ("filteredBlue", 1);

  
  ros::Rate r(5);
  // Spin
  ros::spin ();
  
 
}
