#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pubR;
ros::Publisher pubG;
ros::Publisher pubB;
ros::Publisher pubY;
ros::Publisher pubK;

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
	}else if(r*0.5>b && g*0.5>b){
		//YELLOW OBJECT
		lab=4;
		
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorR (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorG (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorB (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorY (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorK (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_blob);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered_blob);

	pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

	pcl::PointCloud<pcl::PointXYZRGB>::iterator b1;
	for (b1 = cloud_filtered->points.begin(); b1 < cloud_filtered->points.end(); b1++){
		int label=getCol(b1->r,b1->g,b1->b);
		if(label==-1)continue;
		pcl::PointXYZRGB newpoint;
		newpoint.x = b1->x;
		newpoint.y = b1->y;
		newpoint.z = b1->z;
		newpoint.r = b1->r;
		newpoint.g = b1->g;
		newpoint.b = b1->b;
		switch(label){
			case 0:colorK->push_back(newpoint);
			break;
			case 1:colorR->push_back(newpoint);
			break;
			case 2:colorG->push_back(newpoint);
			break;
			case 3:colorB->push_back(newpoint);
			break;
			case 4:colorY->push_back(newpoint);
			break;
			default:printf("Unknown point cloud color label!\n");
			break;
		}
	}
	pubR.publish(colorR);
	pubG.publish(colorG);
	pubB.publish(colorB);
	pubY.publish(colorY);
	pubK.publish(colorK);

}



int main (int argc, char** argv) {

  // Initialize ROS
  ros::init (argc, argv, "find_blu");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, callback);

  // Create a ROS publisher for the output point cloud
  pubR = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ("pclR", 1);
  pubG = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ("pclG", 1);
  pubB = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ("pclB", 1);
  pubY = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ("pclY", 1);
  pubK = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ("pclK", 1);

  
  ros::Rate r(5);
  // Spin
  ros::spin ();
  
 
}
