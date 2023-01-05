#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Process the point cloud here
  // ...
  
  int num_points = cloud->points.size();
  // Access the data in the point cloud
  float x = cloud->points[0].x;
  float y = cloud->points[0].y;
  float z = cloud->points[0].z;

  std::cout << "size : " << num_points << "x: "<< x <<"y: "<<  y <<"z: "<<  z << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_node");
  std::cout << "point_cloud_node initialized ..." << std::endl;
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe("/royale_camera_driver/point_cloud", 1, pointCloudCallback);

  // Spin to process the point cloud
  ros::spin();
}