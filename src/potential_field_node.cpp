#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>

double speed;
double angular;
double userspeed;
// void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
//   //Process images
//   if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
//     mono8_img = cv::Mat(float_img.size(), CV_8UC1);}
//   cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
//   //The following doesn't work due to NaNs
//   //double minVal, maxVal; 
//   //minMaxLoc(float_img, &minVal, &maxVal);
//   //ROS_DEBUG("Minimum/Maximum Depth in current image: %f/%f", minVal, maxVal);
//   //mono8_img = cv::Scalar(0);
//   //cv::line( mono8_img, cv::Point2i(10,10),cv::Point2i(200,100), cv::Scalar(255), 3, 8);
// }


// void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
// {
//   // // Convert the ROS image message to a OpenCV image
//   // cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//   cv_bridge::CvImagePtr cv_ptr ;
//   try
//   {
//       //Always copy, returning a mutable CvImage
//       //OpenCV expects color images to use BGR channel order.
//       cv_ptr = cv_bridge::toCvCopy(msg);
//   }
//   catch (cv_bridge::Exception& e)
//   {
//       //if there is an error during conversion, display it
//       ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
//       return;
//   }

//   //Copy the image.data to imageBuf.
//   cv::Mat depth_float_img = cv_ptr->image;
//   cv::Mat depth_mono8_img;
//   depthToCV8UC1(depth_float_img, depth_mono8_img);

//   // Access the image data
//   cv::Mat image = cv_ptr->image;
//   double minVal; 
//   double maxVal; 
//   cv::Point minLoc; 
//   cv::Point maxLoc;

//   cv::minMaxLoc( image, &minVal, &maxVal, &minLoc, &maxLoc );
//   // assign speed as the minimum value of the depth
//   std::cout << minVal;
//   // speed = minVal;
// }

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Convert the PointCloud2 message to a PCL point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // Process the point cloud here
  // ...
  
  int num_points = cloud->points.size();
  // Access the data in the point cloud
  // float x = cloud->points[0].x;
  // float y = cloud->points[0].y;
  // float z = cloud->points[0].z;
  float minDist = 99999;
  for(int i = 0; i< cloud->points.size();i++){
    float z = cloud->points[0].z;
    if(minDist > z){
      minDist = z;
    }
    
  }
  if(minDist > 1){
    minDist = 1;
  }
  if(minDist < 0){
    minDist = 0;
  }

  if(userspeed < 0){
    speed = -1 * minDist;
  }else if (userspeed > 0){
    speed = minDist;
  }else{
    speed = 0;
  }
  
  ROS_INFO_STREAM("minDist:"<<minDist);
  // std::cout << "size : " << num_points << "x: "<< x <<"y: "<<  y <<"z: "<<  z << std::endl;
}

void userInputCallback(const geometry_msgs::Twist& msg)
{
  userspeed = msg.linear.x;
  angular = msg.angular.z;
  // if (msg.angular.z > 0){
  //   ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
  // }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "potential_field_node");
  ros::NodeHandle n;
  // publishes to potentialfield speed & angular
  ros::Publisher potential_Field_speed_pub = n.advertise<std_msgs::Float64>("/potentialField/speed", 10);
  ros::Publisher potential_Field_angular_pub = n.advertise<std_msgs::Float64>("/potentialField/angular", 10);

  speed = 0.0;
  angular = 0.0;
  
  // Subscribe to the point cloud topic
  ros::Subscriber user_input_sub = n.subscribe("/cmd_vel", 100, userInputCallback);

  // Subscribe to the image topic
  // image_transport::ImageTransport it(n);
  // image_transport::Subscriber sub = it.subscribe("/royale_camera_driver/depth_image", 1, imageCallback);

  // subcribe to the point cloud topic
  ros::Subscriber pointcloud_sub = n.subscribe("/royale_camera_driver/point_cloud", 1, pointCloudCallback);


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ROS_INFO_STREAM(speed);
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    potential_Field_speed_pub.publish(speed_msg);

    std_msgs::Float64 angular_msg;
    angular_msg.data = angular;
    potential_Field_angular_pub.publish(angular_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}




