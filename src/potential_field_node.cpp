#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
double speed;
double angle;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // Convert the ROS image message to a OpenCV image
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

  // Access the image data
  cv::Mat image = cv_ptr->image;
  double minVal; 
  double maxVal; 
  cv::Point minLoc; 
  cv::Point maxLoc;

  cv::minMaxLoc( image, &minVal, &maxVal, &minLoc, &maxLoc );
  // assign speed as the minimum value of the depth
  std::cout << minVal;
  speed = minVal;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "potential_field_node");
  ros::NodeHandle n;
  // publishes to potentialfield speed & angle
  ros::Publisher potential_Field_speed_pub = n.advertise<std_msgs::Float64>("potentialField/speed", 10);
  ros::Publisher potential_Field_angle_pub = n.advertise<std_msgs::Float64>("potentialField/angle", 10);

  speed = 10.0;
  angle = 0.0;
  // Subscribe to the image topic
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("/royale_camera_driver/depth_image", 1, imageCallback);


  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    potential_Field_speed_pub.publish(speed_msg);

    std_msgs::Float64 angle_msg;
    angle_msg.data = angle;
    potential_Field_angle_pub.publish(angle_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}




