#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
double speed;
double angular;

void userInputCallback(const geometry_msgs::Twist& msg)
{
  speed = msg.linear.x;
  angular = msg.angular.z;
  // if (msg.angular.z > 0){
  //   ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
  // }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manual_node");
  ros::NodeHandle nh;

  // Subscribe to the point cloud topic
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 100, userInputCallback);

  // publishes to manual speed and angular
  ros::Publisher manual_speed_pub = nh.advertise<std_msgs::Float64>("/manual/speed", 10);
  ros::Publisher manual_angular_pub = nh.advertise<std_msgs::Float64>("/manual/angular", 10);
  speed = 0.0;
  angular = 0.0;

  
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Float64 speed_msg;
    speed_msg.data = speed;
    manual_speed_pub.publish(speed_msg);

    std_msgs::Float64 angular_msg;
    angular_msg.data = angular;
    manual_angular_pub.publish(angular_msg);

    ros::spinOnce();
    loop_rate.sleep();

  }

  // Spin to process the point cloud
  ros::spin();
}