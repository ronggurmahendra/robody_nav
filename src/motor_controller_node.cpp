#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <navigation_package/drive_modes.hpp>
#include <std_msgs/Int8.h>
double manualSpeed;
double manualAngular;
double potentialFieldSpeed;
double potentialFieldAngular;
drive_Modes curr_drive_modes;

void manualSpeedCallback(const std_msgs::Float64::ConstPtr& msg)
{
  manualSpeed = msg->data;
}

void manualAngularCallback(const std_msgs::Float64::ConstPtr& msg)
{
  manualAngular = msg->data;
}


void potentialFieldSpeedCallback(const std_msgs::Float64::ConstPtr& msg)
{
  potentialFieldSpeed = msg->data;
}

void potentialFieldAngularCallback(const std_msgs::Float64::ConstPtr& msg)
{
  potentialFieldAngular = msg->data;
}

void driveModeCallback(const std_msgs::Int8& msg){
  curr_drive_modes = static_cast<drive_Modes>(msg.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");
  ros::NodeHandle n;

  // this publishes the left and right motor speed
  ros::Publisher left_motor_pub = n.advertise<std_msgs::Float64>("/motor/left_motor_speed", 10);
  ros::Publisher right_motor_pub = n.advertise<std_msgs::Float64>("/motor/right_motor_speed", 10);

  // this subscribes the manual speed input and angular input  
  ros::Subscriber manual_speed_sub = n.subscribe("/manual/speed", 10, manualSpeedCallback);
  ros::Subscriber manual_angular_sub = n.subscribe("/manual/angular", 10, manualAngularCallback);
  
  // this subscribes the Potential field speed input and angular input  
  ros::Subscriber potentialField_speed_sub = n.subscribe("/potentialField/speed", 10, potentialFieldSpeedCallback);
  ros::Subscriber potentialField_angular_sub = n.subscribe("/potentialField/angular", 10, potentialFieldAngularCallback);
  
  // this subscribes the driveModes input
  ros::Subscriber driveMode_sub = n.subscribe("/motorControler/driveMode", 10, driveModeCallback);

  // initialize the variables default values
  curr_drive_modes = Manual;
  manualSpeed = 0.0;
  manualAngular = 0.0;
  potentialFieldSpeed = 0.0;
  potentialFieldAngular = 0.0;

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    //publishes the left and right motor
    double left_speed,right_speed;
    switch(curr_drive_modes){
      case Manual:
        //calculate the each motor speed in case of manual
        left_speed = manualSpeed * (1 - manualAngular / 90);
        right_speed = manualSpeed * (1 + manualAngular / 90);
        break;
      case potentialField:
        //calculate the each motor speed in case of potentialField
        left_speed = potentialFieldSpeed * (1 - potentialFieldAngular / 90);
        right_speed = potentialFieldSpeed * (1 + potentialFieldAngular / 90);
        break;
      default: //in case of unrecognizeable mode switches to manual 
        //calculate the each motor speed in case of manual
        left_speed = manualSpeed * (1 - manualAngular / 90);
        right_speed = manualSpeed * (1 + manualAngular / 90);
        break;
    }

    std_msgs::Float64 left_msg;
    left_msg.data = left_speed;
    left_motor_pub.publish(left_msg);

    std_msgs::Float64 right_msg;
    right_msg.data = right_speed;
    right_motor_pub.publish(right_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
