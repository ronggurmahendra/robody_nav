#include <ros/ros.h>
#include <std_msgs/Float64.h>

double speed;
double angle;

void speedCallback(const std_msgs::Float64::ConstPtr& msg)
{
  speed = msg->data;
}

void angleCallback(const std_msgs::Float64::ConstPtr& msg)
{
  angle = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");
  ros::NodeHandle n;

  ros::Publisher left_motor_pub = n.advertise<std_msgs::Float64>("left_motor_speed", 10);
  ros::Publisher right_motor_pub = n.advertise<std_msgs::Float64>("right_motor_speed", 10);
  ros::Subscriber speed_sub = n.subscribe("speed_input", 10, speedCallback);
  ros::Subscriber angle_sub = n.subscribe("angle_input", 10, angleCallback);

  speed = 0.0;
  angle = 0.0;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    double left_speed = speed * (1 - angle / 90);
    double right_speed = speed * (1 + angle / 90);

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
