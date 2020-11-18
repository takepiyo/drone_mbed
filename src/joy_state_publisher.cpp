#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
// #include "std_msgs/MultiArrayLayout.h"
// #include "std_msgs/MultiArrayDimension.h"
// #include <std_msgs/Float32MultiArray.h>

#define MOTOR_NUM 4

std_msgs::Float32 duties;
ros::Publisher cmd_pub;
ros::Subscriber joy_sub;

void joy_callback(const sensor_msgs::Joy& joy_msg)
{
  // duties.data[0] = joy_msg.axes[1];
  // duties.data[1] = joy_msg.axes[1];
  // duties.data[2] = joy_msg.axes[1];
  // duties.data[3] = joy_msg.axes[1];
  // duties.x = joy_msg.axes[1];
  // duties.y = joy_msg.axes[1];
  // duties.z = joy_msg.axes[1];
  // duties.w = joy_msg.axes[1];
  duties.data = joy_msg.axes[1];
  cmd_pub.publish(duties);
}

int main(int argc, char** argv)
{
  // duties.data.resize(MOTOR_NUM);
  ros::init(argc, argv, "joy_state_publisher");
  ros::NodeHandle nh;
  // cmd_pub = nh.advertise<std_msgs::Float32MultiArray>("input_duties", 1000);
  cmd_pub = nh.advertise<std_msgs::Float32>("input_duties", 1000);
  joy_sub = nh.subscribe("joy", 1000, joy_callback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}