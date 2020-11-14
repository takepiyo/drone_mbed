#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

// geometry_msgs::Twist cmd_vel;
std_msgs::Float32 duty;
void joy_callback(const sensor_msgs::Joy& joy_msg)
{
  duty.data = joy_msg.axes[1];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_state_publisher");
  ros::NodeHandle nh;
  // ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Publisher cmd_pub = nh.advertise<std_msgs::Float32>("input_duty", 1000);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joy_callback);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    cmd_pub.publish(duty);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}