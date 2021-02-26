#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#define MOTOR_NUM 4

// std_msgs::Float32MultiArray duties;
std_msgs::Float32 ref_z;
ros::Publisher ref_z_pub;
std_msgs::Float32 ref_roll;
ros::Publisher ref_roll_pub;
std_msgs::Float32 ref_pitch;
ros::Publisher ref_pitch_pub;
std_msgs::Float32 ref_yaw;
ros::Publisher ref_yaw_pub;
std_msgs::Empty toggle;
ros::Publisher emergency_stop_pub;
ros::Publisher start_control_pub;
ros::Subscriber joy_sub;

void joy_callback(const sensor_msgs::Joy& joy_msg) {
  ref_z.data     = joy_msg.axes[1];
  ref_roll.data  = joy_msg.axes[3];
  ref_pitch.data = joy_msg.axes[4];
  ref_yaw.data   = joy_msg.axes[0];
  if (joy_msg.buttons[11] == 1 && joy_msg.buttons[12] == 1) { emergency_stop_pub.publish(toggle); }
  if (joy_msg.axes[2] < -0.5 && joy_msg.axes[5] < -0.5) { start_control_pub.publish(toggle); }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_state_publisher");
  ros::NodeHandle nh;
  ref_z_pub          = nh.advertise<std_msgs::Float32>("ref_z", 1000);
  ref_roll_pub       = nh.advertise<std_msgs::Float32>("ref_roll", 1000);
  ref_pitch_pub      = nh.advertise<std_msgs::Float32>("ref_pitch", 1000);
  ref_yaw_pub        = nh.advertise<std_msgs::Float32>("ref_yaw", 1000);
  emergency_stop_pub = nh.advertise<std_msgs::Empty>("emergency_stop", 1000);
  start_control_pub  = nh.advertise<std_msgs::Empty>("stable", 1000);
  joy_sub            = nh.subscribe("joy", 1000, joy_callback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    ref_z_pub.publish(ref_z);
    ref_roll_pub.publish(ref_roll);
    ref_pitch_pub.publish(ref_pitch);
    ref_yaw_pub.publish(ref_yaw);
    loop_rate.sleep();
  }
  return 0;
}