/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include "mbed.h"
#include <ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

#include <string>
using std::string;

#define PERIOD  20

ros::NodeHandle nh;
// geometry_msgs::Pose2D pose;
std_msgs::Float32 duty;
ros::Publisher pub("now_duty", &duty);

DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
std_msgs::UInt32 cnt;

// std_msgs::String str_msg;
ros::Publisher chatter("chatter", &cnt);

// char hello[13] = "hello world!";

void update_duty(const std_msgs::Float32& input_duty)
{
    duty.data = input_duty.data;
    pub.publish(&duty);
    chatter.publish(&cnt);
    led1 = !led1;   
    cnt.data = cnt.data + 1;
}

ros::Subscriber<std_msgs::Float32> sub("input_duty", &update_duty);

int main()
{
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    nh.advertise(chatter);
    cnt.data = 0;
    while(1)
    {
        nh.spinOnce();
        wait_ms(10);
        led2 = !led2;
    }
}