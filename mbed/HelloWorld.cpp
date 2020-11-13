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

#define PERIOD  10
#define MIN_DUTY 0.11  //Period 10ms
#define MAX_DUTY 0.21  //Period 10ms

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
PwmOut motor1(p21);

// ros variables
ros::NodeHandle nh;
std_msgs::Float32 duty;
ros::Publisher pub("now_duty", &duty);
std_msgs::UInt32 cnt;
ros::Publisher chatter("chatter", &cnt);

void write_duty(float value)
{
    if(value > MAX_DUTY){value = MAX_DUTY;}
    if(value < MIN_DUTY){value = MIN_DUTY;}
    motor1.write(value);
}

void update_duty(const std_msgs::Float32& input_duty)
{
    write_duty(input_duty.data);
    pub.publish(&input_duty);
    chatter.publish(&cnt);
    led1 = !led1;   
    cnt.data = cnt.data + 1;
}
ros::Subscriber<std_msgs::Float32> sub("input_duty", &update_duty);

void init_mbed()
{
    motor1.period_ms(PERIOD);
    write_duty(0.1);  //esc is initialized by 10% duty
    wait_ms(1000);
}

void init_ros()
{
    nh.initNode();
    nh.advertise(pub);
    nh.subscribe(sub);
    nh.advertise(chatter);
    cnt.data = 0;
    duty.data = 0.0;
}

int main()
{
    init_ros();
    init_mbed();
    while(1)
    {
        nh.spinOnce();
        wait_ms(10);
        led2 = !led2;
    }
}