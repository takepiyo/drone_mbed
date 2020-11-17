#include "mbed.h"
#include <ros.h>
#include <BMI088.h>
#include <Esc.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#define PERIOD  10
#define MIN_DUTY 0.10  //Period 10ms
#define MAX_DUTY 0.21  //Period 10ms

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;

Esc motor[4] = {p25, p24, p23, p22};

// ros variables
ros::NodeHandle nh;
std_msgs::Float32MultiArray duties;
ros::Publisher duties_pub("now_duty", &duties);
std_msgs::String echo;
ros::Publisher debugger("debug_message", &echo); 

void publish_string(char* message)
{
    echo.data = message;
    debugger.publish(&echo);
}

// void init_duty()
// {
//     motor1.period_ms(PERIOD);
//     std_msgs::Float32 initial_duty;
//     initial_duty.data = 0.1;  //esc is initialized by 10% duty
//     write_duty(initial_duty);  
// }

// void update_duty(const std_msgs::Float32& input_duty)
// {
//     write_duty(input_duty);
//     // chatter.publish(&cnt);
//     led1 = !led1;   
//     // cnt.data = cnt.data + 1;
// }
// ros::Subscriber<std_msgs::Float32> sub("input_duty", &update_duty);

// void write_duty(std_msgs::Float32 input_duty)
// {
//     input_duty.data = (MAX_DUTY - MIN_DUTY) * input_duty.data + MIN_DUTY;
//     motor1.write(input_duty.data);
//     pub.publish(&input_duty);
// }

void init_mbed()
{
    publish_string("initialize mbed...");
    wait_ms(500);
}

void init_ros()
{
    publish_string("initialize ros...");
    nh.initNode();
    nh.advertise(duties_pub);
    nh.advertise(debugger);
    // nh.subscribe(sub);
}

int main()
{
    init_ros();
    init_mbed();
    publish_string("start loop!");
    while(1)
    {
        nh.spinOnce();
        wait_ms(1);
        led2 = !led2;
    }
    return 0;
}