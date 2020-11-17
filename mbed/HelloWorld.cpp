#include "mbed.h"
#include <ros.h>
#include <BMI088.h>
#include <Esc.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Accel.h>

#include <bits/stdc++.h>
using namespace std;

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;

Esc motor[4] = {p25, p24, p23, p22};
BMI088 bmi088;

// ros variables
ros::NodeHandle nh;
std_msgs::Float32MultiArray duties;
ros::Publisher duties_pub("now_duty", &duties);
std_msgs::String echo;
ros::Publisher debugger("debug_message", &echo); 
geometry_msgs::Accel accel;
ros::Publisher acc_gyro("acc_gyro", &accel);

void publish_string(string message)
{
    int length = message.length();
    char char_array[length + 1];
    strcpy(char_array, message.c_str());
    echo.data = char_array;
    debugger.publish(&echo);
}

void get_acc_gyro()
{   
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    accel.linear.x = ax;
    accel.linear.y = ay;
    accel.linear.z = az;
    accel.angular.x = gx;
    accel.angular.y = gy;
    accel.angular.z = gz;
    // bmi088.getAcceleration(&accel.linear.x, &accel.linear.y, &accel.linear.z);
    // bmi088.getGyroscope(&accel.angular.x, &accel.angular.y, &accel.angular.z);
    acc_gyro.publish(&accel);
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
    led3 = 0;
    // publish_string("initialize mbed...");
    while (1) 
    {
        if (bmi088.isConnection()) {
            bmi088.initialize();
            // publish_string("BMI088 is connected");
            break;
        } else {
            // publish_string("BMI088 is not connected");
        }
        led3 = !led3;
        wait_ms(200);
    }
    led3 = 1;    
    duties.data_length = 4;
    duties.data = (float *)malloc(sizeof(float)*4);
}

void init_ros()
{
    nh.initNode();
    nh.advertise(duties_pub);
    nh.advertise(debugger);
    nh.advertise(acc_gyro);
    // publish_string("finish ros_init!!!");
    // nh.subscribe(sub);
}

int main()
{
    led1 = 0;
    init_ros();
    init_mbed();
    led1 = 1;
    // publish_string("start loop!");
    while(1)
    {
        nh.spinOnce();
        led2 = !led2;
        get_acc_gyro();
        // publish_string("loop!");
        wait_ms(1000);
    }
    return 0;
}