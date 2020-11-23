#include "mbed.h"
#include <ros.h>
#include <BMI088.h>
#include <Esc.h>
#include <std_msgs/String.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Accel.h>

#include <vector>
#include <bits/stdc++.h>
using namespace std;

#define MOTOR_NUM 4

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p23, p22};
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

void update_duties(const std_msgs::Float32MultiArray& input_duties)
{
    duties = input_duties;
    led4 = !led4;
}

void update_motor_rotation()
{
    motor[0].update(duties.data[0]);
    motor[1].update(duties.data[1]);
    motor[2].update(duties.data[2]);
    motor[3].update(duties.data[3]);
    duties_pub.publish(&duties);
}
ros::Subscriber<std_msgs::Float32MultiArray> duties_sub("input_duties", &update_duties);

void init_mbed()
{
    led3 = 0;
    led4 = 0;
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
    duties.data_length = MOTOR_NUM;
    duties.data = (float *)malloc(sizeof(float)*MOTOR_NUM);
    for(int i=0; i < MOTOR_NUM; i++)
    {
        duties.data[i] = 0.0;
    }
}

void init_ros()
{
    nh.initNode();
    nh.advertise(duties_pub);
    nh.advertise(debugger);
    nh.advertise(acc_gyro);
    // publish_string("finish ros_init!!!");
    nh.subscribe(duties_sub);
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
        update_motor_rotation();
        // publish_string("loop!");
        wait_ms(100);
    }
    return 0;
}