#include "mbed.h"

#include <BMI088.h>
#include <Esc.h>
#include <ExtendedKalmanFilter.h>

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Vector3.h>

#include <vector>
#include <bits/stdc++.h>

#define MOTOR_NUM 4
#define PERIOD    0.01

// declare functions
void update_pose();

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p22, p23};
BMI088 bmi088(p9, p10);
Ekf    ex_kalman_filter(PERIOD);

Ticker timer;

// ros variables
ros::NodeHandle nh;
// std_msgs::Float32MultiArray duties;
std_msgs::Float32 duties;
ros::Publisher duties_pub("now_duty", &duties);
std_msgs::String echo;
ros::Publisher debugger("debug_message", &echo); 
geometry_msgs::Accel accel;
ros::Publisher acc_gyro("acc_gyro", &accel);
geometry_msgs::Vector3 linear_acc;
geometry_msgs::Vector3 angular_vel;
geometry_msgs::Vector3 RPY_angle;
ros::Publisher RPY_pub("RPY_angle", &RPY_angle);
// tf2::Quaternion quaternion;

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
    linear_acc.x = ax;
    linear_acc.y = ay;
    linear_acc.z = az;
    angular_vel.x = gx;
    angular_vel.y = gy;
    angular_vel.z = gz;
    // bmi088.getAcceleration(&accel.linear.x, &accel.linear.y, &accel.linear.z);
    // bmi088.getGyroscope(&accel.angular.x, &accel.angular.y, &accel.angular.z);
    acc_gyro.publish(&accel);
}

void publish_acc_gyro()
{
    accel.linear = linear_acc;
    accel.angular = angular_vel;
    acc_gyro.publish(&accel);
}

// void update_duties(const std_msgs::Float32MultiArray& input_duties)
void update_duties(const std_msgs::Float32& input_duties)
{
    duties = input_duties;
    led4 = !led4;
}

void update_motor_rotation()
{
    motor[0].update(duties.data);
    motor[1].update(duties.data);
    motor[2].update(duties.data);
    motor[3].update(duties.data);
    // motor.update(duties.data);
    duties_pub.publish(&duties);
}
// ros::Subscriber<std_msgs::Float32MultiArray> duties_sub("input_duties", &update_duties);
ros::Subscriber<std_msgs::Float32> duties_sub("input_duties", &update_duties);

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
    // duties.data_length = MOTOR_NUM;
    // duties.data = (float *)malloc(sizeof(float)*MOTOR_NUM);
    // for(int i=0; i < MOTOR_NUM; i++)
    // {
    //     duties.data[i] = 0.0;
    // }
    duties.data = 0.0;
}

void init_ros()
{
    nh.initNode();
    nh.advertise(duties_pub);
    nh.advertise(debugger);
    nh.advertise(acc_gyro);
    nh.advertise(RPY_pub);
    // publish_string("finish ros_init!!!");
    nh.subscribe(duties_sub);
}

int main()
{
    led1 = 0;
    init_ros();
    init_mbed();
    led1 = 1;
    // publish_stri ng("start loop!");
    // timer.attach(&update_pose, PERIOD);
    while(1)
    {
        // get_acc_gyro();
        // update_motor_rotation();
        // publish_string("loop!");
        // publish_acc_gyro();
        update_pose();
        RPY_pub.publish(&RPY_angle);
        nh.spinOnce();
        wait(PERIOD);
        led2 = !led2;
    }
    return 0;
}

void update_pose()
{
    get_acc_gyro();
    RPY_angle = ex_kalman_filter.get_corrected(linear_acc, angular_vel);
}