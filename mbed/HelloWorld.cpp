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

void update_pose();
void publish_acc_gyro();
void rad_to_deg(const geometry_msgs::Vector3& radian, geometry_msgs::Vector3& degree);
void update_pose_without_kalman();
void pose_from_acc();

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

geometry_msgs::Vector3 RPY_raw;
geometry_msgs::Vector3 RPY_raw_deg;
ros::Publisher RPY_pub_raw("RPY_raw", &RPY_raw_deg);

geometry_msgs::Vector3 RPY_kalman;
ros::Publisher RPY_pub_kalman("RPY_kalman", &RPY_kalman);

geometry_msgs::Vector3 RPY_acc;
ros::Publisher RPY_pub_acc("RPY_acc", &RPY_acc);

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
}

void init_ros()
{
    duties.data = 0.0;
    RPY_raw.x = 0.0;
    RPY_raw.y = 0.0;
    RPY_raw.z = 0.0;
    RPY_acc.x = 0.0;
    RPY_acc.y = 0.0;
    RPY_acc.z = 0.0;
    nh.initNode();
    nh.advertise(duties_pub);
    nh.advertise(debugger);
    nh.advertise(acc_gyro);
    nh.advertise(RPY_pub_kalman);
    nh.advertise(RPY_pub_raw);
    nh.advertise(RPY_pub_acc);
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
    timer.attach(&update_pose, PERIOD);
    while(1)
    {
        // get_acc_gyro();
        // update_motor_rotation();
        // publish_string("loop!");
        // publish_acc_gyro();
        // update_pose();
        RPY_pub_kalman.publish(&RPY_kalman);
        RPY_pub_raw.publish(&RPY_raw_deg);
        RPY_pub_acc.publish(&RPY_acc);
        publish_acc_gyro();
        nh.spinOnce();
        wait(PERIOD);
        led2 = !led2;
    }
    return 0;
}

void update_pose()
{
    get_acc_gyro();
    pose_from_acc();
    RPY_kalman = ex_kalman_filter.get_corrected(linear_acc, angular_vel);
    rad_to_deg(RPY_kalman, RPY_kalman);
    update_pose_without_kalman();
    rad_to_deg(RPY_raw, RPY_raw_deg);
}

void rad_to_deg(const geometry_msgs::Vector3& radian, geometry_msgs::Vector3& degree)
{
    degree.x = (radian.x * 180) / 3.1415;
    degree.y = (radian.y * 180) / 3.1415;
    degree.z = (radian.z * 180) / 3.1415; 
}

void update_pose_without_kalman()
{
    RPY_raw.x += PERIOD * (angular_vel.x + std::sin(RPY_raw.x) * std::tan(RPY_raw.y) * angular_vel.y + std::cos(RPY_raw.x) * std::tan(RPY_raw.y) * angular_vel.z);
    RPY_raw.y += PERIOD * (std::cos(RPY_raw.x) * angular_vel.y - std::sin(RPY_raw.x) * angular_vel.z);
    RPY_raw.z += PERIOD * ((std::sin(RPY_raw.x) * angular_vel.y) / (std::cos(RPY_raw.y)) + (std::cos(RPY_raw.x) * angular_vel.z)/(std::cos(RPY_raw.y)));
}

void pose_from_acc()
{
    RPY_acc.x = (atan2(linear_acc.y, linear_acc.z) * 180) / 3.1415;
    RPY_acc.y = (-1 * atan2(linear_acc.x, sqrt(pow(linear_acc.y, 2) + pow(linear_acc.z, 2))) * 180) / 3.1415;
    RPY_acc.z = 4545.0;
}