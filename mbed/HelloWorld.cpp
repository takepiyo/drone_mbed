#include "mbed.h"

#include <BMI088.h>
#include <Esc.h>
// #include <ExtendedKalmanFilter.h>
#include <bmm150.h>
#include "bmm150_defs.h"

#include <MadgwickFilter.hpp>

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <bits/stdc++.h>
#include <vector>

#define MOTOR_NUM 4
#define PERIOD 0.01
#define DO_CALIB 0

void update_pose();
void rad_to_deg(const geometry_msgs::Vector3 &radian,
                geometry_msgs::Vector3 &degree);
void get_dealed_roll_pitch_magne(geometry_msgs::Vector3 &accel);

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p22, p23};

BMM150 bmm150(p9, p10);
BMI088 bmi088(p9, p10, PERIOD);
// Ekf ex_kalman_filter(PERIOD);
MadgwickFilter madgwickfilter;

Ticker timer;

// ros variables
ros::NodeHandle nh;
// std_msgs::Float32MultiArray duties;
std_msgs::Float32 duties;
ros::Publisher duties_pub("now_duty", &duties);

geometry_msgs::Vector3 acc;
ros::Publisher acc_pub("acc", &acc);
geometry_msgs::Vector3 gyro;
ros::Publisher gyro_pub("gyro", &gyro);
geometry_msgs::Vector3 mag;
ros::Publisher mag_pub("mag", &mag);

geometry_msgs::Vector3 rotate_magne;
ros::Publisher rotate_magne_pub("rotate_magne", &rotate_magne);

geometry_msgs::Vector3 magne_offset;
ros::Publisher magne_offset_pub("magne_offset", &magne_offset);

geometry_msgs::Vector3 RPY_kalman_deg;
ros::Publisher RPY_pub_kalman_deg("RPY_kalman_deg", &RPY_kalman_deg);

geometry_msgs::Vector3 RPY_kalman_rad;
ros::Publisher RPY_pub_kalman_rad("RPY_kalman_rad", &RPY_kalman_rad);

geometry_msgs::Vector3 no_filter_pred;
ros::Publisher no_filter_pred_pub("no_filter_pred", &no_filter_pred);

geometry_msgs::Vector3 no_filter_obse;
ros::Publisher no_filter_obse_pub("no_filter_obse", &no_filter_obse);

geometry_msgs::Quaternion quat;
ros::Publisher quat_pub("quat", &quat);

geometry_msgs::Vector3 eular;
ros::Publisher eular_pub("eular", &eular);

void update_duties(const std_msgs::Float32 &input_duties) {
  duties = input_duties;
  led4   = !led4;
}

void update_motor_rotation() {
  motor[0].update(duties.data);
  motor[1].update(duties.data);
  motor[2].update(duties.data);
  motor[3].update(duties.data);
  // motor.update(duties.data);
  duties_pub.publish(&duties);
}
ros::Subscriber<std_msgs::Float32> duties_sub("input_duties", &update_duties);

void init_mbed() {
  led3 = 0;
  led4 = 0;
  while (1) {
    if (bmm150.initialize() == BMM150_E_ID_NOT_CONFORM) {
      led4 = !led4;
      wait_ms(200);
    } else {
      led4 = 1;
      if (DO_CALIB) {
        wait_ms(1000);
        led4 = 0;
        for (int i = 0; i < 10; i++) {
          led4 = !led4;
          wait_ms(10);
        }
        led4 = 1;
        bmm150.calibration();
        led4 = 0;
        wait_ms(3000);
      }
      break;
    }
  }
  while (1) {
    if (bmi088.isConnection()) {
      bmi088.initialize();
      led3 = 1;
      break;
    }
    led3 = !led3;
    wait_ms(200);
  }
  led3 = 1;

  geometry_msgs::Vector3 init_yaw = bmm150.read_mag_data();
  // ex_kalman_filter.init_yaw(init_yaw);
}

void init_ros() {
  duties.data = 0.0;

  nh.initNode();
  // nh.advertise(duties_pub);
  // nh.advertise(debugger);
  // nh.advertise(acc_pub);
  // nh.advertise(gyro_pub);
  nh.advertise(mag_pub);
  // nh.advertise(rotate_magne_pub);
  // nh.advertise(magne_offset_pub);
  // nh.advertise(RPY_pub_kalman_deg);
  // nh.advertise(RPY_pub_kalman_rad);
  // nh.advertise(no_filter_pred_pub);
  // nh.advertise(no_filter_obse_pub);
  nh.advertise(quat_pub);
  nh.advertise(eular_pub);
  // nh.subscribe(duties_sub);
}

int main() {
  led1 = 0;
  init_ros();
  init_mbed();
  led1 = 1;
  // publish_stri ng("start loop!");
  timer.attach(&update_pose, PERIOD);
  while (1) {
    __disable_irq();  // 禁止
    mag_pub.publish(&mag);
    quat_pub.publish(&quat);
    test_pub.publish(&eular);
    nh.spinOnce();
    __enable_irq();  // 許可
    wait(PERIOD);
    led2 = !led2;
  }
  return 0;
}

void update_pose() {
  acc  = bmi088.getAcceleration();
  gyro = bmi088.getGyroscope();
  mag  = bmm150.read_mag_data();
  // magne_offset = bmm150.get_offset();

  madgwickfilter.MadgwickAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);
  madgwickfilter.getAttitude(&quat.w, &quat.x, &quat.y, &quat.z);
  madgwickfilter.getgetEulerAngle(&eular.x, &eular.y, &eular.z);
}