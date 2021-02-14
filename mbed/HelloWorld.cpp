#include "mbed.h"

#include <BMI088.h>
#include <Esc.h>
#include <bmm150.h>
#include "bmm150_defs.h"

#include <MadgwickFilter.hpp>

#include <ros.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include <bits/stdc++.h>
#include <vector>

#define MOTOR_NUM 4
#define PERIOD 0.01
#define DO_CALIB 0

void update_pose();

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p22, p23};

BMM150 bmm150(p9, p10);
BMI088 bmi088(p9, p10, PERIOD);
MadgwickFilter madgwickfilter;

Ticker timer;

// ros variables
ros::NodeHandle nh;
std_msgs::Float32 duties;
ros::Publisher duties_pub("now_duty", &duties);

geometry_msgs::Vector3 acc;
ros::Publisher acc_pub("acc", &acc);
geometry_msgs::Vector3 gyro;
ros::Publisher gyro_pub("gyro", &gyro);
geometry_msgs::Vector3 mag;
ros::Publisher mag_pub("mag", &mag);

geometry_msgs::Quaternion quat;
ros::Publisher quat_pub("quat", &quat);

void update_duties(const std_msgs::Float32 &input_duties) {
  duties = input_duties;
  led4   = !led4;
}

void update_motor_rotation() {
  motor[0].update(duties.data);
  motor[1].update(duties.data);
  motor[2].update(duties.data);
  motor[3].update(duties.data);
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
}

void init_ros() {
  duties.data = 0.0;

  nh.initNode();
  nh.advertise(quat_pub);
  nh.subscribe(duties_sub);
}

int main() {
  led1 = 0;
  init_ros();
  init_mbed();
  led1 = 1;
  timer.attach(&update_pose, PERIOD);
  while (1) {
    __disable_irq();  // 禁止
    quat_pub.publish(&quat);
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

  madgwickfilter.MadgwickAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);
  madgwickfilter.getAttitude(&quat.w, &quat.x, &quat.y, &quat.z);

  update_motor_rotation();
}