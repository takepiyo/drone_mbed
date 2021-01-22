#include "mbed.h"

#include <BMI088.h>
#include <Esc.h>
#include <ExtendedKalmanFilter.h>
#include <bmm150.h>
#include "bmm150_defs.h"

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
Ekf ex_kalman_filter(PERIOD);

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
geometry_msgs::Vector3 magne;
ros::Publisher magne_pub("magne", &magne);

geometry_msgs::Vector3 rotate_magne;
ros::Publisher rotate_magne_pub("rotate_magne", &rotate_magne);

geometry_msgs::Vector3 magne_offset;
ros::Publisher magne_offset_pub("magne_offset", &magne_offset);

geometry_msgs::Vector3 test;
ros::Publisher test_pub("test", &test);

geometry_msgs::Quaternion quat;
ros::Publisher quat_pub("quat", &quat);

geometry_msgs::Quaternion pred_quat;
ros::Publisher pred_quat_pub("pred_quat", &pred_quat);

geometry_msgs::Vector3 rpy;
ros::Publisher rpy_pub("RPY_kalman_rad", &rpy);

// void publish_string(string message)
// {
//     int length = message.length();
//     char char_array[length + 1];
//     strcpy(char_array, message.c_str());
//     echo.data = char_array;
//     debugger.publish(&echo);
// }

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
  nh.advertise(acc_pub);
  nh.advertise(gyro_pub);
  // nh.advertise(magne_pub);
  // nh.advertise(rotate_magne_pub);
  // nh.advertise(magne_offset_pub);
  nh.advertise(quat_pub);
  nh.advertise(pred_quat_pub);
  nh.advertise(rpy_pub);
  // nh.advertise(test_pub);
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
    // update_motor_rotation();
    acc_pub.publish(&acc);
    gyro_pub.publish(&gyro);
    // magne_pub.publish(&magne);
    // rotate_magne_pub.publish(&rotate_magne);
    // magne_offset_pub.publish(&magne_offset);
    quat_pub.publish(&quat);
    pred_quat_pub.publish(&pred_quat);
    rpy_pub.publish(&rpy);
    // test_pub.publish(&test);
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

  quat = ex_kalman_filter.get_compensation_state(acc, gyro, magne);
  // rad_to_deg(RPY_kalman_rad, RPY_kalman_deg);
}

void rad_to_deg(const geometry_msgs::Vector3 &radian,
                geometry_msgs::Vector3 &degree) {
  degree.x = (radian.x * 180) / 3.1415;
  degree.y = (radian.y * 180) / 3.1415;
  degree.z = (radian.z * 180) / 3.1415;
}