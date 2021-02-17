#include "mbed.h"

#include <BMI088.h>
#include <Esc.h>
#include <bmm150.h>
#include "bmm150_defs.h"

#include "Eigen/Dense.h"

#include <MadgwickFilter.hpp>
#include "RangeFinder.h"

#include <ros.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>

#define MOTOR_NUM 4
#define PERIOD 0.01
#define DO_CALIB 0

void update_duty(const std_msgs::Float32 &input_duty);
void update_motor_rotation();
void mixing_duty();
void init_mbed();
void init_ros();
void update_pose();
Eigen::Matrix<double, 4, 1> ros_to_eigen(geometry_msgs::Quaternion &input);
geometry_msgs::Quaternion eigen_to_ros(Eigen::Matrix<double, 4, 1> &input);

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p22, p23};

Eigen::Matrix<double, 4, 4> mix_duty_mat;

BMM150 bmm150(p9, p10);
BMI088 bmi088(p9, p10, PERIOD);
MadgwickFilter madgwickfilter;
RangeFinder rf(p21, 10, 5800.0, 100000);
Ticker timer;

// ros variables
ros::NodeHandle nh;
geometry_msgs::Quaternion duty;  //  output duty  [w x y z] correspond motor [0 1 2 3]
ros::Publisher duty_pub("corrent_duty", &duty);
ros::Subscriber<std_msgs::Float32> duty_sub("input_duty", &update_duty);

geometry_msgs::Vector3 acc;
ros::Publisher acc_pub("acc", &acc);
geometry_msgs::Vector3 gyro;
ros::Publisher gyro_pub("gyro", &gyro);
geometry_msgs::Vector3 mag;
ros::Publisher mag_pub("mag", &mag);

geometry_msgs::PoseStamped pose_stamped;
ros::Publisher pose_pub("pose", &pose_stamped);

geometry_msgs::QuaternionStamped mixed_duty;  // [z roll pitch yaw] depend [w x y z]
ros::Publisher mixed_duty_pub("mixed_duty", &mixed_duty);

void update_duty(const std_msgs::Float32 &input_duty) {
  duty.w = input_duty.data;
  duty.x = input_duty.data;
  duty.y = input_duty.data;
  duty.z = input_duty.data;
  led4   = !led4;
}

void update_motor_rotation() {
  motor[0].update(duty.w);
  motor[1].update(duty.x);
  motor[2].update(duty.y);
  motor[3].update(duty.z);
  mixing_duty();
}

void mixing_duty() {
  mixed_duty.header.stamp = nh.now();

  Eigen::Matrix<double, 4, 1> mixed = mix_duty_mat * ros_to_eigen(duty);
  mixed_duty.quaternion             = eigen_to_ros(mixed);
}

Eigen::Matrix<double, 4, 1> ros_to_eigen(geometry_msgs::Quaternion &input) {
  Eigen::Matrix<double, 4, 1> output;
  output << input.w, input.x, input.y, input.z;
  return output;
}

geometry_msgs::Quaternion eigen_to_ros(Eigen::Matrix<double, 4, 1> &input) {
  geometry_msgs::Quaternion output;
  output.w = input(0);
  output.x = input(1);
  output.y = input(2);
  output.z = input(3);
  return output;
}

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
  // clang-format off
  mix_duty_mat << 1.0, 1.0, 1.0, 1.0,
                  -1.0, 1.0, 1.0, -1.0,
                  -1.0, -1.0, 1.0, 1.0,
                  -1.0, 1.0, -1.0, 1.0;
  // clang-format on
}

void init_ros() {
  nh.initNode();
  nh.advertise(pose_pub);
  nh.advertise(duty_pub);
  nh.advertise(mixed_duty_pub);
  nh.subscribe(duty_sub);
}

int main() {
  led1 = 0;
  init_ros();
  init_mbed();
  led1 = 1;
  timer.attach(&update_pose, PERIOD);
  while (1) {
    __disable_irq();  // 禁止
    duty_pub.publish(&duty);
    mixed_duty_pub.publish(&mixed_duty);
    pose_pub.publish(&pose_stamped);
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

  pose_stamped.pose.position.z = rf.read_m();
  madgwickfilter.MadgwickAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);
  madgwickfilter.getAttitude(&pose_stamped.pose.orientation.w, &pose_stamped.pose.orientation.x, &pose_stamped.pose.orientation.y, &pose_stamped.pose.orientation.z);
  pose_stamped.header.stamp = nh.now();
  update_motor_rotation();
}