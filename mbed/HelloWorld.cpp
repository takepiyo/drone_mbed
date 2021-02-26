#include "mbed.h"

#include <PIDController.h>

#include <BMI088.h>
#include <Esc.h>
#include <bmm150.h>
#include "bmm150_defs.h"

#include "Eigen/Dense.h"

#include <MadgwickFilter.hpp>
#include "RangeFinder.h"

#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>

#define MOTOR_NUM 4
#define PERIOD 0.01
#define LIMIT_DUTY 0.5
#define IF_STABLE 0.001

//PIDController variables
const float control_period = 0.0769;

const PIDController::Param roll_controller_param = {
  .P = 4.5578,
  .I = 4.2896,
  .D = 0.16891,
};

const PIDController::Param pitch_controller_param = {
  .P = -4.4182,
  .I = -21.646,
  .D = -0.13788,
};

PIDController roll_pid_controller(control_period, roll_controller_param);
PIDController pitch_pid_controller(control_period, pitch_controller_param);

// function
void update_ref_z(const std_msgs::Float32 &ref_z);
void update_ref_roll(const std_msgs::Float32 &ref_roll);
void update_ref_pitch(const std_msgs::Float32 &ref_pitch);
void update_ref_yaw(const std_msgs::Float32 &ref_yaw);
double cap(double duty);
void update_motor_rotation();
// void mixing_duty();
void init_mbed();
void init_ros();
void update_control_input();
void unmixing_duty();
void update_control_loop();
void emergency_stop(const std_msgs::Empty &toggle);
void control_start(const std_msgs::Empty &toggle);
Eigen::Matrix<double, 4, 1> ros_to_eigen(geometry_msgs::Quaternion &input);
geometry_msgs::Quaternion eigen_to_ros(Eigen::Matrix<double, 4, 1> &input);

// check pose stable
bool stable = false;

// mbed variables
DigitalOut led1 = LED1;
DigitalOut led2 = LED2;
DigitalOut led3 = LED3;
DigitalOut led4 = LED4;

Esc motor[MOTOR_NUM] = {p25, p24, p22, p23};

// Eigen::Matrix<double, 4, 4> mix_duty_mat;
Eigen::Matrix<double, 4, 4> unmix_duty_mat;

BMM150 bmm150(p9, p10);
BMI088 bmi088(p9, p10, PERIOD);
MadgwickFilter madgwickfilter;
RangeFinder rf(p21, 10, 5800.0, 100000);
Ticker timer;

// ros variables
ros::NodeHandle nh;
geometry_msgs::Quaternion duty;  //  output duty  [w x y z] correspond motor [0 1 2 3]
ros::Publisher duty_pub("current_duty", &duty);
geometry_msgs::Quaternion ref_zrpy;                   //  reference:  [w x y z] correspond ref throttle roll pitch yaw
geometry_msgs::QuaternionStamped control_input_zrpy;  //  control amount:  [w x y z] correspond ref throttle roll pitch yaw
ros::Publisher control_input_pub("control_input", &control_input_zrpy);
ros::Subscriber<std_msgs::Float32> z_sub("ref_z", &update_ref_z);
ros::Subscriber<std_msgs::Float32> roll_sub("ref_roll", &update_ref_roll);
ros::Subscriber<std_msgs::Float32> pitch_sub("ref_pitch", &update_ref_pitch);
ros::Subscriber<std_msgs::Float32> yaw_sub("ref_yaw", &update_ref_yaw);
ros::Subscriber<std_msgs::Empty> emergency_stop_sub("emergency_stop", &emergency_stop);
ros::Subscriber<std_msgs::Empty> check_stable_sub("stable", &control_start);

geometry_msgs::Vector3 acc;
ros::Publisher acc_pub("acc", &acc);
geometry_msgs::Vector3 gyro;
ros::Publisher gyro_pub("gyro", &gyro);
geometry_msgs::Vector3 mag;
ros::Publisher mag_pub("mag", &mag);

geometry_msgs::Vector3Stamped eular;
ros::Publisher eular_pub("pose", &eular);

// geometry_msgs::QuaternionStamped mixed_duty;  // [z roll pitch yaw] depend [w x y z]
// ros::Publisher mixed_duty_pub("mixed_duty", &mixed_duty);

void update_ref_z(const std_msgs::Float32 &ref_z) {
  ref_zrpy.w = ref_z.data;
  led4       = 0;
}

void update_ref_roll(const std_msgs::Float32 &ref_roll) {
  ref_zrpy.x = ref_roll.data;
  led4       = 1;
}

void update_ref_pitch(const std_msgs::Float32 &ref_pitch) {
  ref_zrpy.y = ref_pitch.data;
  led3       = 0;
}

void update_ref_yaw(const std_msgs::Float32 &ref_yaw) {
  ref_zrpy.z = ref_yaw.data;
  led3       = 1;
}

void update_control_input() {
  control_input_zrpy.quaternion.w = ref_zrpy.w;
  control_input_zrpy.quaternion.x = roll_pid_controller.update(ref_zrpy.x, eular.vector.x);
  control_input_zrpy.quaternion.y = pitch_pid_controller.update(ref_zrpy.y, eular.vector.y);
  control_input_zrpy.quaternion.z = ref_zrpy.z;
}

void update_motor_rotation() {
  unmixing_duty();
  control_input_zrpy.header.stamp = nh.now();
  motor[0].update(cap(duty.w));
  motor[1].update(cap(duty.x));
  motor[2].update(cap(duty.y));
  motor[3].update(cap(duty.z));
  // mixing_duty();
}

// void mixing_duty() {
//   mixed_duty.header.stamp = nh.now();

//   Eigen::Matrix<double, 4, 1> mixed = mix_duty_mat * ros_to_eigen(duty);
//   mixed_duty.quaternion             = eigen_to_ros(mixed);
// }

void unmixing_duty() {
  Eigen::Matrix<double, 4, 1> input_duty = unmix_duty_mat * ros_to_eigen(control_input_zrpy.quaternion);

  duty = eigen_to_ros(input_duty);
}

double cap(double duty) {
  return std::min(duty, LIMIT_DUTY);
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
  // mix_duty_mat << 1.0, 1.0, 1.0, 1.0,
  //                 -1.0, 1.0, 1.0, -1.0,
  //                 -1.0, -1.0, 1.0, 1.0,
  //                 -1.0, 1.0, -1.0, 1.0;
  unmix_duty_mat << 0.25, -0.25, -0.25, -0.25,
                    0.25, 0.25, -0.25, 0.25,
                    0.25, 0.25, 0.25, -0.25,
                    0.25, -0.25, 0.25, 0.25;
  // clang-format on
}

void init_ros() {
  nh.initNode();
  nh.advertise(eular_pub);
  nh.advertise(duty_pub);
  nh.advertise(control_input_pub);
  nh.subscribe(z_sub);
  nh.subscribe(roll_sub);
  nh.subscribe(pitch_sub);
  nh.subscribe(yaw_sub);
  nh.subscribe(emergency_stop_sub);
  nh.subscribe(check_stable_sub);
}

int main() {
  led1 = 0;
  init_ros();
  init_mbed();
  timer.attach(&update_control_loop, PERIOD);
  while (1) {
    __disable_irq();  // 禁止
    duty_pub.publish(&duty);
    control_input_pub.publish(&control_input_zrpy);
    eular_pub.publish(&eular);
    nh.spinOnce();
    __enable_irq();  // 許可
    wait(PERIOD);
    led2 = !led2;
  }
  return 0;
}

void update_control_loop() {
  acc  = bmi088.getAcceleration();
  gyro = bmi088.getGyroscope();
  mag  = bmm150.read_mag_data();

  madgwickfilter.MadgwickAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z);
  madgwickfilter.getEulerAngle(&eular.vector.x, &eular.vector.y, &eular.vector.z);
  eular.header.stamp = nh.now();
  if (stable) {
    update_control_input();
    update_motor_rotation();
  }
}

void emergency_stop(const std_msgs::Empty &toggle) {
  timer.detach();
  motor[0].update(0.0);
  motor[1].update(0.0);
  motor[2].update(0.0);
  motor[3].update(0.0);
  led1 = 0;
}

void control_start(const std_msgs::Empty &toggle) {
  stable = true;
  led1   = 1;
}