#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <ros.h>

Ekf::Ekf(double delta_t) {
  this->_delta_t = delta_t;

  _covariance_p = Eigen::Matrix4Xd::Zero(4, 4);
  _covariance_r = Eigen::Matrix3Xd::Zero(3, 3);
  _covariance_q = Eigen::Matrix4Xd::Zero(4, 4);

  _covariance_q(0)  = 1.00E-6;
  _covariance_q(5)  = 1.00E-6;
  _covariance_q(10) = 1.00E-6;
  _covariance_q(15) = 1.00E-6;

  _covariance_r(0) = 2.0;
  _covariance_r(4) = 2.0;
  _covariance_r(8) = 2.0;

  _covariance_q(0)  = 0.5;
  _covariance_q(5)  = 0.5;
  _covariance_q(10) = 0.5;
  _covariance_q(15) = 0.5;

  grav_acc = 9.79;

  _state_value << 1.0, 0.0, 0.0, 0.0;
}

Ekf::~Ekf() {}

geometry_msgs::Quaternion Ekf::get_compensation_state(
  const geometry_msgs::Vector3& linear_acc,
  const geometry_msgs::Vector3& angular_vel,
  const geometry_msgs::Vector3& geomagnetism) {
  // clang-format off
  Matrix<double, 3, 1> _gyro_sense;
  _gyro_sense << angular_vel.x,
                 angular_vel.y,
                 angular_vel.z;

  Matrix<double, 3, 1> _acc_geo_sense;
  _acc_geo_sense << linear_acc.x,
                    linear_acc.y,
                    linear_acc.z;
  // clang-format on
  Matrix<double, 4, 1> predicted_state        = _get_predicted_state(_gyro_sense);
  Matrix<double, 4, 4> predicted_jacobian_F   = _get_predicted_jacobian_F(_gyro_sense);
  Matrix<double, 4, 4> predicted_covariance_p = _get_predicted_covariance_p(predicted_jacobian_F);

  Matrix<double, 3, 4> observation_jacobian_H = _get_observation_jacobian_H(predicted_state);
  Matrix<double, 3, 1> observation_state      = _get_observation_state(predicted_state);
  Matrix<double, 4, 3> kalman_gain            = _get_kalman_gain(observation_jacobian_H, predicted_covariance_p);
  _update_covariance_p(kalman_gain, observation_jacobian_H, predicted_covariance_p);
  Matrix<double, 4, 1> com_quat = _get_compensation_state(predicted_state, kalman_gain, _acc_geo_sense, observation_state);

  _state_value = com_quat;

  double quat_norm = sqrt(com_quat(0) * com_quat(0) + com_quat(1) * com_quat(1) + com_quat(2) * com_quat(2) + com_quat(3) * com_quat(3));

  geometry_msgs::Quaternion _compensation_quaternion;
  _compensation_quaternion.w = com_quat(0) / quat_norm;
  _compensation_quaternion.x = com_quat(1) / quat_norm;
  _compensation_quaternion.y = com_quat(2) / quat_norm;
  _compensation_quaternion.z = com_quat(3) / quat_norm;

  return _compensation_quaternion;
}

geometry_msgs::Quaternion Ekf::get_predicted_state(
  const geometry_msgs::Vector3& linear_acc,
  const geometry_msgs::Vector3& angular_vel,
  const geometry_msgs::Vector3& geomagnetism) {
  // clang-format off
  Matrix<double, 3, 1> _gyro_sense;
  _gyro_sense << angular_vel.x,
                 angular_vel.y,
                 angular_vel.z;

  Matrix<double, 3, 1> _acc_geo_sense;
  _acc_geo_sense << linear_acc.x,
                    linear_acc.y,
                    linear_acc.z;
  // clang-format on
  Matrix<double, 4, 1> predicted_state = _get_predicted_state(_gyro_sense);

  _state_value = predicted_state;

  double quat_norm = sqrt(predicted_state(0) * predicted_state(0) + predicted_state(1) * predicted_state(1) + predicted_state(2) * predicted_state(2) + predicted_state(3) * predicted_state(3));

  geometry_msgs::Quaternion _compensation_quaternion;
  _compensation_quaternion.w = predicted_state(0) / quat_norm;
  _compensation_quaternion.x = predicted_state(1) / quat_norm;
  _compensation_quaternion.y = predicted_state(2) / quat_norm;
  _compensation_quaternion.z = predicted_state(3) / quat_norm;

  return _compensation_quaternion;
}

Matrix<double, 4, 1> Ekf::_get_predicted_state(const Matrix<double, 3, 1>& gyro_sense) {
  double omega_x_dt = 0.5 * _delta_t * gyro_sense(0);
  double omega_y_dt = 0.5 * _delta_t * gyro_sense(1);
  double omega_z_dt = 0.5 * _delta_t * gyro_sense(2);
  Matrix<double, 4, 4> f;
  f << 1, -omega_x_dt, -omega_y_dt, -omega_z_dt,
    omega_x_dt, 1, omega_z_dt, -omega_y_dt,
    omega_y_dt, -omega_z_dt, 1, omega_x_dt,
    omega_z_dt, omega_y_dt, -omega_x_dt, 1;

  Matrix<double, 4, 1> predicted_state;
  predicted_state = f * _state_value;
  return predicted_state;
}

Matrix<double, 4, 4> Ekf::_get_predicted_jacobian_F(const Matrix<double, 3, 1>& gyro_sense) {
  double omega_x_dt = 0.5 * _delta_t * gyro_sense(0);
  double omega_y_dt = 0.5 * _delta_t * gyro_sense(1);
  double omega_z_dt = 0.5 * _delta_t * gyro_sense(2);
  Matrix<double, 4, 4> predicted_jacobian_F;
  predicted_jacobian_F << 1, -omega_x_dt, -omega_y_dt, -omega_z_dt,
    omega_x_dt, 1, omega_z_dt, -omega_y_dt,
    omega_y_dt, -omega_z_dt, 1, omega_x_dt,
    omega_z_dt, omega_y_dt, -omega_x_dt, 1;
  return predicted_jacobian_F;
}

Matrix<double, 4, 4> Ekf::_get_predicted_covariance_p(const Matrix<double, 4, 4>& predicted_jacobian_F) {
  Matrix<double, 4, 4> predicted_covariance_p;
  predicted_covariance_p = predicted_jacobian_F * _covariance_p * predicted_jacobian_F.transpose() + _covariance_q;
  return predicted_covariance_p;
}

Matrix<double, 3, 4> Ekf::_get_observation_jacobian_H(const Matrix<double, 4, 1>& predicted_state) {
  Matrix<double, 3, 4> observation_jacobian_H;
  observation_jacobian_H << predicted_state(2) * 2.0 * grav_acc, predicted_state(3) * 2.0 * grav_acc, predicted_state(0) * 2.0 * grav_acc, predicted_state(1) * 2.0 * grav_acc,
    -predicted_state(1) * 2.0 * grav_acc, -predicted_state(0) * 2.0 * grav_acc, predicted_state(3) * 2.0 * grav_acc, predicted_state(2) * 2.0 * grav_acc,
    predicted_state(0) * 2.0 * grav_acc, -predicted_state(1) * 2.0 * grav_acc, -predicted_state(2) * 2.0 * grav_acc, predicted_state(3) * 2.0 * grav_acc;

  return observation_jacobian_H;
}

Matrix<double, 3, 1> Ekf::_get_observation_state(const Matrix<double, 4, 1>& predicted_state) {
  Matrix<double, 3, 1> observation_state;
  // clang-format off
  observation_state << 2 * (predicted_state(1) * predicted_state(3) + predicted_state(0) * predicted_state(2)) * grav_acc,
                       2 * (predicted_state(2) * predicted_state(3) - predicted_state(0) * predicted_state(1)) * grav_acc,
                       (predicted_state(0) * predicted_state(0) - predicted_state(1) * predicted_state(1) - predicted_state(2) * predicted_state(2) + predicted_state(3) * predicted_state(3)) * grav_acc;
  // clang-format on
  return observation_state;
}

Matrix<double, 4, 3> Ekf::_get_kalman_gain(const Matrix<double, 3, 4>& observation_jacobian_H,
                                           const Matrix<double, 4, 4>& predicted_covariance_p) {
  Matrix<double, 4, 3> kalman_gain;
  kalman_gain = predicted_covariance_p * observation_jacobian_H.transpose() * (observation_jacobian_H * predicted_covariance_p * observation_jacobian_H.transpose() + _covariance_r).inverse();
  return kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 4, 3>& kalman_gain,
                               const Matrix<double, 3, 4>& observation_jacobian_H,
                               const Matrix<double, 4, 4>& predicted_covariance_p) {
  _covariance_p = (Matrix<double, 4, 4>::Identity() - kalman_gain * observation_jacobian_H) * predicted_covariance_p;
}

Matrix<double, 4, 1> Ekf::_get_compensation_state(const Matrix<double, 4, 1>& predicted_state,
                                                  const Matrix<double, 4, 3>& kalman_gain,
                                                  const Matrix<double, 3, 1>& acc_geo_sense,
                                                  const Matrix<double, 3, 1>& observation_state) {
  Matrix<double, 4, 1> compensation_state;
  compensation_state = predicted_state + kalman_gain * (acc_geo_sense - observation_state);
  return compensation_state;
}