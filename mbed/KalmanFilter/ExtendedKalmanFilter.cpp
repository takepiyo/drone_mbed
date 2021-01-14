#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <ros.h>

Ekf::Ekf(double delta_t) {
  this->_delta_t = delta_t;

  _covariance_q(0) = 1.74E-2 * this->_delta_t * this->_delta_t;
  _covariance_q(4) = 1.74E-2 * this->_delta_t * this->_delta_t;
  _covariance_q(8) = 1.74E-2 * this->_delta_t * this->_delta_t;

  _covariance_r(0)  = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_r(7)  = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_r(14) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_r(21) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_r(28) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_r(35) = 1.0 * this->_delta_t * this->_delta_t;

  _covariance_p(0)  = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(8)  = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(16) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(24) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(32) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(40) = 1.0 * this->_delta_t * this->_delta_t;
  _covariance_p(48) = 1.0 * this->_delta_t * this->_delta_t;

  _beta(0) = 0.0033;
  _beta(4) = 0.0033;
  _beta(8) = 0.0033;

  _G(12) = 1.0 * this->_delta_t * this->_delta_t;
  _G(16) = 1.0 * this->_delta_t * this->_delta_t;
  _G(20) = 1.0 * this->_delta_t * this->_delta_t;

  grav_acc = 9.79;
  mag_n    = 1.0;
  mag_d    = 1.0;

  _state_value << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // TODO init yaw quaternion
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

  Matrix<double, 6, 1> _acc_geo_sense;
  _acc_geo_sense << linear_acc.x,
                    linear_acc.y,
                    linear_acc.z,
                    geomagnetism.x,
                    geomagnetism.y,
                    geomagnetism.z;
  // clang-format on
  Matrix<double, 7, 1> predicted_state        = _get_predicted_state(_gyro_sense);
  Matrix<double, 7, 7> predicted_jacobian_F   = _get_predicted_jacobian_F(_gyro_sense);
  Matrix<double, 7, 7> predicted_covariance_p = _get_predicted_covariance_p(predicted_jacobian_F);

  Matrix<double, 6, 7> observation_jacobian_H = _get_observation_jacobian_H(predicted_state);
  Matrix<double, 6, 1> observation_state      = _get_observation_state(predicted_state);
  Matrix<double, 7, 6> kalman_gain            = _get_kalman_gain(observation_jacobian_H, predicted_covariance_p);
  _update_covariance_p(kalman_gain, observation_jacobian_H, predicted_covariance_p);
  Matrix<double, 7, 1> compensation_state = _get_compensation_state(predicted_state, kalman_gain, _acc_geo_sense, observation_state);

  geometry_msgs::Quaternion _compensation_quaternion;
  _compensation_quaternion.w = compensation_state(0);
  _compensation_quaternion.x = compensation_state(1);
  _compensation_quaternion.y = compensation_state(2);
  _compensation_quaternion.z = compensation_state(3);

  return _compensation_quaternion;
}

Matrix<double, 7, 1> Ekf::_get_predicted_state(const Matrix<double, 3, 1>& gyro_sense) {
  double q_0     = _state_value(0);
  double q_1     = _state_value(1);
  double q_2     = _state_value(2);
  double q_3     = _state_value(3);
  double b_x     = _state_value(4);
  double b_y     = _state_value(5);
  double b_z     = _state_value(6);
  double omega_x = gyro_sense(0) - b_x;
  double omega_y = gyro_sense(1) - b_y;
  double omega_z = gyro_sense(2) - b_z;
  Matrix<double, 7, 1> predicted_state;
  predicted_state(0) = q_0 + 0.5 * _delta_t * (-omega_x * q_1 - omega_y * q_2 - omega_z * q_3);
  predicted_state(1) = q_1 + 0.5 * _delta_t * (omega_x * q_0 - omega_y * q_3 + omega_z * q_2);
  predicted_state(2) = q_2 + 0.5 * _delta_t * (omega_x * q_3 + omega_y * q_0 - omega_z * q_1);
  predicted_state(3) = q_3 + 0.5 * _delta_t * (-omega_x * q_2 + omega_y * q_1 + omega_z * q_0);
  predicted_state(4) = -_beta(0) * _delta_t * b_x;
  predicted_state(5) = -_beta(4) * _delta_t * b_y;
  predicted_state(6) = -_beta(8) * _delta_t * b_z;
  return predicted_state;
}

Matrix<double, 7, 7> Ekf::_get_predicted_jacobian_F(const Matrix<double, 3, 1>& gyro_sense) {
  double q_0     = _state_value(0);
  double q_1     = _state_value(1);
  double q_2     = _state_value(2);
  double q_3     = _state_value(3);
  double b_x     = _state_value(4);
  double b_y     = _state_value(5);
  double b_z     = _state_value(6);
  double omega_x = gyro_sense(0) - b_x;
  double omega_y = gyro_sense(1) - b_y;
  double omega_z = gyro_sense(2) - b_z;
  // clang-format off
  Matrix<double, 7, 7> predicted_jacobian_F;
  predicted_jacobian_F <<
  1.0, -0.5 * _delta_t * omega_x, -0.5 * _delta_t * omega_y, -0.5 * _delta_t * omega_z, 0.5 * _delta_t * q_1, 0.5 * _delta_t * q_2, 0.5 * _delta_t * q_3,
  0.5 * _delta_t * omega_x, 1.0, 0.5 * _delta_t * omega_z, -0.5 * _delta_t * omega_y, -0.5 * _delta_t * q_0, 0.5 * _delta_t * q_3, -0.5 * _delta_t * q_2,
  0.5 * _delta_t * omega_y, -0.5 * _delta_t * omega_z, 1.0, 0.5 * _delta_t * omega_x, -0.5 * _delta_t * q_3, -0.5 * _delta_t * q_0, 0.5 * _delta_t * q_1,
  0.5 * _delta_t * omega_z, 0.5 * _delta_t * omega_y, -0.5 * _delta_t * omega_x, 1.0, 0.5 * _delta_t * q_2, -0.5 * _delta_t * q_1, -0.5 * _delta_t * q_0,
  0.0, 0.0, 0.0, 0.0, -_beta(0) * _delta_t, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, -_beta(4) * _delta_t, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -_beta(8) * _delta_t;
  // clang-format on
  return predicted_jacobian_F;
}

Matrix<double, 7, 7> Ekf::_get_predicted_covariance_p(const Matrix<double, 7, 7>& predicted_jacobian_F) {
  Matrix<double, 7, 7> predicted_covariance_p;
  predicted_covariance_p = predicted_jacobian_F * _covariance_p * predicted_jacobian_F.transpose() + _G * _covariance_q * _G.transpose();
  return predicted_covariance_p;
}

Matrix<double, 6, 7> Ekf::_get_observation_jacobian_H(const Matrix<double, 7, 1>& predicted_state) {
  double q_0 = predicted_state(0);
  double q_1 = predicted_state(1);
  double q_2 = predicted_state(2);
  double q_3 = predicted_state(3);
  // clang-format off
  Matrix<double, 6, 7> observation_jacobian_H;
  observation_jacobian_H <<
  -2.0 * grav_acc * q_2, 2.0 * grav_acc * q_3, -2.0 * grav_acc *q_0, 2.0 * grav_acc * q_1, 0.0, 0.0, 0.0,
  2.0 * grav_acc * q_1, 2.0 * grav_acc * q_0, 2.0 * grav_acc * q_3, 2.0 * grav_acc * q_2, 0.0, 0.0, 0.0,
  2.0 * grav_acc * q_0, -2.0 * grav_acc * q_1, -2.0 * grav_acc * q_2, 2.0 * grav_acc * q_3, 0.0, 0.0, 0.0,
  2.0 * (q_0 * mag_n - q_2 * mag_d),  2.0 * (q_1 * mag_n + q_3 * mag_d), 2.0 * (-q_2 * mag_n - q_0 * mag_d), 2.0 * (-q_3 * mag_n + q_1 * mag_d), 0.0, 0.0, 0.0,
  2.0 * (-q_3 * mag_n + q_1 * mag_d), 2.0 * (q_2 * mag_n + q_0 * mag_d), 2.0 * (q_1 * mag_n + q_3 * mag_d),  2.0 * (-q_0 * mag_n + q_2 * mag_d), 0.0, 0.0, 0.0,
  2.0 * (q_2 * mag_n + q_0 * mag_d),  2.0 * (q_3 * mag_n - q_1 * mag_d), 2.0 * (q_0 * mag_n - q_2 * mag_d),  2.0 * (q_1 * mag_n + q_3 * mag_d),  0.0, 0.0, 0.0;
  // clang-format on
  return observation_jacobian_H;
}

Matrix<double, 6, 1> Ekf::_get_observation_state(const Matrix<double, 7, 1>& predicted_state) {
  double q_0 = predicted_state(0);
  double q_1 = predicted_state(1);
  double q_2 = predicted_state(2);
  double q_3 = predicted_state(3);
  Matrix<double, 6, 1> observation_state;
  // clang-format off
  observation_state << 2 * (q_1 * q_3 - q_0 * q_2) * grav_acc,
                       2 * (q_2 * q_3 + q_0 * q_1) * grav_acc,
                       (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3) * grav_acc,
                       (q_0 * q_0 + q_1 * q_1 - q_2 * q_2 - q_3 * q_3) * mag_n + 2 * (q_1 * q_3 - q_0 * q_2) * mag_d,
                       2 * (q_1 * q_2 - q_0 * q_3) * mag_n + 2 * (q_2 * q_3 + q_0 * q_1) * mag_d,
                       2 * (q_1 * q_3 + q_0 * q_2) * mag_n + (q_0 * q_0 - q_1 * q_1 - q_2 * q_2 + q_3 * q_3) * mag_d;
  // clang-format on
  return observation_state;
}

Matrix<double, 7, 6> Ekf::_get_kalman_gain(const Matrix<double, 6, 7>& observation_jacobian_H,
                                           const Matrix<double, 7, 7>& predicted_covariance_p) {
  Matrix<double, 7, 6> kalman_gain;
  kalman_gain = predicted_covariance_p * observation_jacobian_H.transpose() * (observation_jacobian_H * predicted_covariance_p * observation_jacobian_H.transpose() + _covariance_r).inverse();
  return kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 7, 6>& kalman_gain,
                               const Matrix<double, 6, 7>& observation_jacobian_H,
                               const Matrix<double, 7, 7>& predicted_covariance_p) {
  _covariance_p = predicted_covariance_p - kalman_gain * observation_jacobian_H * predicted_covariance_p;
}

Matrix<double, 7, 1> Ekf::_get_compensation_state(const Matrix<double, 7, 1>& predicted_state,
                                                  const Matrix<double, 7, 6>& kalman_gain,
                                                  const Matrix<double, 6, 1>& acc_geo_sense,
                                                  const Matrix<double, 6, 1>& observation_state) {
  Matrix<double, 7, 1> compensation_state;
  compensation_state = predicted_state + kalman_gain * (acc_geo_sense - observation_state);
  return compensation_state;
}