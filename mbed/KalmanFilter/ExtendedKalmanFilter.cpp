#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <ros.h>

#define PI 3.141592

Ekf::Ekf(double delta_t) {
  this->_delta_t                          = delta_t;
  this->_yaw_sign_reverse_count           = 0;
  this->_no_filter_yaw_sign_reverse_count = 0;
  // clang-format off
  this->_covariance_q << 1.74E-2*this->_delta_t*this->_delta_t, 0                                   , 0                                   ,
                         0                                   , 1.74E-2*this->_delta_t*this->_delta_t, 0                                   ,
                         0                                   , 0                                   , 1.74E-2*this->_delta_t*this->_delta_t;

  this->_covariance_r << 1.0 * this->_delta_t*this->_delta_t       , 0                                       , 0                                  ,
                         0                                         , 1.0 * this->_delta_t*this->_delta_t     , 0                                  ,
                         0                                         , 0                                       , 1.0 * this->_delta_t*this->_delta_t;
  this->_covariance_p = this->_covariance_q;

  this->_roll_pitch_yaw << 0.0,
                           0.0,
                           0.0;

  this->_no_filter_pred << 0.0,
                           0.0,
                           0.0;

  this->_no_filter_obse << 0.0,
                           0.0,
                           0.0;

  this->_observation_matrix_H << 1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0;
  // clang-format on
}

Ekf::~Ekf() {}

void Ekf::init_yaw(geometry_msgs::Vector3& geomagnetism) {
  double yaw               = atan2(-geomagnetism.y, geomagnetism.x);
  this->_roll_pitch_yaw(2) = yaw;
  this->_no_filter_pred(2) = yaw;
  this->_no_filter_obse(2) = yaw;
  this->_no_filter_yaw_raw = yaw;
}

geometry_msgs::Vector3 Ekf::get_corrected(
  const geometry_msgs::Vector3& linear_acc,
  const geometry_msgs::Vector3& angular_vel,
  const geometry_msgs::Vector3& geomagnetism) {
  // clang-format off
  this->_linear_acc << linear_acc.x,
                       linear_acc.y,
                       linear_acc.z;

  this->_angular_vel << angular_vel.x,
                        angular_vel.y,
                        angular_vel.z;

  this->_geomagnetism << geomagnetism.x,
                         geomagnetism.y,
                         geomagnetism.z;
  // clang-format on
  Matrix<double, 3, 1> pre_roll_pitch   = _predict_angle();
  Matrix<double, 3, 3> state_jacobian_F = _get_state_jacobian();
  Matrix<double, 3, 3> pre_covariance_p =
    _predict_covariance_p(state_jacobian_F);

  Matrix<double, 3, 1> actual_observation_angle =
    _get_actual_observation_angle();

  Matrix<double, 3, 3> kalman_gain = _get_kalman_gain(pre_covariance_p);
  _update_covariance_p(kalman_gain, pre_covariance_p);
  return _get_corrected_angle(pre_roll_pitch, actual_observation_angle,
                              kalman_gain);
}

geometry_msgs::Vector3 Ekf::get_predicted_value_no_filter() {
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_no_filter_pred);
  // clang-format off
  Matrix<double, 3, 3> A;
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            , -1.0 * tri(0, 0)     ,
       0.0, tri(0, 0) / tri(1, 1), tri(1, 0) / tri(1, 1);

  this->_no_filter_pred = this->_no_filter_pred + this->_delta_t * (A * (this->_angular_vel));
  // clang-format on
  geometry_msgs::Vector3 output;
  output.x = this->_no_filter_pred(0);
  output.y = this->_no_filter_pred(1);
  output.z = this->_no_filter_pred(2);
  return output;
}

geometry_msgs::Vector3 Ekf::get_observation_no_filter() {
  // clang-format off
  _no_filter_obse(0) = atan2(this->_linear_acc(1), this->_linear_acc(2));
  _no_filter_obse(1) = atan2(-1 * this->_linear_acc(0), hypot(this->_linear_acc(1), this->_linear_acc(2)));
  double previous_obse_yaw_raw = this->_no_filter_yaw_raw;
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_no_filter_obse);
  this->_no_filter_yaw_raw = atan2(-(tri(1, 0) * this->_geomagnetism(1) - tri(0, 0) * this->_geomagnetism(2)), (tri(1, 1) * this->_geomagnetism(0) + tri(0, 1) * tri(0, 0) * this->_geomagnetism(1) + tri(0, 1) * tri(1, 0) * this->_geomagnetism(2)));
  if (this->_no_filter_yaw_raw > 0 && previous_obse_yaw_raw < 0 && abs(this->_no_filter_yaw_raw - previous_obse_yaw_raw) > PI) { this->_no_filter_yaw_sign_reverse_count = this->_no_filter_yaw_sign_reverse_count - 1; }
  if (this->_no_filter_yaw_raw < 0 && previous_obse_yaw_raw > 0 && abs(this->_no_filter_yaw_raw - previous_obse_yaw_raw) > PI) { this->_no_filter_yaw_sign_reverse_count = this->_no_filter_yaw_sign_reverse_count + 1; }
  _no_filter_obse(2) =this->_no_filter_yaw_raw + 2 * PI * this->_no_filter_yaw_sign_reverse_count;

  this->no_filter_yaw_test.x = this->_no_filter_yaw_sign_reverse_count;
  this->no_filter_yaw_test.y = previous_obse_yaw_raw;
  this->no_filter_yaw_test.z = this->_no_filter_yaw_raw;

  // clang-format on
  geometry_msgs::Vector3 output;
  output.x = _no_filter_obse(0);
  output.y = _no_filter_obse(1);
  output.z = _no_filter_obse(2);
  return output;
}

Matrix<double, 3, 2> Ekf::_get_trigonometric(
  const Matrix<double, 3, 1>& roll_pitch_yaw) {
  Matrix<double, 3, 2> trigonometric;
  double roll  = roll_pitch_yaw(0);
  double pitch = roll_pitch_yaw(1);
  // clang-format off
  trigonometric << sin(roll), sin(pitch),
                   cos(roll), cos(pitch),
                   tan(roll), tan(pitch);
  // clang-format on
  return trigonometric;
}

Matrix<double, 3, 1> Ekf::_predict_angle() {
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_yaw);

  Matrix<double, 3, 3> A;
  // clang-format off
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            , -1.0 * tri(0, 0)     ,
       0.0, tri(0, 0) / tri(1, 1), tri(1, 0) / tri(1, 1);
  Matrix<double, 3, 1> pred_roll_pitch_yaw;
  pred_roll_pitch_yaw = this->_roll_pitch_yaw + this->_delta_t * (A * (this->_angular_vel));
  // clang-format on
  return pred_roll_pitch_yaw;
}

Matrix<double, 3, 3> Ekf::_get_state_jacobian() {
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_yaw);

  Matrix<double, 3, 3> state_jacobian_F;
  state_jacobian_F(0, 0) = 1 + this->_delta_t * (this->_angular_vel(1) * tri(1, 0) * tri(2, 1) - this->_angular_vel(2) * tri(0, 0) * tri(2, 1));
  state_jacobian_F(0, 1) = this->_delta_t * ((this->_angular_vel(1) * tri(0, 0)) / pow(tri(1, 1), 2) + (this->_angular_vel(2) * tri(1, 0)) / pow(tri(1, 1), 2));
  state_jacobian_F(0, 2) = 0.0;
  state_jacobian_F(1, 0) = -1 * this->_delta_t * (this->_angular_vel(1) * tri(0, 0) + this->_angular_vel(2) * tri(1, 0));
  state_jacobian_F(1, 1) = 1.0;
  state_jacobian_F(1, 2) = 0.0;
  state_jacobian_F(2, 0) = this->_delta_t * ((this->_angular_vel(1) * tri(1, 0)) / tri(1, 1));
  state_jacobian_F(2, 1) = this->_delta_t * ((_angular_vel(1) * tri(2, 1) * tri(0, 0)) / tri(1, 1) + (_angular_vel(2) * tri(2, 1) * tri(1, 0)) / tri(1, 1));
  state_jacobian_F(2, 2) = 1.0;
  return state_jacobian_F;
}

Matrix<double, 3, 3> Ekf::_predict_covariance_p(
  const Matrix<double, 3, 3>& state_jacobian_F) {
  Matrix<double, 3, 3> pre_covariance_p;
  pre_covariance_p = state_jacobian_F * this->_covariance_p * state_jacobian_F.transpose() + this->_covariance_q;
  return pre_covariance_p;
}

Matrix<double, 3, 1> Ekf::_get_actual_observation_angle() {
  Matrix<double, 3, 1> observation_angle;
  // clang-format off
  observation_angle << atan2(this->_linear_acc(1), this->_linear_acc(2)),
                       atan2(-1 * this->_linear_acc(0), hypot(this->_linear_acc(1), this->_linear_acc(2))),
                       0.0;
  double previous_obse_yaw_raw = this->_yaw_raw;
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(observation_angle);
  this->_yaw_raw = atan2(-(tri(1, 0) * this->_geomagnetism(1) - tri(0, 0) * this->_geomagnetism(2)), (tri(1, 1) * this->_geomagnetism(0) + tri(0, 1) * tri(0, 0) * this->_geomagnetism(1) + tri(0, 1) * tri(1, 0) * this->_geomagnetism(2)));
  // clang-format on
  if (this->_yaw_raw > 0 && previous_obse_yaw_raw < 0 && abs(this->_yaw_raw - previous_obse_yaw_raw) > PI) { this->_yaw_sign_reverse_count = this->_yaw_sign_reverse_count - 1; }
  if (this->_yaw_raw < 0 && previous_obse_yaw_raw > 0 && abs(this->_yaw_raw - previous_obse_yaw_raw) > PI) { this->_yaw_sign_reverse_count = this->_yaw_sign_reverse_count + 1; }
  observation_angle(2) = this->_yaw_raw + 2 * PI * this->_yaw_sign_reverse_count;
  return observation_angle;
}

Matrix<double, 3, 3> Ekf::_get_kalman_gain(
  const Matrix<double, 3, 3>& pre_covariance_p) {
  Matrix<double, 3, 3> _kalman_gain;
  _kalman_gain = pre_covariance_p * this->_observation_matrix_H.transpose() *
                 (this->_observation_matrix_H * pre_covariance_p * this->_observation_matrix_H.transpose() + this->_covariance_r).inverse();
  return _kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 3, 3>& kalman_gain,
                               const Matrix<double, 3, 3>& pre_covariance_p) {
  this->_covariance_p = (Matrix<double, 3, 3>::Identity() - kalman_gain * this->_observation_matrix_H) * pre_covariance_p;
}

geometry_msgs::Vector3 Ekf::_get_corrected_angle(
  const Matrix<double, 3, 1>& pre_roll_pitch_yaw,
  const Matrix<double, 3, 1>& actual_observation_angle,
  const Matrix<double, 3, 3>& kalman_gain) {
  geometry_msgs::Vector3 output;
  this->_roll_pitch_yaw = pre_roll_pitch_yaw + kalman_gain * (actual_observation_angle - this->_observation_matrix_H * pre_roll_pitch_yaw);

  output.x = this->_roll_pitch_yaw(0);
  output.y = this->_roll_pitch_yaw(1);
  output.z = this->_roll_pitch_yaw(2);
  return output;
}