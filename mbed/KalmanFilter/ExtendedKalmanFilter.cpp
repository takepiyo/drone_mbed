#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>

#define TBD 5.5915E-4

Ekf::Ekf(double delta_t)
{
  this->_delta_t = delta_t;

  this->_covariance_q << 5.5915E-4,         0,         0,
                                 0, 9.8645E-4,         0,
                                 0,         0,       TBD;

  this->_covariance_r << 1.4475E-06,          0,       0,
                                  0, 1.6644E-06,       0,
                                  0,          0,     TBD;
  this->_covariance_p = this->_covariance_q;

  this->_roll_pitch_yaw << 0.0,
                           0.0,
                           0.0;

  this->_observation_matrix_H << 1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0;
}

Ekf::~Ekf()
{

}

geometry_msgs::Vector3 Ekf::get_corrected(const geometry_msgs::Vector3& linear_acc,
                                          const geometry_msgs::Vector3& angular_vel,
                                          const geometry_msgs::Vector3& geomagnetism)
{
  this->_linear_acc << linear_acc.x,
                       linear_acc.y,
                       linear_acc.z;

  this->_angular_vel << angular_vel.x,
                        angular_vel.y,
                        angular_vel.z;
  this->_geomagnetism << geomagnetism.x,
                         geomagnetism.y,
                         geomagnetism.z;

  Matrix<double, 3, 1> pre_roll_pitch = _predict_angle();
  Matrix<double, 3, 3> state_jacobian_F = _get_state_jacobian();
  Matrix<double, 3, 3> pre_covariance_p = _predict_covariance_p(state_jacobian_F);
  
  Matrix<double, 3, 1> actual_observation_angle = _get_actual_observation_angle();

  Matrix<double, 3, 3> kalman_gain = _get_kalman_gain(pre_covariance_p);
  _update_covariance_p(kalman_gain, pre_covariance_p);
  return _get_corrected_angle(pre_roll_pitch, actual_observation_angle, kalman_gain);
}

geometry_msgs::Transform Ekf::get_predicted_value_no_filter()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_raw_no_filter);
                  
  Matrix<double, 3, 3> A;
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            , -1.0      * tri(0, 1),
       0.0, tri(0, 0) / tri(1, 1), tri(1, 0) / tri(1, 1);

  Matrix<double, 3, 1> pred_angle;
  pred_angle = _roll_pitch_raw_no_filter + this->_delta_t * (A * (this->_angular_vel));

  Matrix<double, 3, 1> observation_angle;
  observation_angle << atan2(this->_linear_acc(1), this->_linear_acc(2)), 
                       -1 * atan2(this->_linear_acc(0), hypot(this->_linear_acc(1), this->_linear_acc(2))),
                       TBD                                                                                ;

  geometry_msgs::Transform output;
  output.rotation.w = pred_angle(0);
  output.rotation.x = pred_angle(1);
  output.rotation.y = observation_angle(0);
  output.rotation.z = observation_angle(1);
  return output;
}

 Matrix<double, 3, 2> Ekf::_get_trigonometric(const Matrix<double, 3, 1>&  roll_pitch_yaw)
{
  Matrix<double, 3, 2> trigonometric;
  double roll = roll_pitch_yaw(0);
  double pitch = roll_pitch_yaw(1);
  trigonometric << sin(roll), sin(pitch), 
                   cos(roll), cos(pitch), 
                   tan(roll), tan(pitch);
  return trigonometric;
}

Matrix<double, 3, 1> Ekf::_predict_angle()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_yaw);

  Matrix<double, 3, 3> A;
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            ,      -1.0 * tri(0, 1),
       0.0, tri(0, 0) / tri(1, 1), tri(1, 0) / tri(1, 1);
  Matrix<double, 3, 1> pred_roll_pitch_yaw;
  pred_roll_pitch_yaw = _roll_pitch_yaw + this->_delta_t * (A * (this->_angular_vel));

  return pred_roll_pitch_yaw;
}

Matrix<double, 3, 3> Ekf::_get_state_jacobian()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_yaw);

  Matrix<double, 3, 3> state_jacobian_F;
  state_jacobian_F << 1 + this->_delta_t * (this->_angular_vel(1) * tri(1,0) * tri(2,1) - this->_angular_vel(2) * tri(0,0) * tri(2,1)), this->_delta_t * ((this->_angular_vel(1) * tri(0,0)) / pow(tri(1,1), 2) + (this->_angular_vel(2) * tri(1,0)) / pow(tri(1,1), 2)),
                     -1 * this->_delta_t * (this->_angular_vel(1) * tri(0,0) + this->_angular_vel(2) * tri(1,0))                      , 1.0                                                                                                                             ,
                     TBD , TBD, TBD;

  return state_jacobian_F;
}

Matrix<double, 3, 3> Ekf::_predict_covariance_p(const Matrix<double, 3, 3>& state_jacobian_F)
{
  Matrix<double, 3, 3> pre_covariance_p;
  pre_covariance_p = state_jacobian_F * this->_covariance_p * state_jacobian_F.transpose() + this->_covariance_q;
  return pre_covariance_p;
}

Matrix<double, 3, 1> Ekf::_get_actual_observation_angle()
{
  Matrix<double, 3, 1> observation_angle;
  observation_angle << atan2(this->_linear_acc(1), this->_linear_acc(2)), 
                       -1 * atan2(this->_linear_acc(0), hypot(this->_linear_acc(1), this->_linear_acc(2))),
                       TBD;
  return observation_angle;
}

Matrix<double, 3, 3> Ekf::_get_kalman_gain(const Matrix<double, 3, 3>& pre_covariance_p)
{
  Matrix<double, 3, 3> _kalman_gain;
  _kalman_gain = pre_covariance_p * this->_observation_matrix_H.transpose() * (this->_observation_matrix_H * pre_covariance_p * this->_observation_matrix_H.transpose() + this->_covariance_r).inverse();
  return _kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 3, 3>& kalman_gain, const Matrix<double, 3, 3>& pre_covariance_p)
{
  this->_covariance_p = (Matrix<double, 3, 3>::Identity() - kalman_gain * this->_observation_matrix_H) * pre_covariance_p;
}

geometry_msgs::Vector3 Ekf::_get_corrected_angle(const Matrix<double, 3, 1>& pre_roll_pitch_yaw,
                                                 const Matrix<double, 3, 1>& actual_observation_angle,
                                                 const Matrix<double, 3, 3>& kalman_gain)
{
  geometry_msgs::Vector3 output;
  this->_roll_pitch_yaw = pre_roll_pitch_yaw + kalman_gain * (actual_observation_angle - this->_observation_matrix_H * pre_roll_pitch_yaw);
  output.x = this->_roll_pitch_yaw(0);
  output.y = this->_roll_pitch_yaw(1);
  output.z = this->_roll_pitch_yaw(2);
  return output;
}