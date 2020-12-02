#include "ExtendedKalmanFilter.h"

#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <ros.h>
#include <geometry_msgs/Vector3.h>

Ekf::Ekf(float delta_t)
{
  this->_delta_t = delta_t;
  this->_covariance_q << 1.74E-2*delta_t*delta_t, 0,
                         0                      , 1.74E-2*delta_t*delta_t;
  this->_covariance_r << 1.00*delta_t*delta_t, 0,
                         0                      , 1.00*delta_t*delta_t;
  this->_covariance_p = this->_covariance_q;
  this->_angle << 0.0,
                  0.0;
  this->_observation_matrix_H << 1.0, 0.0,
                                 0.0, 1.0;
}

Ekf::~Ekf()
{

}

geometry_msgs::Vector3 Ekf::get_corrected(const geometry_msgs::Vector3& linear_acc,
                                          const geometry_msgs::Vector3& angular_vel)
{
  this->_linear_acc << linear_acc.x,
                       linear_acc.y,
                       linear_acc.z;

  this->_angular_vel << angular_vel.x,
                        angular_vel.y,
                        angular_vel.z;

  Matrix<double, 2, 1> pre_angle = _predict_angle();
  Matrix<double, 2, 2> state_jacobian_F = _get_state_jacobian();
  Matrix<double, 2, 2> pre_covariance_p = _predict_covariance_p(state_jacobian_F);
  
  Matrix<double, 2, 1> actual_observation_angle = _get_actual_observation_angle();

  //  Because observatio matrix H is identity, pre_angle is able to be used as pred_observation_angle
  Matrix<double, 2, 2> kalman_gain = _get_kalman_gain(pre_covariance_p);
  _update_covariance_p(kalman_gain, pre_covariance_p);
  return _get_corrected_angle(pre_angle, actual_observation_angle, kalman_gain);
}

 Matrix<double, 3, 2> Ekf::_get_trigonometric(const Matrix<double, 2, 1>& angle)
{
  Matrix<double, 3, 2> trigonometric;
  trigonometric << sin(angle(0)), sin(angle(1)), 
                   cos(angle(0)), cos(angle(1)), 
                   tan(angle(0)), tan(angle(1));
  return trigonometric;
}

Matrix<double, 2, 1> Ekf::_predict_angle()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_angle);
  Matrix<double, 2, 3> A;
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            , -1.0 * tri(0, 0); 
  Matrix<double, 2, 1> pred_angle;
  pred_angle = this->_angle + this->_delta_t * (A * this->_angular_vel);
  return pred_angle;
}

Matrix<double, 2, 2> Ekf::_get_state_jacobian()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(_angle);
  Matrix<double, 2, 2> state_jacobian_F;
  state_jacobian_F << 1 + this->_delta_t * (this->_angular_vel(1) * tri(1,0) * tri(2,1) - this->_angular_vel(2) * tri(0,0) * tri(2,1)), this->_delta_t * ((this->_angular_vel(1) * tri(0,0)) / pow(tri(1,1), 2) + (this->_angular_vel(2) * tri(1,0)) / pow(tri(1,1), 2)),
                     -1 * this->_delta_t * (this->_angular_vel(1) * tri(0,0) + this->_angular_vel(2) * tri(1,0))                      , 1.0; 
  return state_jacobian_F;
}

Matrix<double, 2, 2> Ekf::_predict_covariance_p(const Matrix<double, 2, 2>& state_jacobian_F)
{
  Matrix<double, 2, 2> pre_covariance_p;
  pre_covariance_p = state_jacobian_F * this->_covariance_p * state_jacobian_F.transpose() + this->_covariance_q;
  return pre_covariance_p;
}

Matrix<double, 2, 1> Ekf::_get_actual_observation_angle()
{
  Matrix<double, 2, 1> observation_angle;
  observation_angle << atan2(this->_linear_acc(1), this->_linear_acc(2)), 
                       -1 * atan2(this->_linear_acc(0), hypot(this->_linear_acc(1), this->_linear_acc(2)));
  return observation_angle;
}

Matrix<double, 2, 2> Ekf::_get_kalman_gain(const Matrix<double, 2, 2>& pre_covariance_p)
{
  Matrix<double, 2, 2> _kalman_gain;
  _kalman_gain = pre_covariance_p * this->_observation_matrix_H.transpose() * (this->_observation_matrix_H * pre_covariance_p * this->_observation_matrix_H.transpose() + this->_covariance_r).inverse();
  return _kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 2, 2>& kalman_gain, const Matrix<double, 2, 2>& pre_covariance_p)
{
  this->_covariance_p = (Matrix<double, 2, 2>::Identity() - kalman_gain * this->_observation_matrix_H) * pre_covariance_p;
}

geometry_msgs::Vector3 Ekf::_get_corrected_angle(const Matrix<double, 2, 1>& predict_angle,
                                                 const Matrix<double, 2, 1>& actual_observation_angle,
                                                 const Matrix<double, 2, 2>& kalman_gain)
{
  geometry_msgs::Vector3 output;
  this->_angle = predict_angle + kalman_gain * (actual_observation_angle - predict_angle);
  output.x = this->_angle(0);
  output.y = this->_angle(1);
  return output;
}