#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense.h"
using namespace Eigen;
using namespace std;

#include <ros.h>
#include <geometry_msgs/Vector3.h>

Ekf::Ekf(double delta_t)
{
  this->_delta_t = delta_t;
  this->_covariance_q << 1.74E-2*delta_t*delta_t, 0,                        0,                       0,                       0, 
                         0,                       1.74E-2*delta_t*delta_t,  0,                       0,                       0,
                         0,                       0,                        1.74E-2*delta_t*delta_t, 0,                       0,
                         0,                       0,                        0,                       1.74E-2*delta_t*delta_t, 0, 
                         0,                       0,                        0,                       0,                       1.74E-2*delta_t*delta_t;

  this->_covariance_r << 1.0*delta_t*delta_t, 0                  ,                    
                         0                  , 1.0*delta_t*delta_t;

  this->_covariance_p = this->_covariance_q;
  this->_roll_pitch_bias << 0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0;
  this->_yaw = 0.0;
  this->_observation_matrix_H << 1.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0, 0.0, 0.0;
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

  Matrix<double, 5, 1> pre_roll_pitch_bais = _predict_angle();
  Matrix<double, 5, 5> state_jacobian_F = _get_state_jacobian();
  Matrix<double, 5, 5> pre_covariance_p = _predict_covariance_p(state_jacobian_F);
  
  Matrix<double, 2, 1> actual_observation_angle = _get_actual_observation_angle();

  //  Because observatio matrix H is identity, pre_angle is able to be used as pred_observation_angle
  Matrix<double, 5, 2> kalman_gain = _get_kalman_gain(pre_covariance_p);
  _update_covariance_p(kalman_gain, pre_covariance_p);
  return _get_corrected_angle(pre_roll_pitch_bais, actual_observation_angle, kalman_gain);
}

 Matrix<double, 3, 2> Ekf::_get_trigonometric(const Matrix<double, 5, 1>&  roll_pitch_bais)
{
  Matrix<double, 3, 2> trigonometric;
  double roll = roll_pitch_bais(0);
  double pitch = roll_pitch_bais(1);
  trigonometric << sin(roll), sin(pitch), 
                   cos(roll), cos(pitch), 
                   tan(roll), tan(pitch);
  return trigonometric;
}

Matrix<double, 5, 1> Ekf::_predict_angle()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_bias);

  Matrix<double, 2, 1> roll_pitch;
  roll_pitch << _roll_pitch_bias(0),
                _roll_pitch_bias(1);

  Matrix<double, 3, 1> angular_bais;
  angular_bais << _roll_pitch_bias(2),
                  _roll_pitch_bias(3),
                  _roll_pitch_bias(4);
                  
  Matrix<double, 2, 3> A;
  A << 1.0, tri(0, 0) * tri(2, 1), tri(1, 0) * tri(2, 1),
       0.0, tri(1, 0)            ,      -1.0 * tri(0, 1); 
  Matrix<double, 2, 1> pred_angle;
  pred_angle = roll_pitch + this->_delta_t * (A * (this->_angular_vel - angular_bais));

  Matrix<double, 5, 1> pred_roll_pitch_bais;
  pred_roll_pitch_bais << pred_angle(0),
                          pred_angle(1),
                          angular_bais(0),
                          angular_bais(1),
                          angular_bais(2); 

  // predict yaw angle here to minimize tri calcurate
  this->_yaw += this->_delta_t * ((this->_angular_vel(1) * tri(0, 0)) / tri(1, 1) + ((this->_angular_vel(2) - angular_bais(2)) * tri(1, 0)) / tri(1, 1));
  return pred_roll_pitch_bais;
}

Matrix<double, 5, 5> Ekf::_get_state_jacobian()
{
  Matrix<double, 3, 2> tri;
  tri = _get_trigonometric(this->_roll_pitch_bias);

  Matrix<double, 3, 1> biased_angular_vel;
  biased_angular_vel << this->_angular_vel(0) - this->_roll_pitch_bias(2),
                        this->_angular_vel(1) - this->_roll_pitch_bias(3),
                        this->_angular_vel(2) - this->_roll_pitch_bias(4);

  Matrix<double, 5, 5> state_jacobian_F;
  state_jacobian_F << 1 + this->_delta_t * (biased_angular_vel(1) * tri(1,0) * tri(2,1) - biased_angular_vel(2) * tri(0,0) * tri(2,1)), this->_delta_t * ((biased_angular_vel(1) * tri(0,0)) / pow(tri(1,1), 2) + (biased_angular_vel(2) * tri(1,0)) / pow(tri(1,1), 2)), -1 * this->_delta_t, -1 * this->_delta_t * tri(0,0) * tri(2,1), -1 * this->_delta_t * tri(1,0) * tri(2,1),
                     -1 * this->_delta_t * (biased_angular_vel(1))                                                                    , 1.0 + (-1) * biased_angular_vel(2) * tri(1,1)                                                                                   , 0                  , -1 * this->_delta_t * tri(1,0)           , -1 * this->_delta_t * tri(0,1)           ,
                     0                                                                                                                , 0                                                                                                                               , 1                  , 0                                        , 0                                        ,
                     0                                                                                                                , 0                                                                                                                               , 0                  , 1                                        , 0                                        ,
                     0                                                                                                                , 0                                                                                                                               , 0                  , 0                                        , 1                                        ; 

  return state_jacobian_F;
}

Matrix<double, 5, 5> Ekf::_predict_covariance_p(const Matrix<double, 5, 5>& state_jacobian_F)
{
  Matrix<double, 5, 5> pre_covariance_p;
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

Matrix<double, 5, 2> Ekf::_get_kalman_gain(const Matrix<double, 5, 5>& pre_covariance_p)
{
  Matrix<double, 5, 2> _kalman_gain;
  _kalman_gain = pre_covariance_p * this->_observation_matrix_H.transpose() * (this->_observation_matrix_H * pre_covariance_p * this->_observation_matrix_H.transpose() + this->_covariance_r).inverse();
  return _kalman_gain;
}

void Ekf::_update_covariance_p(const Matrix<double, 5, 2>& kalman_gain, const Matrix<double, 5, 5>& pre_covariance_p)
{
  this->_covariance_p = (Matrix<double, 5, 5>::Identity() - kalman_gain * this->_observation_matrix_H) * pre_covariance_p;
}

geometry_msgs::Vector3 Ekf::_get_corrected_angle(const Matrix<double, 5, 1>& pre_roll_pitch_bias,
                                                 const Matrix<double, 2, 1>& actual_observation_angle,
                                                 const Matrix<double, 5, 2>& kalman_gain)
{
  geometry_msgs::Vector3 output;
  this->_roll_pitch_bias = pre_roll_pitch_bias + kalman_gain * (actual_observation_angle - this->_observation_matrix_H * pre_roll_pitch_bias);
  output.x = this->_roll_pitch_bias(0);
  output.y = this->_roll_pitch_bias(1);
  output.z = this->_yaw;
  return output;
}