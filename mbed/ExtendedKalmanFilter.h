#include "Eigen/Dense.h"
using namespace Eigen;

#include <ros.h>
#include <geometry_msgs/Vector3.h>

class Ekf 
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ekf(float delta_t);
  ~Ekf();
  geometry_msgs::Vector3 get_corrected(const geometry_msgs::Vector3& linear_acc,
                                       const geometry_msgs::Vector3& angular_vel);
  private:
  float _delta_t;

  // input values
  Matrix<double, 3, 1> _linear_acc;  //  m/s^2
  Matrix<double, 3, 1> _angular_vel;  //  rad/s

  // Error Covariance Matrix
  Matrix<double, 2, 2> _covariance_p;  // this matrix will be updated
  Matrix<double, 2, 2> _covariance_q;  //const (should be tuning)
  Matrix<double, 2, 2> _covariance_r;  //const (should be tuning)

  // observation equation matrix H
  Matrix<double, 2, 2> _observation_matrix_H;

 // to retain angle for next step
  Matrix<double, 2, 1> _angle;

  Matrix<double, 3, 2> _get_trigonometric(const Matrix<double, 2, 1>& angle);
 
  // predict step
  Matrix<double, 2, 1> _predict_angle();
  Matrix<double, 2, 2> _get_state_jacobian();
  Matrix<double, 2, 2> _predict_covariance_p(const Matrix<double, 2, 2>& state_jacobian_F);

  // get_observation_step
  Matrix<double, 2, 1> _get_actual_observation_angle();

  // update_step
  Matrix<double, 2, 2> _get_kalman_gain(const Matrix<double, 2, 2>& _pre_covariance_p);
  void _update_covariance_p(const Matrix<double, 2, 2>& kalman_gain, const Matrix<double, 2, 2>& pre_covariance_p);
  geometry_msgs::Vector3 _get_corrected_angle(const Matrix<double, 2, 1>& predict_angle, 
                                              const Matrix<double, 2, 1>& actual_observation_angle,
                                              const Matrix<double, 2, 2>& kalman_gain);
};