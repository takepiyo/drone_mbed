#include "Eigen/Dense.h"
using namespace Eigen;

#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Transform.h>

class Ekf 
{
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ekf(double delta_t);
  ~Ekf();
  geometry_msgs::Vector3 get_corrected(const geometry_msgs::Vector3& linear_acc,
                                       const geometry_msgs::Vector3& angular_vel,
                                       const geometry_msgs::Vector3& geomagnetism);
  geometry_msgs::Vector3 get_bais();
  geometry_msgs::Transform get_predicted_value_no_filter();

  private:
  double _delta_t;
  double _yaw;

  // input values
  Matrix<double, 3, 1> _linear_acc;  //  m/s^2
  Matrix<double, 3, 1> _angular_vel;  //  rad/s
  Matrix<double, 3, 1> _geomagnetism;

  // Error Covariance Matrix
  Matrix<double, 3, 3> _covariance_p;  // this matrix will be updated
  Matrix<double, 3, 3> _covariance_q;  // const (should be tuning)
  Matrix<double, 3, 3> _covariance_r;  // const (should be tuning)

  // observation equation matrix H
  Matrix<double, 3, 3> _observation_matrix_H;

 // to retain angle for next step
  Matrix<double, 3, 1> _roll_pitch_yaw;

  // to measure variance, no filtered angle
  Matrix<double, 3, 1> _roll_pitch_raw_no_filter;
  Matrix<double, 3, 1> _acc_angle_no_filter;

  Matrix<double, 3, 2> _get_trigonometric(const Matrix<double, 3, 1>& roll_pitch_yaw);
 
  // predict step
  Matrix<double, 3, 1> _predict_angle();
  Matrix<double, 3, 3> _get_state_jacobian();
  Matrix<double, 3, 3> _predict_covariance_p(const Matrix<double, 3, 3>& state_jacobian_F);

  // get_observation_step
  Matrix<double, 3, 1> _get_actual_observation_angle();

  // update_step
  Matrix<double, 3, 3> _get_kalman_gain(const Matrix<double, 3, 3>& _pre_covariance_p);
  void _update_covariance_p(const Matrix<double, 3, 3>& kalman_gain, const Matrix<double, 3, 3>& pre_covariance_p);
  geometry_msgs::Vector3 _get_corrected_angle(const Matrix<double, 3, 1>& pre_roll_pitch_yaw, 
                                              const Matrix<double, 3, 1>& actual_observation_angle,
                                              const Matrix<double, 3, 3>& kalman_gain);
};