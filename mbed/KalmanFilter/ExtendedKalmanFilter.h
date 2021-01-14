#include "Eigen/Dense.h"
using namespace Eigen;

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <ros.h>

class Ekf {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Ekf(double delta_t);
  ~Ekf();
  geometry_msgs::Quaternion get_compensation_state(
    const geometry_msgs::Vector3 &linear_acc,
    const geometry_msgs::Vector3 &angular_vel,
    const geometry_msgs::Vector3 &geomagnetism);

private:
  double _delta_t;

  double grav_acc;
  double mag_n;
  double mag_d;

  // Error Covariance Matrix
  Matrix<double, 7, 7> _covariance_p;  // this matrix will be updated
  Matrix<double, 3, 3> _covariance_q;  // const (should be tuning)
  Matrix<double, 6, 6> _covariance_r;  // const (should be tuning)

  // bias beta matrix
  Matrix<double, 3, 3> _beta;

  // const matrix G
  Matrix<double, 7, 3> _G;

  // to retain angle for next step
  Matrix<double, 7, 1> _state_value;

  Matrix<double, 7, 1> _get_predicted_state(const Matrix<double, 3, 1> &gyro_sense);
  Matrix<double, 7, 7> _get_predicted_jacobian_F(const Matrix<double, 3, 1> &gyro_sense);
  Matrix<double, 7, 7> _get_predicted_covariance_p(const Matrix<double, 7, 7> &predicted_jacobian_F);

  Matrix<double, 6, 7> _get_observation_jacobian_H(const Matrix<double, 7, 1> &predicted_state);
  Matrix<double, 6, 1> _get_observation_state(const Matrix<double, 7, 1> &predicted_state);

  Matrix<double, 7, 6> _get_kalman_gain(const Matrix<double, 6, 7> &observation_jacobian_H,
                                        const Matrix<double, 7, 7> &predicted_covariance_p);

  void _update_covariance_p(const Matrix<double, 7, 6> &kalman_gain,
                            const Matrix<double, 6, 7> &observation_jacobian_H,
                            const Matrix<double, 7, 7> &predicted_covariance_p);

  Matrix<double, 7, 1> _get_compensation_state(
    const Matrix<double, 7, 1> &predicted_state,
    const Matrix<double, 7, 6> &kalman_gain,
    const Matrix<double, 6, 1> &acc_geo_sense,
    const Matrix<double, 6, 1> &observation_state);
};