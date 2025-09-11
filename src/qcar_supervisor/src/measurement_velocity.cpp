#include "qcar_supervisor/measurement_velocity.h"

Eigen::VectorXd h_vel(const qcar_nav::KinematicModel::StateVec& x) {
  return Eigen::VectorXd::Constant(1, x(0)); // vx
}

Eigen::MatrixXd H_vel() {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
  H(0, 0) = 1.0;
  return H;
}
