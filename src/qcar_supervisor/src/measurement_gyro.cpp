#include "qcar_supervisor/measurement_gyro.h"

Eigen::VectorXd h_gyro(const qcar_nav::KinematicModel::StateVec& x, const qcar_nav::ModelParams& params) {
  double vx = x(0);
  double delta = x(5);
  double r_pred = (vx / params.L) * std::tan(delta);
  return Eigen::VectorXd::Constant(1, r_pred + x(3)); // r + bg
}

Eigen::MatrixXd H_gyro(const qcar_nav::KinematicModel::StateVec& x, const qcar_nav::ModelParams& params) {
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
  double vx = x(0);
  double delta = x(5);
  double sec2 = 1.0 / std::pow(std::cos(delta), 2);
  H(0, 0) = std::tan(delta) / params.L;
  H(0, 5) = vx * sec2 / params.L;
  H(0, 3) = 1.0; // bg
  return H;
}
