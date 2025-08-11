#include "qcar_visnav/estimation/ekf/ekf_core.h"
#include "qcar_visnav/estimation/integrators/rk4sde.h"
#include <cmath>

namespace qcar_nav {

SREKF::SREKF() {
  mu_.setZero();
  S_.setIdentity();
  S_ *= 1e-1;
}

void SREKF::setInitial(const Vec& mu0, const Mat& S0) {
  mu_ = mu0;
  S_  = S0;
}

void SREKF::predict(const qcar_nav::KinematicModel& model, double t, double dt) {
  // 1) RK4 step with Jacobians
  Eigen::Vector3d sqrtQc; Eigen::Matrix<double,7,3> Jdw;
  RK4SDE::Vec x_next; RK4SDE::Mat Jdx;
  RK4SDE::stepWithJac(model, t, mu_, dt, x_next, Jdx, Jdw); // idxQ defaults to {4,5,6}

  // 2) Build discrete process covariance via Jdw
  Eigen::Vector3d tmp; Eigen::Matrix<double,7,3> L;
  model.processNoise(dt, tmp, L);
  // Discrete driving noise covariance (choose one consistent scheme)
  Eigen::Matrix3d Qw = (sqrtQc.array().square()).matrix().asDiagonal() * dt;// white-noise ~ √Hz -> × dt
  // Qk via affine map
  SREKF::Mat Qk = Jdw * Qw * Jdw.transpose();

  // 3) Covariance predict via P
  SREKF::Mat P  = S_.transpose() * S_;
  SREKF::Mat Pn = Jdx * P * Jdx.transpose() + Qk;
  Pn = 0.5*(Pn + Pn.transpose());
  Eigen::LLT<SREKF::Mat> llt(Pn);
  S_ = llt.matrixU();
  mu_ = x_next;
  mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));

}

void SREKF::update(const Eigen::VectorXd& z,
                   const Eigen::VectorXd& h,
                   const Eigen::MatrixXd& H,
                   const Eigen::MatrixXd& sqrtR)
{
  // P from square root
  Mat P = S_.transpose() * S_;

  // innovation
  Eigen::VectorXd y = z - h;

  // Kalman gain
  Eigen::MatrixXd Szz = H * P * H.transpose() + (sqrtR.transpose() * sqrtR);
  Eigen::MatrixXd K   = P * H.transpose() * Szz.inverse();

  // state update
  mu_.noalias() += K * y;
  mu_(2) = std::atan2(std::sin(mu_(2)), std::cos(mu_(2)));

  // Joseph form for numerical stability
  Mat I = Mat::Identity();
  Mat Pn = (I - K*H) * P * (I - K*H).transpose() + K * (sqrtR.transpose()*sqrtR) * K.transpose();
  Pn = 0.5 * (Pn + Pn.transpose());

  // Refresh S via Cholesky
  Eigen::LLT<Mat> llt(Pn);
  S_ = llt.matrixU();
}

} // namespace qcar_nav
