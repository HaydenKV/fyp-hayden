// ekf.cpp
#include <qcar_supervisor/ekf.h>

namespace qcar_supervisor {

ExtendedKalmanFilter::ExtendedKalmanFilter() {
  // Nothing to do here
}

void ExtendedKalmanFilter::init(const StateVec& x0, const StateMat& P0) {
  x_ = x0;
  P_ = P0;
}

void ExtendedKalmanFilter::setModelParams(const ModelParams& params) {
  model_.setParams(params);
}

void ExtendedKalmanFilter::setInput(const ModelInput& input) {
  model_.setInput(input);
  R_u_ = model_.getInputNoise(input.dt);
}

void ExtendedKalmanFilter::predict(double t, double dt) {
  // 1. State prediction
  StateVec x_pred = model_.predict(x_, t, dt);

  // 2. Compute Jacobians
  StateMat F = model_.getProcessJacobian(x_, dt);
  InputMat G = model_.getInputJacobian(x_, dt);

  //Eigen::Matrix<double, KinematicModel::INPUT_SIZE, KinematicModel::INPUT_SIZE> G 
  //  = model_.getInputJacobian(x_, dt);

  // 3. Compute process noise
  StateMat Q_c = model_.getProcessNoise(dt);

  // 4. Covariance prediction
  P_ = F * P_ * F.transpose() + Q_c + G * R_u_ * G.transpose();

  // 5. Commit
  x_ = x_pred;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& z,
                                  std::function<Eigen::VectorXd(const StateVec&)> h,
                                  std::function<Eigen::MatrixXd(const StateVec&)> H,
                                  const Eigen::MatrixXd& R) {
  // 1. Predicted measurement
  Eigen::VectorXd z_pred = h(x_);

  // 2. Innovation
  Eigen::VectorXd y = z - z_pred;

  // 3. Measurement Jacobian
  Eigen::MatrixXd H_jac = H(x_);

  // 4. Innovation covariance
  Eigen::MatrixXd S = H_jac * P_ * H_jac.transpose() + R;

  // 5. Kalman gain
  Eigen::MatrixXd K = P_ * H_jac.transpose() * S.inverse();

  // 6. State & covariance update
  x_ = x_ + K * y;
  StateMat I = StateMat::Identity();
  P_ = (I - K * H_jac) * P_;
}

} // namespace qcar_supervisor

