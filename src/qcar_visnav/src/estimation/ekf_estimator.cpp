#include "ekf_estimator.h"
#include "params.h"
#include <Eigen/Dense>

namespace qcar_visnav {

EKFEstimator::EKFEstimator() {
    // Initialize state vector and covariance
    x_.setZero();
    P_ = 0.01 * Eigen::Matrix<double, 10, 10>::Identity();
}

void EKFEstimator::predict(const Eigen::Vector2d& u, double dt) {
    // Prediction step
    Eigen::Matrix<double, 10, 1> x_pred = dynamics::computeDynamics(x_, u);
    Eigen::Matrix<double, 10, 10> A = jacobian::computeStateJacobian(x_, u);
    x_ = x_pred;
    P_ = A * P_ * A.transpose() + Q_;
}

void EKFEstimator::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& R, const Eigen::VectorXd& h_x) {
    Eigen::VectorXd y = z - h_x;
    Eigen::MatrixXd S = H * P_ * H.transpose() + R;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;
    Eigen::Matrix<double, 10, 10> I = Eigen::Matrix<double, 10, 10>::Identity();
    P_ = (I - K * H) * P_;
}

Eigen::Matrix<double, 10, 1> EKFEstimator::getState() const {
    return x_;
}

void EKFEstimator::setInitialState(const Eigen::Matrix<double, 10, 1>& x0) {
    x_ = x0;
}

void EKFEstimator::setProcessNoise(const Eigen::Matrix<double, 10, 10>& Q) {
    Q_ = Q;
}

} // namespace qcar_visnav
