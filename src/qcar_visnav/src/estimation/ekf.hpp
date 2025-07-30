// File location: qcar_visnav/src/estimation/ekf.hpp
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include "system_qcar.hpp"
#include "measurement.hpp"

class EKF {
public:
    EKF() {}

    void initialize() {
        x_ = Eigen::VectorXd::Zero(10);  // Initial state [u, v, r, omegaF, omegaR, delta, N, E, psi, gyro_bias]
        P_ = Eigen::MatrixXd::Identity(10, 10) * 0.1;  // Initial covariance
    }

    void update(const Measurement& meas) {
        const SystemQCAR sys;

        // Predict expected measurement
        Eigen::VectorXd h = meas.h(x_, sys);
        Eigen::MatrixXd H = meas.H(x_, sys);
        Eigen::MatrixXd R = meas.covariance();
        Eigen::VectorXd y = meas.z - h;  // Innovation (residual)

        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(10, 10) - K * H) * P_;
    }

    Eigen::VectorXd getState() const { return x_; }
    Eigen::MatrixXd getCovariance() const { return P_; }

private:
    Eigen::VectorXd x_;  // State estimate
    Eigen::MatrixXd P_;  // Covariance
};
