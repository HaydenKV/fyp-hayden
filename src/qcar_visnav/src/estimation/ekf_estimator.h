
#ifndef QCAR_EKF_ESTIMATOR_H
#define QCAR_EKF_ESTIMATOR_H

#include <Eigen/Dense>
#include <ros/ros.h>

class EKFEstimator
{
public:
    EKFEstimator();
    void initialize(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);
    void predict(const Eigen::Vector2d& u, double dt);
    void update(const Eigen::VectorXd& z, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

    const Eigen::VectorXd& getState() const { return x_; }
    const Eigen::MatrixXd& getCovariance() const { return P_; }

private:
    Eigen::VectorXd x_;  // State estimate
    Eigen::MatrixXd P_;  // Covariance
    Eigen::MatrixXd Q_;  // Process noise

    // Model dynamics
    Eigen::VectorXd f(const Eigen::VectorXd& x, const Eigen::Vector2d& u);
    Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd& x, const Eigen::Vector2d& u, double dt);
};

#endif // QCAR_EKF_ESTIMATOR_H
