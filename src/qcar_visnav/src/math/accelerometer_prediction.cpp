#include "qcar_visnav/math/accelerometer_prediction.h"
#include <ros/ros.h>
#include <cmath>

namespace qcar_visnav {

AccelerometerPredictor::AccelerometerPredictor() {
    // Default values from MATLAB MeasurementAccelerometer.m
    rMBb_ << 0.1278, 0.0223, 0.0895;
    Rbm_ = Eigen::Matrix3d::Identity();
}

void AccelerometerPredictor::setMountingParams(const Eigen::Vector3d& rMBb, const Eigen::Matrix3d& Rbm) {
    rMBb_ = rMBb;
    Rbm_ = Rbm;
}

PredictionResult AccelerometerPredictor::predict(const Eigen::VectorXd& x, const SystemParams& system) {
    PredictionResult result;
    
    if (x.size() != 10) {
        ROS_ERROR("State vector must have 10 elements, got %ld", x.size());
        return result;
    }
    
    // Extract state variables - from MATLAB: x = [u; v; r; wF; wR; delta; N; E; psi; gyro]
    double u = x(0);    // Forward velocity
    double v = x(1);    // Lateral velocity  
    double r = x(2);    // Yaw rate
    double psi = x(8);  // Heading angle
    
    // Rotation matrices
    Eigen::Matrix3d Rbn;
    Rbn << cos(psi), -sin(psi), 0,
           sin(psi),  cos(psi), 0,
           0,         0,        1;
    Eigen::Matrix3d Rnb = Rbn.transpose();
    
    // Angular velocity vector
    Eigen::Vector3d omegaBNb(0, 0, r);
    Eigen::Matrix3d SomegaBNb = skewSymmetric(omegaBNb);
    Eigen::Matrix3d SrMBb = skewSymmetric(rMBb_);
    
    // Gravity vector in navigation frame
    Eigen::Vector3d gn(0, 0, system.g);
    
    // Get system dynamics
    Eigen::VectorXd dxdt = getSystemDynamics(x, system);
    Eigen::Vector3d omegadot(0, 0, dxdt(2)); // Only yaw rate derivative
    
    // Velocity and acceleration in body frame
    Eigen::Vector3d vBNb(u, v, 0);
    Eigen::Vector3d aBNb(dxdt(0), dxdt(1), 0);
    
    // Accelerometer acceleration (from MATLAB predict.m)
    Eigen::Vector3d aMNb = aBNb 
                         + SomegaBNb * vBNb 
                         - SrMBb * omegadot
                         - SomegaBNb * SrMBb * omegaBNb;
    
    // Measurement prediction h(x) = [1 0 0; 0 1 0] * Rbm * (aMNb - Rnb*gn)
    Eigen::Matrix<double, 2, 3> H_select;
    H_select << 1, 0, 0,
                0, 1, 0;
    
    result.h = H_select * Rbm_ * (aMNb - Rnb * gn);
    
    // Compute Jacobian (from MATLAB predict.m)
    result.H = Eigen::MatrixXd::Zero(2, 10);
    
    // Jacobian of aMNb w.r.t. x (key parts from MATLAB)
    Eigen::MatrixXd daMNbdx = Eigen::MatrixXd::Zero(3, 10);
    
    // From MATLAB predict.m jacobian calculations
    daMNbdx(0, 1) = -r;                    // du/dv term
    daMNbdx(0, 2) = -v - rMBb_(0) * 2 * r; // du/dr term with centripetal
    
    daMNbdx(1, 0) = r;                     // dv/du term  
    daMNbdx(1, 2) = u - rMBb_(1) * 2 * r;  // dv/dr term with centripetal
    
    // Add dynamics Jacobian contributions
    Eigen::MatrixXd ddxdtdx = computeSystemDynamicsJacobian(x, system);
    daMNbdx.row(0) += ddxdtdx.row(0) + rMBb_(1) * ddxdtdx.row(2);
    daMNbdx.row(1) += ddxdtdx.row(1) - rMBb_(0) * ddxdtdx.row(2);
    
    // Final Jacobian: H = [1 0 0; 0 1 0] * Rbm * daMNbdx
    result.H = H_select * Rbm_ * daMNbdx;
    
    result.valid = true;
    return result;
}

Eigen::Matrix3d AccelerometerPredictor::skewSymmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0,    -v(2),  v(1),
         v(2),  0,    -v(0),
        -v(1),  v(0),  0;
    return S;
}

Eigen::VectorXd AccelerometerPredictor::getSystemDynamics(const Eigen::VectorXd& x, const SystemParams& system) {
    // Simplified dynamics - you'll need to port the full QCAR dynamics from MATLAB
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(10);
    
    // Basic kinematic model for now
    double u = x(0);
    double v = x(1); 
    double r = x(2);
    double psi = x(8);
    
    // Simple bicycle model derivatives
    dxdt(0) = 0.0;  // du/dt - would need full dynamics
    dxdt(1) = 0.0;  // dv/dt - would need full dynamics  
    dxdt(2) = 0.0;  // dr/dt - would need full dynamics
    dxdt(6) = u * cos(psi) - v * sin(psi);  // dN/dt
    dxdt(7) = u * sin(psi) + v * cos(psi);  // dE/dt
    dxdt(8) = r;                            // dpsi/dt
    
    return dxdt;
}

Eigen::MatrixXd AccelerometerPredictor::computeSystemDynamicsJacobian(const Eigen::VectorXd& x, const SystemParams& system) {
    // Simplified Jacobian - you'll need to port full QCAR dynamics Jacobian
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(10, 10);
    
    double psi = x(8);
    
    // Basic kinematic jacobians
    J(6, 0) = cos(psi);   // dN/du
    J(6, 1) = -sin(psi);  // dN/dv
    J(6, 8) = -x(0) * sin(psi) - x(1) * cos(psi); // dN/dpsi
    
    J(7, 0) = sin(psi);   // dE/du
    J(7, 1) = cos(psi);   // dE/dv  
    J(7, 8) = x(0) * cos(psi) - x(1) * sin(psi);  // dE/dpsi
    
    J(8, 2) = 1.0;        // dpsi/dr
    
    return J;
}

} // namespace qcar_visnav