#include "qcar_visnav/math/ekf_filter.h"
#include <ros/ros.h>

namespace qcar_visnav {

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    // Initialize state vector (10 elements: [u; v; r; wF; wR; delta; N; E; psi; gyro])
    state_ = Eigen::VectorXd::Zero(10);
    
    // Initialize covariance (from MATLAB SystemQCAR.m)
    Eigen::VectorXd P0_diag(10);
    P0_diag << 1e-2, 1e-2, 1e-1, 5e-2, 5e-2, 1e-3, 1e-2, 1e-2, 5e-2, 1e1;
    covariance_ = P0_diag.asDiagonal();
    
    // Initialize measurement noise (from MATLAB noiseDensity.m)
    setMeasurementNoise(0.1);
}

void ExtendedKalmanFilter::initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance) {
    if (initial_state.size() != 10) {
        ROS_ERROR("Initial state must have 10 elements");
        return;
    }
    
    state_ = initial_state;
    covariance_ = initial_covariance;
    
    ROS_INFO("EKF initialized with state: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             state_(0), state_(1), state_(2), state_(3), state_(4), 
             state_(5), state_(6), state_(7), state_(8), state_(9));
}

bool ExtendedKalmanFilter::measurementUpdate(const AccelerometerMeasurement& measurement) {
    if (!validateMeasurement(measurement)) {
        ROS_WARN("Invalid accelerometer measurement, skipping update");
        return false;
    }
    
    // Get prediction h(x) and Jacobian H - equivalent to MATLAB predict.m
    PredictionResult prediction = predictor_.predict(state_, system_params_);
    
    if (!prediction.valid) {
        ROS_ERROR("Accelerometer prediction failed");
        return false;
    }
    
    // Innovation (residual) - equivalent to MATLAB y = z - h(x)
    Eigen::Vector2d z_accel = measurement.acceleration.head<2>(); // Only x,y components
    Eigen::Vector2d innovation = z_accel - prediction.h;
    
    // Innovation covariance S = H*P*H' + R
    Eigen::Matrix2d S = prediction.H * covariance_ * prediction.H.transpose() + R_;
    
    // Check for numerical issues
    if (S.determinant() < 1e-12) {
        ROS_WARN("Innovation covariance near singular, skipping update");
        return false;
    }
    
    // Kalman gain K = P*H'*inv(S)
    Eigen::MatrixXd K = covariance_ * prediction.H.transpose() * S.inverse();
    
    // State update: x = x + K*innovation
    state_ = state_ + K * innovation;
    
    // Covariance update: P = (I - K*H)*P (Joseph form for numerical stability)
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_.size(), state_.size());
    covariance_ = (I - K * prediction.H) * covariance_;
    
    // Validate updated state
    if (!validateState()) {
        ROS_WARN("State validation failed after update");
        return false;
    }
    
    ROS_DEBUG("EKF measurement update completed. Innovation norm: %.6f", innovation.norm());
    return true;
}

void ExtendedKalmanFilter::timeUpdate(double dt) {
    // Simple time update - would need full QCAR dynamics for complete implementation
    // For now, just propagate kinematic states
    
    double u = state_(0);
    double v = state_(1);
    double r = state_(2);
    double psi = state_(8);
    
    // Update position based on velocity (Euler integration)
    state_(6) += dt * (u * cos(psi) - v * sin(psi)); // North
    state_(7) += dt * (u * sin(psi) + v * cos(psi)); // East  
    state_(8) += dt * r;                             // Heading
    
    // Wrap heading angle
    while (state_(8) > M_PI) state_(8) -= 2 * M_PI;
    while (state_(8) < -M_PI) state_(8) += 2 * M_PI;
    
    // Simple process noise (would need proper process model)
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(10, 10) * 0.001;
    covariance_ += Q * dt;
    
    system_params_.time += dt;
}

void ExtendedKalmanFilter::setMeasurementNoise(double noise_std) {
    R_ = Eigen::Matrix2d::Identity() * (noise_std * noise_std);
}

bool ExtendedKalmanFilter::validateMeasurement(const AccelerometerMeasurement& measurement) {
    if (!measurement.is_valid) return false;
    
    // Check for reasonable acceleration values
    double accel_norm = measurement.acceleration.norm();
    if (accel_norm > 50.0) { // Max acceleration from config
        return false;
    }
    
    // Check for NaN or infinity
    if (!measurement.acceleration.allFinite()) {
        return false;
    }
    
    return true;
}

bool ExtendedKalmanFilter::validateState() {
    // Check for NaN or infinity in state
    if (!state_.allFinite()) {
        ROS_ERROR("State contains NaN or infinity");
        return false;
    }
    
    // Check for reasonable velocity bounds
    if (abs(state_(0)) > 50.0 || abs(state_(1)) > 50.0) { // u, v bounds
        ROS_WARN("Velocity states outside reasonable bounds: u=%.2f, v=%.2f", state_(0), state_(1));
        return false;
    }
    
    return true;
}

} // namespace qcar_visnav