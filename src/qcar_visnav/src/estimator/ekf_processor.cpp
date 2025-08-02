#include "qcar_visnav/estimator/ekf_processor.h"
#include <ros/ros.h>

namespace qcar_visnav {

EKFProcessor::EKFProcessor() : verbosity_(1), last_prediction_time_(ros::Time::now()) {
    // Initialize current state
    current_state_.mean = Eigen::VectorXd::Zero(10);
    
    // Initialize covariance (from MATLAB SystemQCAR.m P0)
    Eigen::VectorXd P0_diag(10);
    P0_diag << 1e-2, 1e-2, 1e-1, 5e-2, 5e-2, 1e-3, 1e-2, 1e-2, 5e-2, 1e1;
    current_state_.covariance = P0_diag.asDiagonal();
    current_state_.valid = true;
    current_state_.timestamp = ros::Time::now();
}

void EKFProcessor::initialize(const SystemParams& params) {
    system_params_ = params;
    
    // Initialize EKF with parameters
    ekf_.initialize(current_state_.mean, current_state_.covariance);
    ekf_.setSystemParams(system_params_);
    
    // Set accelerometer mounting parameters (from MATLAB MeasurementAccelerometer.m)
    Eigen::Vector3d rMBb(0.1278, 0.0223, 0.0895);
    Eigen::Matrix3d Rbm = Eigen::Matrix3d::Identity();
    accel_predictor_.setMountingParams(rMBb, Rbm);
    
    ROS_INFO("EKF Processor initialized with %d-element state vector", (int)current_state_.mean.size());
}

void EKFProcessor::predict(double target_time) {
    // Time update - equivalent to MATLAB system.predict(target_time)
    double current_time = system_params_.time;
    double dt = target_time - current_time;
    
    if (dt > 0.001) { // Minimum time step
        ekf_.timeUpdate(dt);
        system_params_.time = target_time;
        updateCurrentState();
        
        ROS_DEBUG("Time prediction: t=%.3f, dt=%.3f", target_time, dt);
    }
}

bool EKFProcessor::processAccelerometerMeasurement(const AccelerometerMeasurement& measurement) {
    if (!validateAccelerometerMeasurement(measurement)) {
        ROS_WARN("Invalid accelerometer measurement");
        return false;
    }
    
    // Perform measurement update - equivalent to MATLAB MeasurementAccelerometer.update()
    bool success = ekf_.measurementUpdate(measurement);
    
    if (success) {
        updateCurrentState();
        ROS_DEBUG("Accelerometer update successful at t=%.3f", system_params_.time);
    }
    
    return success;
}

bool EKFProcessor::processOdometryMeasurement(const OdometryMeasurement& measurement) {
    if (!validateOdometryMeasurement(measurement)) {
        ROS_WARN("Invalid odometry measurement");
        return false;
    }
    
    // For now, basic implementation - would need full odometry measurement model
    ROS_DEBUG("Odometry measurement processed at t=%.3f", system_params_.time);
    return true;
}

bool EKFProcessor::processLiDARMeasurement(const sensor_msgs::LaserScan::ConstPtr& scan) {
    // Placeholder for LiDAR processing - would need full implementation
    ROS_DEBUG("LiDAR measurement processed at t=%.3f", system_params_.time);
    return true;
}

SystemState EKFProcessor::getCurrentState() const {
    return current_state_;
}

void EKFProcessor::updateCurrentState() {
    current_state_.mean = ekf_.getState();
    current_state_.covariance = ekf_.getCovariance();
    current_state_.timestamp = ros::Time::now();
    current_state_.valid = validateState();
}

bool EKFProcessor::validateAccelerometerMeasurement(const AccelerometerMeasurement& meas) {
    if (!meas.is_valid) return false;
    if (!meas.acceleration.allFinite()) return false;
    if (meas.acceleration.norm() > 50.0) return false; // Reasonable limit
    return true;
}

bool EKFProcessor::validateOdometryMeasurement(const OdometryMeasurement& meas) {
    if (!meas.is_valid) return false;
    if (!meas.position.allFinite() || !meas.velocity.allFinite()) return false;
    if (meas.velocity.norm() > 30.0) return false; // 30 m/s max velocity
    return true;
}

bool EKFProcessor::validateState() {
    if (!current_state_.mean.allFinite()) {
        ROS_ERROR("State contains NaN or infinity");
        return false;
    }
    
    // Check velocity bounds
    if (abs(current_state_.mean(0)) > 50.0 || abs(current_state_.mean(1)) > 50.0) {
        ROS_WARN("Velocity states outside bounds: u=%.2f, v=%.2f", 
                 current_state_.mean(0), current_state_.mean(1));
        return false;
    }
    
    return true;
}

} // namespace qcar_visnav