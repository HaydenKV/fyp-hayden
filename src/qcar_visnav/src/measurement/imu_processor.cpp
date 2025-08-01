// =====================================================================
// FILE: src/measurement/imu_processor.cpp
// =====================================================================
#include "qcar_visnav/measurement/imu_processor.h"
#include <ros/ros.h>
#include <cmath>

namespace qcar_visnav {

AccelerometerProcessor::AccelerometerProcessor() : 
    params_(AccelerometerParameters()) {
}

AccelerometerProcessor::AccelerometerProcessor(const AccelerometerParameters& params) : 
    params_(params) {
}

AccelData AccelerometerProcessor::processMeasurement(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    AccelData measurement;  // Changed from AccelerometerMeasurement to AccelData
    measurement.timestamp = imu_msg->header.stamp;
    
    // Raw acceleration - no processing, just pass through
    measurement.acceleration = rosVectorToEigen(imu_msg->linear_acceleration);
    
    // Set measurement noise variance for EKF
    measurement.measurement_variance = params_.noise_std_dev * params_.noise_std_dev;
    
    // Basic validation only
    measurement.is_valid = validateMeasurement(measurement.acceleration);
    
    if (!measurement.is_valid) {
        ROS_WARN_THROTTLE(1.0, "Accelerometer measurement validation failed - acceleration magnitude: %.2f", 
                         measurement.acceleration.norm());
    }
    
    return measurement;
}

void AccelerometerProcessor::setParameters(const AccelerometerParameters& params) {
    params_ = params;
}

bool AccelerometerProcessor::validateMeasurement(const Eigen::Vector3d& acceleration) {
    // Check for reasonable acceleration magnitudes (including gravity)
    double accel_magnitude = acceleration.norm();
    
    if (accel_magnitude > params_.max_acceleration) {
        return false;
    }
    
    // Check for NaN or infinity
    if (!acceleration.allFinite()) {
        return false;
    }
    
    return true;
}

// Helper function for ROS -> Eigen conversion
Eigen::Vector3d AccelerometerProcessor::rosVectorToEigen(const geometry_msgs::Vector3& ros_vec) {
    return Eigen::Vector3d(ros_vec.x, ros_vec.y, ros_vec.z);
}

} // namespace qcar_visnav