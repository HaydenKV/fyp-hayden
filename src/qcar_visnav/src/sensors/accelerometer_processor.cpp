#include "qcar_visnav/sensors/accelerometer_processor.h"
#include <ros/ros.h>

namespace qcar_visnav {

AccelerometerProcessor::AccelerometerProcessor() {
    // Load default parameters
    params_.noise_std_dev = 0.1;        // From MATLAB noiseDensity.m
    params_.max_acceleration = 50.0;     // Validation limit
    
    // Mounting parameters from MATLAB MeasurementAccelerometer.m
    rMBb_ << 0.1278, 0.0223, 0.0895;
    Rbm_ = Eigen::Matrix3d::Identity();
}

AccelerometerMeasurement AccelerometerProcessor::processRawIMU(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    AccelerometerMeasurement measurement;
    measurement.timestamp = imu_msg->header.stamp;
    
    // Extract acceleration - minimal processing like MATLAB
    measurement.acceleration = Eigen::Vector3d(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z
    );
    
    // Set noise variance for EKF (from MATLAB noiseDensity.m)
    measurement.variance = params_.noise_std_dev * params_.noise_std_dev;
    
    // Validate measurement
    measurement.is_valid = validateMeasurement(measurement.acceleration);
    
    if (!measurement.is_valid) {
        ROS_WARN_THROTTLE(1.0, "Accelerometer validation failed: magnitude=%.2f", 
                         measurement.acceleration.norm());
    }
    
    return measurement;
}

bool AccelerometerProcessor::validateMeasurement(const Eigen::Vector3d& acceleration) {
    // Check magnitude
    double magnitude = acceleration.norm();
    if (magnitude > params_.max_acceleration) {
        return false;
    }
    
    // Check for NaN/infinity
    if (!acceleration.allFinite()) {
        return false;
    }
    
    return true;
}

void AccelerometerProcessor::setMountingParameters(const Eigen::Vector3d& rMBb, const Eigen::Matrix3d& Rbm) {
    rMBb_ = rMBb;
    Rbm_ = Rbm;
}

Eigen::Vector3d AccelerometerProcessor::getMountingPosition() const {
    return rMBb_;
}

Eigen::Matrix3d AccelerometerProcessor::getMountingOrientation() const {
    return Rbm_;
}

} // namespace qcar_visnav