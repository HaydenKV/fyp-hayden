#include "qcar_visnav/estimator/measurement_events.h"
#include "qcar_visnav/estimator/ekf_processor.h"

namespace qcar_visnav {

bool IMUEvent::process(EKFProcessor& processor, SystemParams& system) {
    // Equivalent to MATLAB MeasurementAccelerometer.update(system)
    
    if (!imu_data) {
        ROS_WARN("IMU event has no data at t=%.3f", time);
        return false;
    }
    
    // Convert IMU data to measurement
    AccelerometerMeasurement measurement;
    measurement.acceleration = Eigen::Vector3d(
        imu_data->linear_acceleration.x,
        imu_data->linear_acceleration.y,
        imu_data->linear_acceleration.z
    );
    measurement.timestamp = imu_data->header.stamp;
    measurement.variance = 0.01; // From parameters
    measurement.is_valid = true;
    
    // Validate measurement
    if (measurement.acceleration.norm() > 50.0 || !measurement.acceleration.allFinite()) {
        ROS_WARN("Invalid IMU measurement at t=%.3f", time);
        return false;
    }
    
    // Process through EKF - equivalent to MATLAB measurement update
    return processor.processAccelerometerMeasurement(measurement);
}

bool OdometryEvent::process(EKFProcessor& processor, SystemParams& system) {
    // Equivalent to MATLAB MeasurementEncoder.update(system)
    
    if (!odom_data) {
        ROS_WARN("Odometry event has no data at t=%.3f", time);
        return false;
    }
    
    // Convert odometry data to measurement
    OdometryMeasurement measurement;
    measurement.position = Eigen::Vector3d(
        odom_data->pose.pose.position.y, // North
        odom_data->pose.pose.position.x, // East
        0.0
    );
    measurement.velocity = Eigen::Vector3d(
        odom_data->twist.twist.linear.x,
        odom_data->twist.twist.linear.y,
        0.0
    );
    
    // Convert quaternion to heading
    tf2::Quaternion q;
    tf2::fromMsg(odom_data->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    measurement.heading = yaw;
    
    measurement.timestamp = odom_data->header.stamp;
    measurement.is_valid = true;
    
    return processor.processOdometryMeasurement(measurement);
}

bool LiDAREvent::process(EKFProcessor& processor, SystemParams& system) {
    // Equivalent to MATLAB MeasurementLiDAR.update(system)
    
    if (!lidar_data) {
        ROS_WARN("LiDAR event has no data at t=%.3f", time);
        return false;
    }
    
    // Process LiDAR scan
    return processor.processLiDARMeasurement(lidar_data);
}

bool ControlEvent::process(EKFProcessor& processor, SystemParams& system) {
    // Equivalent to MATLAB ControlEvent.update(system)
    // For now, just update system time and input
    
    // This would typically involve:
    // 1. Getting current reference trajectory
    // 2. Computing control input
    // 3. Updating system input parameters
    
    ROS_DEBUG("Control event processed at t=%.3f", time);
    return true;
}

} // namespace qcar_visnav