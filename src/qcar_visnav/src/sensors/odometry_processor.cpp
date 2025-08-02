#include "qcar_visnav/sensors/odometry_processor.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace qcar_visnav {

OdometryProcessor::OdometryProcessor() {
    // Default parameters
    params_.position_noise_std = 0.01;   // 1 cm position noise
    params_.velocity_noise_std = 0.1;    // 0.1 m/s velocity noise
    params_.heading_noise_std = 0.01;    // 0.01 rad heading noise
    params_.max_velocity = 30.0;         // 30 m/s maximum velocity
}

OdometryMeasurement OdometryProcessor::processRawOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg) {
    OdometryMeasurement measurement;
    measurement.timestamp = odom_msg->header.stamp;
    
    // Extract position (swap x,y to N,E convention)
    measurement.position = Eigen::Vector3d(
        odom_msg->pose.pose.position.y,  // North
        odom_msg->pose.pose.position.x,  // East
        odom_msg->pose.pose.position.z   // Up
    );
    
    // Extract velocity
    measurement.velocity = Eigen::Vector3d(
        odom_msg->twist.twist.linear.x,
        odom_msg->twist.twist.linear.y,
        odom_msg->twist.twist.linear.z
    );
    
    // Extract heading from quaternion
    tf2::Quaternion q;
    tf2::fromMsg(odom_msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    measurement.heading = yaw;
    
    // Validate measurement
    measurement.is_valid = validateMeasurement(measurement);
    
    if (!measurement.is_valid) {
        ROS_WARN_THROTTLE(1.0, "Odometry validation failed: vel_norm=%.2f, pos_finite=%d", 
                         measurement.velocity.norm(), measurement.position.allFinite());
    }
    
    return measurement;
}

bool OdometryProcessor::validateMeasurement(const OdometryMeasurement& measurement) {
    // Check velocity bounds
    if (measurement.velocity.norm() > params_.max_velocity) {
        return false;
    }
    
    // Check for NaN/infinity
    if (!measurement.position.allFinite() || !measurement.velocity.allFinite()) {
        return false;
    }
    
    // Check heading range
    if (abs(measurement.heading) > M_PI + 0.1) { // Small tolerance
        return false;
    }
    
    return true;
}

} // namespace qcar_visnav