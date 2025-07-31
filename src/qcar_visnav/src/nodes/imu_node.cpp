// =====================================================================
// FILE: src/nodes/imu_node.cpp
// =====================================================================
#include "qcar_visnav/nodes/imu_node.h"
#include <ros/ros.h>

namespace qcar_visnav {

AccelerometerNode::AccelerometerNode() : 
    private_nh_("~"),
    message_count_(0),
    last_message_time_(ros::Time::now()) {
    
    // Load parameters from ROS parameter server
    loadParameters();
    
    // Set up subscribers and publishers
    imu_sub_ = nh_.subscribe("/imu", 1, &AccelerometerNode::imuCallback, this);
    accel_pub_ = nh_.advertise<qcar_visnav::AccelerometerMeasurement>("/qcar_visnav/accelerometer", 1);
    
    ROS_INFO("Accelerometer processing node initialized");
    ROS_INFO("Subscribing to: %s", imu_sub_.getTopic().c_str());
    ROS_INFO("Publishing to: %s", accel_pub_.getTopic().c_str());
}

void AccelerometerNode::spin() {
    ros::Rate rate(100); // 100 Hz processing rate
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // Publish diagnostics occasionally
        if (message_count_ % 100 == 0 && message_count_ > 0) {
            publishDiagnostics();
        }
        
        rate.sleep();
    }
}

void AccelerometerNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Process the IMU message (minimal processing)
    AccelerometerMeasurementData measurement = accel_processor_.processMeasurement(msg);
    
    // Convert to ROS message and publish
    qcar_visnav::AccelerometerMeasurement accel_msg;
    accel_msg.header = msg->header;
    accel_msg.header.frame_id = "base_link"; // Vehicle frame
    
    // Fill in the data
    accel_msg.acceleration.x = measurement.acceleration.x();
    accel_msg.acceleration.y = measurement.acceleration.y();
    accel_msg.acceleration.z = measurement.acceleration.z();
    accel_msg.measurement_variance = measurement.measurement_variance;
    accel_msg.is_valid = measurement.is_valid;
    
    // Publish the raw measurement data for EKF
    accel_pub_.publish(accel_msg);
    
    // Update diagnostics
    message_count_++;
    last_message_time_ = ros::Time::now();
}

void AccelerometerNode::loadParameters() {
    AccelerometerParameters params;
    
    // Load parameters from ROS parameter server
    private_nh_.param("noise_std_dev", params.noise_std_dev, 0.1);
    private_nh_.param("max_acceleration", params.max_acceleration, 50.0);
    
    // Set parameters in processor
    accel_processor_.setParameters(params);
    
    ROS_INFO("Accelerometer Parameters loaded:");
    ROS_INFO("  Noise std dev: %.3f m/s²", params.noise_std_dev);
    ROS_INFO("  Max acceleration: %.1f m/s²", params.max_acceleration);
}

void AccelerometerNode::publishDiagnostics() {
    double time_since_last = (ros::Time::now() - last_message_time_).toSec();
    double processing_rate = message_count_ / (ros::Time::now().toSec() - ros::Time(0).toSec());
    
    ROS_INFO_THROTTLE(5.0, "Accelerometer processing: %d messages, %.1f Hz, last message %.2fs ago",
                      message_count_, processing_rate, time_since_last);
}

} // namespace qcar_visnav