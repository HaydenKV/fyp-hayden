// =====================================================================
// FILE: include/qcar_visnav/nodes/imu_node.h
// =====================================================================
#ifndef QCAR_VISNAV_IMU_NODE_H
#define QCAR_VISNAV_IMU_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "qcar_visnav/AccelerometerMeasurement.h"
#include "qcar_visnav/measurement/imu_processor.h"

namespace qcar_visnav {

class AccelerometerNode {
public:
    AccelerometerNode();
    ~AccelerometerNode() = default;
    
    void spin();

private:
    // ROS infrastructure
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Publishers and subscribers
    ros::Subscriber imu_sub_;
    ros::Publisher accel_pub_;
    
    // Processing
    AccelerometerProcessor accel_processor_;
    
    // Callbacks
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    // Configuration
    void loadParameters();
    
    // Diagnostics
    int message_count_;
    ros::Time last_message_time_;
    void publishDiagnostics();
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_IMU_NODE_H