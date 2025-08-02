#ifndef QCAR_VISNAV_ESTIMATOR_NODE_H
#define QCAR_VISNAV_ESTIMATOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <thread>
#include <atomic>
#include <mutex>

#include "qcar_visnav/estimator/event_queue.h"
#include "qcar_visnav/estimator/ekf_processor.h"
#include "qcar_visnav/estimator/measurement_events.h"
#include "qcar_visnav/EstimatorState.h"
#include "qcar_visnav/EventDebug.h"

namespace qcar_visnav {

// Main Estimator Node - equivalent to MATLAB runSim.m
class EstimatorNode {
public:
    EstimatorNode();
    ~EstimatorNode();
    
    // Main execution - equivalent to MATLAB event loop
    void spin();
    
private:
    // ROS infrastructure
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // Subscribers - collect sensor data
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber lidar_sub_;
    
    // Publishers - output estimation results
    ros::Publisher state_pub_;
    ros::Publisher ekf_odom_pub_;
    ros::Publisher debug_pub_;
    
    // Core components
    EventQueue event_queue_;          // Priority event queue (like MATLAB event_queue)
    EKFProcessor ekf_processor_;      // EKF processing engine
    SystemParams system_params_;     // System parameters
    
    // Threading for real-time processing
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::mutex sensor_data_mutex_;
    
    // Sensor data buffers
    std::queue<sensor_msgs::Imu::ConstPtr> imu_buffer_;
    std::queue<nav_msgs::Odometry::ConstPtr> odom_buffer_;
    std::queue<sensor_msgs::LaserScan::ConstPtr> lidar_buffer_;
    
    // Parameters
    double simulation_time_;
    bool run_estimator_;
    bool publish_debug_;
    int verbosity_;
    
    // Callbacks - collect sensor data (don't process immediately)
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    
    // Main processing thread - equivalent to MATLAB event loop
    void processingLoop();
    
    // Event processing - equivalent to MATLAB [obj, system] = process(obj, system)
    void processEventQueue();
    
    // Sensor data integration into events
    void integrateIMUData();
    void integrateOdomData();
    void integrateLiDARData();
    
    // Publishing
    void publishState();
    void publishDebugInfo();
    
    // Parameter loading
    void loadParameters();
    
    // Initialize event queue - equivalent to MATLAB event queue creation
    void initializeEventQueue();
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_ESTIMATOR_NODE_H