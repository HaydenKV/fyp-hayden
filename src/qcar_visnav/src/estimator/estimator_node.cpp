#include "qcar_visnav/estimator/estimator_node.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace qcar_visnav {

EstimatorNode::EstimatorNode() : 
    private_nh_("~"),
    running_(false) {
    
    // Load parameters
    loadParameters();
    
    // Initialize system parameters
    system_params_.time = 0.0;
    system_params_.g = 9.81;
    
    // Initialize EKF processor
    ekf_processor_.initialize(system_params_);
    ekf_processor_.setVerbosity(verbosity_);
    
    // Set up subscribers - collect sensor data into buffers
    imu_sub_ = nh_.subscribe("/qcar/imu", 100, &EstimatorNode::imuCallback, this);
    odom_sub_ = nh_.subscribe("/qcar/ground_truth/odom", 100, &EstimatorNode::odomCallback, this);
    lidar_sub_ = nh_.subscribe("/qcar/lidar", 10, &EstimatorNode::lidarCallback, this);
    
    // Set up publishers
    state_pub_ = nh_.advertise<qcar_visnav::EstimatorState>("/qcar_visnav/estimator_state", 1);
    ekf_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/qcar_visnav/ekf_odom", 1);
    
    if (publish_debug_) {
        debug_pub_ = nh_.advertise<qcar_visnav::EventDebug>("/qcar_visnav/event_debug", 1);
    }
    
    // Initialize event queue - equivalent to MATLAB event queue creation
    initializeEventQueue();
    
    ROS_INFO("Estimator Node initialized");
    ROS_INFO("Event queue size: %zu events", event_queue_.size());
    ROS_INFO("Simulation time: %.2f seconds", simulation_time_);
}

EstimatorNode::~EstimatorNode() {
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void EstimatorNode::spin() {
    // Start processing thread for event queue
    running_ = true;
    processing_thread_ = std::thread(&EstimatorNode::processingLoop, this);
    
    // Main ROS spin for callbacks
    ros::Rate rate(100); // High rate for sensor data collection
    
    while (ros::ok() && running_) {
        ros::spinOnce();
        
        // Integrate new sensor data into pending events
        integrateIMUData();
        integrateOdomData();
        integrateLiDARData();
        
        // Publish current state
        publishState();
        
        rate.sleep();
    }
    
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
}

void EstimatorNode::processingLoop() {
    // Main event processing loop - equivalent to MATLAB event loop
    ros::Rate processing_rate(1000); // High rate for precise timing
    
    while (running_ && ros::ok()) {
        processEventQueue();
        processing_rate.sleep();
    }
}

void EstimatorNode::processEventQueue() {
    // Process events in chronological order - equivalent to MATLAB event loop
    double current_sim_time = (ros::Time::now() - ros::Time::now()).toSec(); // Relative time
    
    while (!event_queue_.empty() && running_) {
        double next_event_time = event_queue_.nextEventTime();
        
        // Check if it's time to process the next event
        if (next_event_time <= current_sim_time) {
            // Process next event - equivalent to MATLAB [obj, system] = process(obj, system)
            bool success = event_queue_.processNextEvent(ekf_processor_, system_params_);
            
            if (publish_debug_ && debug_pub_.getNumSubscribers() > 0) {
                publishDebugInfo();
            }
        } else {
            break; // Wait for next event time
        }
    }
}

void EstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    imu_buffer_.push(msg);
    
    // Keep buffer size reasonable
    while (imu_buffer_.size() > 10) {
        imu_buffer_.pop();
    }
}

void EstimatorNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    odom_buffer_.push(msg);
    
    while (odom_buffer_.size() > 10) {
        odom_buffer_.pop();
    }
}

void EstimatorNode::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    lidar_buffer_.push(msg);
    
    while (lidar_buffer_.size() > 5) {
        lidar_buffer_.pop();
    }
}

void EstimatorNode::integrateIMUData() {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    
    // Associate IMU data with pending IMU events
    while (!imu_buffer_.empty()) {
        auto imu_msg = imu_buffer_.front();
        imu_buffer_.pop();
        
        // Find the closest pending IMU event and associate data
        // This would require a more sophisticated event-data association system
        // For now, just process immediately
        
        AccelerometerMeasurement measurement;
        measurement.acceleration = Eigen::Vector3d(
            imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y, 
            imu_msg->linear_acceleration.z
        );
        measurement.timestamp = imu_msg->header.stamp;
        measurement.variance = 0.01; // From config
        measurement.is_valid = true;
        
        ekf_processor_.processAccelerometerMeasurement(measurement);
    }
}

void EstimatorNode::integrateOdomData() {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    
    while (!odom_buffer_.empty()) {
        auto odom_msg = odom_buffer_.front();
        odom_buffer_.pop();
        
        OdometryMeasurement measurement;
        measurement.position = Eigen::Vector3d(
            odom_msg->pose.pose.position.y, // North
            odom_msg->pose.pose.position.x, // East  
            0.0
        );
        measurement.velocity = Eigen::Vector3d(
            odom_msg->twist.twist.linear.x,
            odom_msg->twist.twist.linear.y,
            0.0
        );
        
        // Convert quaternion to yaw
        tf2::Quaternion q;
        tf2::fromMsg(odom_msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        measurement.heading = yaw;
        
        measurement.timestamp = odom_msg->header.stamp;
        measurement.is_valid = true;
        
        ekf_processor_.processOdometryMeasurement(measurement);
    }
}

void EstimatorNode::integrateLiDARData() {
    std::lock_guard<std::mutex> lock(sensor_data_mutex_);
    
    while (!lidar_buffer_.empty()) {
        auto lidar_msg = lidar_buffer_.front();
        lidar_buffer_.pop();
        
        ekf_processor_.processLiDARMeasurement(lidar_msg);
    }
}

void EstimatorNode::publishState() {
    SystemState state = ekf_processor_.getCurrentState();
    
    if (!state.valid) return;
    
    // Publish EstimatorState message
    qcar_visnav::EstimatorState state_msg;
    state_msg.header.stamp = state.timestamp;
    state_msg.header.frame_id = "base_link";
    
    // Fill state vector
    state_msg.u = state.mean(0);      // Forward velocity
    state_msg.v = state.mean(1);      // Lateral velocity
    state_msg.r = state.mean(2);      // Yaw rate
    state_msg.wF = state.mean(3);     // Front wheel speed
    state_msg.wR = state.mean(4);     // Rear wheel speed
    state_msg.delta = state.mean(5);  // Steering angle
    state_msg.N = state.mean(6);      // North position
    state_msg.E = state.mean(7);      // East position
    state_msg.psi = state.mean(8);    // Heading
    state_msg.gyro = state.mean(9);   // Gyro bias
    
    // Flatten covariance matrix
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) {
            state_msg.covariance[i*10 + j] = state.covariance(i,j);
        }
    }
    
    state_pub_.publish(state_msg);
    
    // Also publish as odometry for compatibility
    nav_msgs::Odometry odom_msg;
    odom_msg.header = state_msg.header;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = state.mean(7); // East
    odom_msg.pose.pose.position.y = state.mean(6); // North
    odom_msg.pose.pose.position.z = 0.0;
    
    // Convert heading to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, state.mean(8));
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    odom_msg.twist.twist.linear.x = state.mean(0);  // u
    odom_msg.twist.twist.linear.y = state.mean(1);  // v
    odom_msg.twist.twist.angular.z = state.mean(2); // r
    
    ekf_odom_pub_.publish(odom_msg);
}

void EstimatorNode::publishDebugInfo() {
    if (!publish_debug_) return;
    
    qcar_visnav::EventDebug debug_msg;
    debug_msg.header.stamp = ros::Time::now();
    debug_msg.current_time = system_params_.time;
    debug_msg.queue_size = event_queue_.size();
    debug_msg.next_event_time = event_queue_.nextEventTime();
    
    debug_pub_.publish(debug_msg);
}

void EstimatorNode::loadParameters() {
    private_nh_.param("simulation_time", simulation_time_, 60.0);
    private_nh_.param("run_estimator", run_estimator_, true);
    private_nh_.param("publish_debug", publish_debug_, false);
    private_nh_.param("verbosity", verbosity_, 1);
    
    ROS_INFO("Estimator parameters: sim_time=%.1fs, debug=%s, verbosity=%d",
             simulation_time_, publish_debug_ ? "true" : "false", verbosity_);
}

void EstimatorNode::initializeEventQueue() {
    // Schedule all events - equivalent to MATLAB event queue creation
    event_queue_.schedulePeriodicEvents(simulation_time_);
}

} // namespace qcar_visnav