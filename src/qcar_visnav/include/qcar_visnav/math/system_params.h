#ifndef QCAR_VISNAV_SYSTEM_PARAMS_H
#define QCAR_VISNAV_SYSTEM_PARAMS_H

#include <Eigen/Dense>
#include <ros/ros.h>

namespace qcar_visnav {

struct SystemParams {
    double g;                    // Gravity constant
    double time;                 // Current simulation time
    Eigen::VectorXd input;       // Control input [throttle, steering]
    
    // Timing parameters (from MATLAB runSim.m)
    double dt_ctrl;              // Control time step
    double dt_imu;               // IMU time step  
    double dt_lidar;             // LiDAR time step
    
    SystemParams() : g(9.81), time(0.0) {
        input = Eigen::VectorXd::Zero(2);
        dt_ctrl = 1.0/50.0;      // 50 Hz control
        dt_imu = 1.0/50.0;       // 50 Hz IMU  
        dt_lidar = 1.0/10.0;     // 10 Hz LiDAR
    }
};

struct SystemState {
    Eigen::VectorXd mean;        // State estimate [u; v; r; wF; wR; delta; N; E; psi; gyro]
    Eigen::MatrixXd covariance;  // State covariance P
    ros::Time timestamp;         // State timestamp
    bool valid;                  // State validity
    
    SystemState() : valid(false) {
        mean = Eigen::VectorXd::Zero(10);
        covariance = Eigen::MatrixXd::Identity(10, 10);
        timestamp = ros::Time::now();
    }
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_SYSTEM_PARAMS_H