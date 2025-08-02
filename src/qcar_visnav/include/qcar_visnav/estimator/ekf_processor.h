#ifndef QCAR_VISNAV_EKF_PROCESSOR_H
#define QCAR_VISNAV_EKF_PROCESSOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "qcar_visnav/math/system_params.h"
#include "qcar_visnav/math/accelerometer_prediction.h"
#include "qcar_visnav/math/ekf_filter.h"

namespace qcar_visnav {

// Measurement structures
struct AccelerometerMeasurement {
    Eigen::Vector3d acceleration;
    ros::Time timestamp;
    double variance;
    bool is_valid;
};

struct OdometryMeasurement {
    Eigen::Vector3d position;    // [N, E, 0]
    Eigen::Vector3d velocity;    // [u, v, 0] 
    double heading;              // psi
    ros::Time timestamp;
    bool is_valid;
};

// EKF Processing Engine - centralizes all estimation like MATLAB SystemQCAR
class EKFProcessor {
public:
    EKFProcessor();
    
    // Initialize system - equivalent to MATLAB SystemQCAR constructor
    void initialize(const SystemParams& params);
    
    // Time prediction - equivalent to MATLAB system.predict(time)
    void predict(double target_time);
    
    // Measurement updates - equivalent to MATLAB measurement.update(system)
    bool processAccelerometerMeasurement(const AccelerometerMeasurement& measurement);
    bool processOdometryMeasurement(const OdometryMeasurement& measurement);
    bool processLiDARMeasurement(const sensor_msgs::LaserScan::ConstPtr& scan);
    
    // State access
    SystemState getCurrentState() const;
    Eigen::VectorXd getStateMean() const { return current_state_.mean; }
    Eigen::MatrixXd getStateCovariance() const { return current_state_.covariance; }
    
    // System parameters
    void setSystemParams(const SystemParams& params) { system_params_ = params; }
    SystemParams getSystemParams() const { return system_params_; }
    
    // Debugging
    void setVerbosity(int level) { verbosity_ = level; }
    
private:
    // Core EKF
    ExtendedKalmanFilter ekf_;
    
    // Current system state - equivalent to MATLAB system.density
    SystemState current_state_;
    
    // System parameters
    SystemParams system_params_;
    
    // Measurement predictors
    AccelerometerPredictor accel_predictor_;
    
    // Timing
    ros::Time last_prediction_time_;
    
    // Debug
    int verbosity_;
    
    // Validation helpers
    bool validateAccelerometerMeasurement(const AccelerometerMeasurement& meas);
    bool validateOdometryMeasurement(const OdometryMeasurement& meas);
    
    // State management
    void updateCurrentState();
    bool validateState();
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_EKF_PROCESSOR_H