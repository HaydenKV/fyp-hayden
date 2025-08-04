#ifndef QCAR_VISNAV_EKF_FILTER_H
#define QCAR_VISNAV_EKF_FILTER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include "qcar_visnav/math/accelerometer_prediction.h"
#include "qcar_visnav/math/system_params.h"

namespace qcar_visnav {

struct AccelerometerMeasurement {
    Eigen::Vector3d acceleration;
    ros::Time timestamp;
    double variance;
    bool is_valid;
};

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    
    // Initialize EKF with state and covariance
    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& initial_covariance);
    
    // Measurement update - ports MATLAB update mechanism
    bool measurementUpdate(const AccelerometerMeasurement& measurement);
    
    // Time prediction step
    void timeUpdate(double dt);
    
    // Getters
    Eigen::VectorXd getState() const { return state_; }
    Eigen::MatrixXd getCovariance() const { return covariance_; }
    
    // Set parameters
    void setSystemParams(const SystemParams& params) { system_params_ = params; }
    void setMeasurementNoise(double noise_std);
    
private:
    // State and covariance
    Eigen::VectorXd state_;           // [u; v; r; wF; wR; delta; N; E; psi; gyro]
    Eigen::MatrixXd covariance_;      // P matrix
    
    // System parameters
    SystemParams system_params_;
    
    // Measurement noise
    Eigen::Matrix2d R_;               // Measurement noise covariance
    
    // Predictor
    AccelerometerPredictor predictor_;
    
    // Validation
    bool validateMeasurement(const AccelerometerMeasurement& measurement);
    bool validateState();
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_EKF_FILTER_H