#ifndef QCAR_VISNAV_ACCELEROMETER_PROCESSOR_H
#define QCAR_VISNAV_ACCELEROMETER_PROCESSOR_H

#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include "qcar_visnav/estimator/ekf_processor.h"

namespace qcar_visnav {

struct AccelerometerParameters {
    double noise_std_dev;      // Noise standard deviation (m/s²)
    double max_acceleration;   // Maximum reasonable acceleration (m/s²)
    
    AccelerometerParameters() : noise_std_dev(0.1), max_acceleration(50.0) {}
};

class AccelerometerProcessor {
public:
    AccelerometerProcessor();
    
    // Process raw IMU data into accelerometer measurement
    AccelerometerMeasurement processRawIMU(const sensor_msgs::Imu::ConstPtr& imu_msg);
    
    // Parameter management
    void setParameters(const AccelerometerParameters& params) { params_ = params; }
    AccelerometerParameters getParameters() const { return params_; }
    
    // Mounting parameters (from MATLAB MeasurementAccelerometer.m)
    void setMountingParameters(const Eigen::Vector3d& rMBb, const Eigen::Matrix3d& Rbm);
    Eigen::Vector3d getMountingPosition() const;
    Eigen::Matrix3d getMountingOrientation() const;
    
private:
    AccelerometerParameters params_;
    
    // Mounting parameters (from MATLAB)
    Eigen::Vector3d rMBb_;    // Position of accelerometer w.r.t B expressed in {b}
    Eigen::Matrix3d Rbm_;     // Accelerometer orientation matrix
    
    // Validation
    bool validateMeasurement(const Eigen::Vector3d& acceleration);
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_ACCELEROMETER_PROCESSOR_H