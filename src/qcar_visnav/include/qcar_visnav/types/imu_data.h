#ifndef QCAR_VISNAV_IMU_DATA_H
#define QCAR_VISNAV_IMU_DATA_H

#include <Eigen/Dense>
#include <ros/time.h>

namespace qcar_visnav {

struct AccelData {  // Renamed to avoid conflict with ROS message
    ros::Time timestamp;
    Eigen::Vector3d acceleration;
    double measurement_variance;
    bool is_valid;
    
    AccelData() : 
        acceleration(Eigen::Vector3d::Zero()),
        measurement_variance(0.01),
        is_valid(false) {}
};

struct AccelerometerParameters {
    double noise_std_dev;
    double max_acceleration;
    
    AccelerometerParameters() :
        noise_std_dev(0.1),
        max_acceleration(50.0) {}
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_IMU_DATA_H