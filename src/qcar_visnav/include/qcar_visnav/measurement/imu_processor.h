#ifndef QCAR_VISNAV_IMU_PROCESSOR_H
#define QCAR_VISNAV_IMU_PROCESSOR_H

#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include "qcar_visnav/types/imu_data.h"

namespace qcar_visnav {

class AccelerometerProcessor {
public:
    AccelerometerProcessor();
    explicit AccelerometerProcessor(const AccelerometerParameters& params);
    
    AccelData processMeasurement(const sensor_msgs::Imu::ConstPtr& imu_msg);  // Changed return type
    void setParameters(const AccelerometerParameters& params);
    AccelerometerParameters getParameters() const { return params_; }
    
private:
    AccelerometerParameters params_;
    Eigen::Vector3d rosVectorToEigen(const geometry_msgs::Vector3& ros_vec);
    bool validateMeasurement(const Eigen::Vector3d& acceleration);
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_IMU_PROCESSOR_H