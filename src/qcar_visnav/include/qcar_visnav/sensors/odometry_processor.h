#ifndef QCAR_VISNAV_ODOMETRY_PROCESSOR_H
#define QCAR_VISNAV_ODOMETRY_PROCESSOR_H

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include "qcar_visnav/estimator/ekf_processor.h"

namespace qcar_visnav {

struct OdometryParameters {
    double position_noise_std;    // Position measurement noise (m)
    double velocity_noise_std;    // Velocity measurement noise (m/s)
    double heading_noise_std;     // Heading measurement noise (rad)
    double max_velocity;          // Maximum reasonable velocity (m/s)
    
    OdometryParameters() : position_noise_std(0.01), velocity_noise_std(0.1), 
                          heading_noise_std(0.01), max_velocity(30.0) {}
};

class OdometryProcessor {
public:
    OdometryProcessor();
    
    // Process raw odometry data
    OdometryMeasurement processRawOdometry(const nav_msgs::Odometry::ConstPtr& odom_msg);
    
    // Parameter management
    void setParameters(const OdometryParameters& params) { params_ = params; }
    OdometryParameters getParameters() const { return params_; }
    
private:
    OdometryParameters params_;
    
    // Validation
    bool validateMeasurement(const OdometryMeasurement& measurement);
};

} // namespace qcar_visnav

#endif // QCAR_VISNAV_ODOMETRY_PROCESSOR_H