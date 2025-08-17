#ifndef QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H
#define QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H

#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

namespace qcar_nav {

/**
 * @brief Body-frame linear acceleration (gravity removed).
 * Biases (bax, bay) are handled by the EKF state.
 */
struct BodyAccel {
  double ax{0.0};  // forward accel [m/s^2]
  double ay{0.0};  // lateral accel [m/s^2]
  double az{0.0};  // vertical accel [m/s^2] (optional, unused now)
};

/**
 * @brief Compute body-frame linear acceleration from IMU, removing gravity.
 * @param imu_msg Incoming IMU message
 * @return BodyAccel with gravity-compensated accelerations in body frame
 */
BodyAccel removeGravity(const sensor_msgs::Imu& imu_msg);

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H
