#include "qcar_visnav/estimation/sensors/accelerometer.h"

namespace qcar_nav {

BodyAccel removeGravity(const sensor_msgs::Imu& imu_msg) {
  // Orientation quaternion (world_from_body)
  tf::Quaternion q;
  tf::quaternionMsgToTF(imu_msg.orientation, q);
  tf::Matrix3x3 R(q);

  // Raw accel in body frame
  tf::Vector3 a_b(
      imu_msg.linear_acceleration.x,
      imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z);

  // Gravity in world frame
  const tf::Vector3 g_w(0.0, 0.0, 9.81);

  // Gravity expressed in body frame: g_b = R^T * g_w
  tf::Vector3 g_b = R.transpose() * g_w;

  // Subtract gravity
  tf::Vector3 a_lin_b = a_b - g_b;

  BodyAccel out;
  out.ax = a_lin_b.x();
  out.ay = a_lin_b.y();
  out.az = a_lin_b.z();
  return out;
}

} // namespace qcar_nav
