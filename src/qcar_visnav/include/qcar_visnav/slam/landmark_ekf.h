#pragma once
#include <Eigen/Dense>

namespace qcar_visnav {
namespace slam {

// Static-landmark EKF with H = I (we fuse 2D position directly in odom)
inline void ekf_update_landmark(
    Eigen::Vector2d& mu,
    Eigen::Matrix2d& Sigma,
    const Eigen::Vector2d& z,
    const Eigen::Matrix2d& R)
{
  const Eigen::Matrix2d S = Sigma + R;        // H = I
  const Eigen::Matrix2d K = Sigma * S.inverse();
  mu += K * (z - mu);
  Sigma = (Eigen::Matrix2d::Identity() - K) * Sigma;
}

} // namespace slam
} // namespace qcar_visnav
