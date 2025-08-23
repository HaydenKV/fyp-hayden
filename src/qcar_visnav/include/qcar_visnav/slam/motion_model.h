#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace qcar_visnav {
namespace slam {

// Simple (v, yawrate) planar integrator in odom
inline void propagate_pose(
    Eigen::Vector3d& pose,   // [x,y,yaw]
    double v_mps,            // linear speed
    double yawrate_rps,      // yaw rate
    double dt)
{
  if (dt <= 0.0) return;
  const double th0 = pose.z();
  const double dth = yawrate_rps * dt;
  if (std::fabs(dth) < 1e-6) {
    pose.x() += v_mps * std::cos(th0) * dt;
    pose.y() += v_mps * std::sin(th0) * dt;
  } else {
    const double R = v_mps / yawrate_rps;
    pose.x() += R * (std::sin(th0 + dth) - std::sin(th0));
    pose.y() += -R * (std::cos(th0 + dth) - std::cos(th0));
  }
  pose.z() = th0 + dth;
  while (pose.z() >  M_PI) pose.z() -= 2*M_PI;
  while (pose.z() < -M_PI) pose.z() += 2*M_PI;
}

} // namespace slam
} // namespace qcar_visnav
