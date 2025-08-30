#pragma once
#include <Eigen/Core>
#include <cmath>

namespace qcar_visnav { namespace slam {

/**
 * LiDAR extrinsics (lidar frame relative to base frame).
 * These are fetched once from TF (base_frame -> lidar_frame).
 */
struct LidarExtrinsics {
  double x{0.0};     // lidar position in base frame [m]
  double y{0.0};
  double yaw{0.0};   // lidar yaw in base frame [rad]
  bool   valid{false};
};

/** Range-bearing measurement container (sensor space, LiDAR frame). */
struct MeasRB {
  double r;       // range [m]
  double b;       // bearing [rad]
  double r_var;   // range variance [m^2]
  double b_var;   // bearing variance [rad^2]
};

/** Utility: wrap angle to [-pi, pi]. */
inline double wrapToPi(double a) {
  while (a >  M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

/**
 * Predict measurement (r_hat, b_hat) and Jacobian H = d h / d m for a given
 * particle pose and landmark. H is 2x2 w.r.t. landmark state (in world).
 */
void predictMeasurementRB(double x, double y, double yaw,
                          const Eigen::Vector2d& m_world,
                          const LidarExtrinsics& ex,
                          double& r_hat, double& b_hat,
                          Eigen::Matrix2d& H);

/**
 * Convert a single range-bearing measurement in LiDAR frame into a world
 * landmark estimate using the particle pose and LiDAR extrinsics.
 * Also returns J = d m_world / d z (2x2) to initialize landmark covariance.
 */
void measRBToWorld(double x, double y, double yaw,
                   const LidarExtrinsics& ex,
                   double r, double b,
                   Eigen::Vector2d& m_world,
                   Eigen::Matrix2d& J);

}} // namespace
