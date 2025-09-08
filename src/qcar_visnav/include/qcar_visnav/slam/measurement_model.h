#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace qcar_visnav { namespace slam {

// --- Sensor structs ---
struct LidarExtrinsics {
  double x{0.0}, y{0.0}, yaw{0.0};
  bool   valid{false};
};

struct MeasRB {
  double r{0.0}, b{0.0};
  double r_var{1e-3}, b_var{1e-3};
  int    id{-1};
  bool   has_id{false};
};

// --- Angle wrap ---
inline double wrapToPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// --- Prototypes implemented in measurement_model.cpp ---
void predictMeasurementRB(double x, double y, double yaw,
                          const Eigen::Vector2d& m_world,
                          const LidarExtrinsics& ex,
                          double& r_hat, double& b_hat,
                          Eigen::Matrix2d& H);

void measRBToWorld(double x, double y, double yaw,
                   const LidarExtrinsics& ex,
                   double r, double b,
                   Eigen::Vector2d& m_world,
                   Eigen::Matrix2d& J);

// (Optional extended Jacobians used elsewhere; keep declaration for compatibility)
void predictRBWithJacobians(double x, double y, double yaw,
                            const Eigen::Vector2d& m_world,
                            const LidarExtrinsics& ex,
                            double& r_hat, double& b_hat,
                            Eigen::Matrix<double,2,3>& Gx,
                            Eigen::Matrix2d& H);

}} // namespace
