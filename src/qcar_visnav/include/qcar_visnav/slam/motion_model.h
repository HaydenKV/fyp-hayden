#pragma once
#include <Eigen/Dense>
#include "qcar_visnav/slam/particle.h"

namespace qcar_visnav { namespace slam {

struct MotionNoise {
  double sigma_x{0.04};    // [m]   per sqrt(s)
  double sigma_y{0.04};    // [m]   per sqrt(s)
  double sigma_yaw{0.01};  // [rad] per sqrt(s)
};

class MotionModel {
public:
  MotionModel() = default;
  explicit MotionModel(const MotionNoise& n) : noise_(n) {}

  // Circular-arc kinematics (body-frame v, r) -> world pose
  void propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const;

  // Diagonal covariance scaled by dt (fixed sigmas)
  Eigen::Matrix3d getMotionCovariance(double vx_b, double vy_b, double r_b, double dt) const;

private:
  void predictMotion(double& x, double& y, double& yaw,
                     double vx_b, double vy_b, double r_b, double dt) const;

  MotionNoise noise_;
};

}} // namespace
