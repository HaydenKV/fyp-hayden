#pragma once
#include <Eigen/Core>
#include <cmath>

namespace qcar_visnav { namespace slam {

/**
 * Diffusive motion model used by the particle filter.
 * Adds zero-mean Gaussian noise ~ N(0, σ * sqrt(dt)) independently
 * on x, y, and yaw each propagation step.
 */
struct MotionNoise {
  double sigma_x{0.02};   // [m / sqrt(s)]
  double sigma_y{0.02};   // [m / sqrt(s)]
  double sigma_yaw{0.005}; // [rad / sqrt(s)]
};

struct Particle;

class MotionModel {
public:
  MotionModel() = default;
  explicit MotionModel(const MotionNoise& n) : noise_(n) {}

  // Propagate one particle forward given body-frame twist (vx, vy, r) over dt.
  void propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const;

  // Configure noise parameters.
  void setNoise(const MotionNoise& n) { noise_ = n; }

private:
  MotionNoise noise_;
};

}} // namespace
