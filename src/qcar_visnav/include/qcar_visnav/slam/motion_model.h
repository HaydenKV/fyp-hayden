#pragma once
#include "particle.h"

namespace qcar_visnav { namespace slam {

struct MotionNoise {
  double sigma_x{0.02};
  double sigma_y{0.02};
  double sigma_yaw{0.005};
};

class MotionModel {
public:
  MotionModel() = default;
  explicit MotionModel(const MotionNoise& n): noise_(n) {}
  void propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const;
private:
  MotionNoise noise_;
};

}} // namespace
