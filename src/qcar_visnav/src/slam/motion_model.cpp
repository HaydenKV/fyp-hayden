#include "qcar_visnav/slam/motion_model.h"
#include "qcar_visnav/slam/particle.h"   // <-- needed for p.x/p.y/p.yaw
#include <random>
#include <cmath>

namespace qcar_visnav { namespace slam {

void MotionModel::propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const
{
  if (dt <= 0.0) return;

  // Body -> world rotation by particle yaw
  const double c = std::cos(p.yaw), s = std::sin(p.yaw);
  const double vx_w =  c*vx_b - s*vy_b;
  const double vy_w =  s*vx_b + c*vy_b;

  // Nominal integration
  p.x   += vx_w * dt;
  p.y   += vy_w * dt;
  p.yaw += r_b * dt;

  // Normalize yaw
  if (p.yaw >  M_PI) p.yaw -= 2*M_PI;
  if (p.yaw < -M_PI) p.yaw += 2*M_PI;

  // Diffusive noise ~ sqrt(dt)
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> n(0.0, 1.0);

  const double sdt = std::sqrt(dt);
  p.x   += noise_.sigma_x   * sdt * n(rng);
  p.y   += noise_.sigma_y   * sdt * n(rng);
  p.yaw += noise_.sigma_yaw * sdt * n(rng);
}

}} // namespace
