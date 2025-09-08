#include "qcar_visnav/slam/motion_model.h"
#include <random>
#include <cmath>
#include <algorithm>

namespace qcar_visnav { namespace slam {

static inline void normalizeYaw(double& a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
}

void MotionModel::predictMotion(double& x, double& y, double& yaw,
                                double vx_b, double vy_b, double r_b, double dt) const
{
  if (dt <= 0.0) return;

  const double c = std::cos(yaw), s = std::sin(yaw);

  if (std::abs(r_b) < 1e-6) {
    // Pure translation in body, rotate to world
    const double dx_w = ( c * vx_b - s * vy_b) * dt;
    const double dy_w = ( s * vx_b + c * vy_b) * dt;
    x += dx_w; y += dy_w;
    yaw += r_b * dt;
  } else {
    // SE(2) exact integration for constant body twist (vx_b, vy_b, r_b)
    const double wdt = r_b * dt;
    const double sw = std::sin(wdt), cw = std::cos(wdt);

    // Body-frame integration matrix V(w, dt)
    // Δp_body = [ sin(wdt)/w   -(1-cos(wdt))/w ] [vx_b]
    //           [ (1-cos(wdt))/w  sin(wdt)/w   ] [vy_b]
    const double invw = 1.0 / r_b;
    const double a11 =  sw * invw, a12 = -(1.0 - cw) * invw;
    const double a21 = (1.0 - cw) * invw, a22 =   sw * invw;

    const double dxb = a11 * vx_b + a12 * vy_b;
    const double dyb = a21 * vx_b + a22 * vy_b;

    // Rotate body increment to world
    const double dxw =  c * dxb - s * dyb;
    const double dyw =  s * dxb + c * dyb;

    x += dxw; y += dyw; yaw += wdt;
  }

  normalizeYaw(yaw);
}

void MotionModel::propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const
{
  if (dt <= 0.0) return;

  // Deterministic motion
  predictMotion(p.x, p.y, p.yaw, vx_b, vy_b, r_b, dt);

  // Add fixed (diagonal) process noise in world frame; std ~ sigma * sqrt(dt)
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> N01(0.0, 1.0);

  const double sdt = std::sqrt(std::max(0.0, dt));
  const double nx  = noise_.sigma_x   * sdt * N01(rng);
  const double ny  = noise_.sigma_y   * sdt * N01(rng);
  const double nyw = noise_.sigma_yaw * sdt * N01(rng);

  p.x   += nx;
  p.y   += ny;
  p.yaw += nyw;
  if (p.yaw >  M_PI) p.yaw -= 2.0 * M_PI;
  if (p.yaw < -M_PI) p.yaw += 2.0 * M_PI;
}

Eigen::Matrix3d MotionModel::getMotionCovariance(double /*vx_b*/, double /*vy_b*/, double /*r_b*/, double dt) const
{
  // Diagonal covariance with variance proportional to dt (white noise)
  const double dtp = std::max(dt, 1e-6);
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  Q(0,0) = noise_.sigma_x   * noise_.sigma_x   * dtp;
  Q(1,1) = noise_.sigma_y   * noise_.sigma_y   * dtp;
  Q(2,2) = noise_.sigma_yaw * noise_.sigma_yaw * dtp;
  return Q;
}

}} // namespace
