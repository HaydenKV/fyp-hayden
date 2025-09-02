// Enhanced motion_model.cpp
#include "qcar_visnav/slam/motion_model.h"
#include "qcar_visnav/slam/particle.h"
#include <random>
#include <cmath>
#include <algorithm>

namespace qcar_visnav { namespace slam {

void MotionModel::propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const
{
  if (dt <= 0.0) return;

  // Deterministic motion prediction
  predictMotion(p.x, p.y, p.yaw, vx_b, vy_b, r_b, dt);

  // **ENHANCED**: Adaptive noise based on motion characteristics
  const double velocity = std::sqrt(vx_b*vx_b + vy_b*vy_b);
  const double yaw_rate = std::abs(r_b);
  
  // Compute adaptive noise scaling
  const double vel_scale = getVelocityNoiseScale(velocity);
  const double turn_scale = getTurningNoiseScale(yaw_rate);
  const double combined_scale = std::sqrt(vel_scale * turn_scale);
  
  // Time-dependent noise scaling
  const double sdt = std::sqrt(dt);
  
  // Adaptive noise magnitudes with bounds
  double sigma_x_adaptive = std::min(noise_.sigma_x * combined_scale, 
                                    noise_.max_position_std / sdt);
  double sigma_y_adaptive = std::min(noise_.sigma_y * combined_scale,
                                    noise_.max_position_std / sdt);
  double sigma_yaw_adaptive = std::min(noise_.sigma_yaw * turn_scale,
                                      noise_.max_yaw_std / sdt);

  // Apply noise in world frame (after rotation)
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> n(0.0, 1.0);

  // **NEW**: Correlated noise for more realistic motion
  const double noise_x_world = sigma_x_adaptive * sdt * n(rng);
  const double noise_y_world = sigma_y_adaptive * sdt * n(rng);
  const double noise_yaw = sigma_yaw_adaptive * sdt * n(rng);

  p.x += noise_x_world;
  p.y += noise_y_world; 
  p.yaw += noise_yaw;
  
  // Normalize yaw
  while (p.yaw >  M_PI) p.yaw -= 2*M_PI;
  while (p.yaw < -M_PI) p.yaw += 2*M_PI;
}

void MotionModel::predictMotion(double& x, double& y, double& yaw,
                               double vx_b, double vy_b, double r_b, double dt) const
{
  // Body -> world rotation by current yaw
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double vx_w =  c*vx_b - s*vy_b;
  const double vy_w =  s*vx_b + c*vy_b;

  // **ENHANCED**: More accurate integration for high velocities/yaw rates
  const double velocity = std::sqrt(vx_b*vx_b + vy_b*vy_b);
  const double abs_yaw_rate = std::abs(r_b);
  
  if (abs_yaw_rate < 1e-3) {
    // Straight line motion - simple integration
    x += vx_w * dt;
    y += vy_w * dt;
    yaw += r_b * dt;
  } else {
    // Curved motion - use more accurate circular arc model
    const double R = velocity / abs_yaw_rate;  // Radius of curvature
    const double delta_theta = r_b * dt;
    
    // Arc integration in body frame, then transform to world
    const double dx_arc = velocity * dt * (std::sin(delta_theta) / delta_theta);
    const double dy_arc = velocity * dt * ((1.0 - std::cos(delta_theta)) / delta_theta);
    
    // Transform to world frame
    x += c * dx_arc - s * dy_arc * (r_b > 0 ? 1 : -1);
    y += s * dx_arc + c * dy_arc * (r_b > 0 ? 1 : -1);
    yaw += delta_theta;
  }
  
  // Normalize yaw
  while (yaw >  M_PI) yaw -= 2*M_PI;
  while (yaw < -M_PI) yaw += 2*M_PI;
}

Eigen::Matrix3d MotionModel::getMotionCovariance(double vx_b, double vy_b, double r_b, double dt) const
{
  const double velocity = std::sqrt(vx_b*vx_b + vy_b*vy_b);
  const double yaw_rate = std::abs(r_b);
  
  // Compute adaptive scaling
  const double vel_scale = getVelocityNoiseScale(velocity);
  const double turn_scale = getTurningNoiseScale(yaw_rate);
  const double combined_scale = std::sqrt(vel_scale * turn_scale);
  
  // Build covariance matrix
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  const double dt_factor = std::max(dt, 1e-6);
  
  // Adaptive diagonal covariance with bounds
  double var_x = std::min(std::pow(noise_.sigma_x * combined_scale, 2) * dt_factor,
                         std::pow(noise_.max_position_std, 2));
  double var_y = std::min(std::pow(noise_.sigma_y * combined_scale, 2) * dt_factor,
                         std::pow(noise_.max_position_std, 2));
  double var_yaw = std::min(std::pow(noise_.sigma_yaw * turn_scale, 2) * dt_factor,
                           std::pow(noise_.max_yaw_std, 2));
  
  Q(0,0) = var_x;
  Q(1,1) = var_y;
  Q(2,2) = var_yaw;
  
  // **NEW**: Add cross-correlation for turning motion
  if (yaw_rate > noise_.min_yawrate_for_turning) {
    const double cross_corr = 0.3 * std::sqrt(var_x * var_yaw) * (r_b / yaw_rate);
    Q(0,2) = Q(2,0) = cross_corr;  // x-yaw correlation during turns
  }
  
  return Q;
}

double MotionModel::getVelocityNoiseScale(double velocity) const
{
  if (velocity < noise_.min_velocity_for_scaling) {
    return 1.0;  // No velocity scaling for very low speeds
  }
  
  // Linear scaling with velocity: higher velocity -> more uncertainty
  const double normalized_vel = velocity / 2.0;  // Normalize around 2 m/s typical speed
  return 1.0 + noise_.velocity_noise_scale * normalized_vel;
}

double MotionModel::getTurningNoiseScale(double yaw_rate) const
{
  if (yaw_rate < noise_.min_yawrate_for_turning) {
    return 1.0;  // No turning scaling for straight motion
  }
  
  // Square root scaling with yaw rate: turning -> more uncertainty
  const double normalized_yaw_rate = yaw_rate / 1.0;  // Normalize around 1 rad/s
  return 1.0 + noise_.turning_noise_scale * std::sqrt(normalized_yaw_rate);
}

}} // namespace