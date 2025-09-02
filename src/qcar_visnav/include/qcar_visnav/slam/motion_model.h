// Enhanced motion_model.h
#pragma once
#include <Eigen/Core>
#include <cmath>

namespace qcar_visnav { namespace slam {

/**
 * Enhanced motion model with velocity-aware noise and constraints
 */
struct MotionNoise {
  // Base noise parameters
  double sigma_x{0.02};         // [m / sqrt(s)]
  double sigma_y{0.02};         // [m / sqrt(s)]  
  double sigma_yaw{0.005};      // [rad / sqrt(s)]
  
  // NEW: Velocity-dependent noise scaling
  double velocity_noise_scale{0.5};    // Scale noise with velocity
  double min_velocity_for_scaling{0.1}; // Minimum velocity to apply scaling
  
  // NEW: Turning-dependent noise
  double turning_noise_scale{1.0};     // Extra noise during turns
  double min_yawrate_for_turning{0.1}; // Minimum yaw rate for turning noise
  
  // NEW: Adaptive noise bounds
  double max_position_std{0.1};        // Maximum position noise per step
  double max_yaw_std{0.2};             // Maximum yaw noise per step
};

struct Particle;

class MotionModel {
public:
  MotionModel() = default;
  explicit MotionModel(const MotionNoise& n) : noise_(n) {}

  /**
   * Enhanced propagation with velocity-aware and adaptive noise
   */
  void propagate(Particle& p, double vx_b, double vy_b, double r_b, double dt) const;
  
  /**
   * Predict motion for pose proposal (deterministic part only)
   */
  void predictMotion(double& x, double& y, double& yaw, 
                    double vx_b, double vy_b, double r_b, double dt) const;

  /**
   * Get motion covariance matrix for pose proposal
   */
  Eigen::Matrix3d getMotionCovariance(double vx_b, double vy_b, double r_b, double dt) const;

  void setNoise(const MotionNoise& n) { noise_ = n; }
  const MotionNoise& noise() const { return noise_; }
  
private:
  MotionNoise noise_;
  
  // Compute adaptive noise scaling factors
  double getVelocityNoiseScale(double velocity) const;
  double getTurningNoiseScale(double yaw_rate) const;
};

}} // namespace