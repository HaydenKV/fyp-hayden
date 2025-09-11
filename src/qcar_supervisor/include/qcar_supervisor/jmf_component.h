#ifndef QCAR_VISNAV_ESTIMATION_JMF_JMF_COMPONENT_H
#define QCAR_VISNAV_ESTIMATION_JMF_JMF_COMPONENT_H

#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

/**
 * @brief Discrete sensor status for fault modeling
 */
struct SensorStatus {
  int acc;  // Accelerometer: 1 = healthy, 2 = faulty
  int vel;  // Velocity EKF:  1 = healthy, 2 = noisy

  SensorStatus() : acc(1), vel(1) {}  // Default to healthy
};

/**
 * @brief One Gaussian component in the Jump-Markov Filter mixture
 */
struct JmfComponent {
  double log_weight;                     // Log of mixture weight
  SensorStatus status;                  // Discrete sensor mode
  KinematicModel::StateVec mean;        // State mean
  KinematicModel::StateMat cov;         // State covariance

  JmfComponent() : log_weight(0.0), mean(KinematicModel::StateVec::Zero()),
                   cov(KinematicModel::StateMat::Identity()) {}
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_JMF_JMF_COMPONENT_H
