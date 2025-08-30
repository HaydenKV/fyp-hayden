#pragma once
#include <vector>
#include <Eigen/Dense>
#include "landmark.h"

namespace qcar_visnav { namespace slam {

// Transient "birth" track for K-hit promotion into a real landmark
struct BirthTrack {
  Eigen::Vector2d mu{0,0};
  Eigen::Matrix2d Sigma = 0.05 * Eigen::Matrix2d::Identity(); // world-pos KF
  int hits{0};  // times reinforced
  int age{0};   // frames since created/last reinforced
};

struct Particle {
  // Pose
  double x{0.0}, y{0.0}, yaw{0.0};

  // Importance weight
  double weight{0.0};
  double log_w{0.0};

  int    id{0};

  // Map & births
  std::vector<Landmark>   map;    // EKF landmarks in world frame
  std::vector<BirthTrack> births; // pending births (for promotion)
};

}} // namespace
