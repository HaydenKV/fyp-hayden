#pragma once
#include <Eigen/Dense>

namespace qcar_visnav { namespace slam {

// EKF landmark in WORLD frame (map_frame)
struct Landmark {
  Eigen::Vector2d mu{0.0, 0.0};
  Eigen::Matrix2d Sigma = Eigen::Matrix2d::Identity();

  int   hits{0};
  int   misses{0};
  bool  confirmed{false};

  // Leaned-out additions:
  bool  locked{false};   // once locked, we stop updating mu/Sigma
  int   id{-1};          // optional tracker ID (>=0 if known)
};

}} // namespace
