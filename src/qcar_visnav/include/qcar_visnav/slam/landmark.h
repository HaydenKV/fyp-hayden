#pragma once
#include <Eigen/Dense>

namespace qcar_visnav { namespace slam {

// EKF landmark in world frame
struct Landmark {
  Eigen::Vector2d mu{0,0};
  Eigen::Matrix2d Sigma = Eigen::Matrix2d::Identity();
  int   hits{0};
  int   misses{0};
  bool  confirmed{false};
};

}} // namespace
