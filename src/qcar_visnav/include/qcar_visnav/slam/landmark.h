#pragma once
#include <Eigen/Core>

namespace qcar_visnav { namespace slam {

struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d mu{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d Sigma{Eigen::Matrix2d::Identity()};
  int  hits{0};
  bool confirmed{false};
};

}} // namespace
