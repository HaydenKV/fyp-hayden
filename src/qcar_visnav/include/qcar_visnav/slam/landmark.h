#pragma once
#include <Eigen/Core>

namespace qcar_visnav { namespace slam {

/**
 * Landmark state for Cartesian (x,y) map coordinates.
 * Lifecycle fields support pruning and confirmation.
 */
struct Landmark {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d mu{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d Sigma{Eigen::Matrix2d::Identity()};
  int  hits{0};
  bool confirmed{false};

  // --- Phase 2 lifecycle bookkeeping ---
  // Counts since last successful update (used for pruning)
  int misses{0};
  // Total number of updates received (matched/merged OR created)
  int age{0};
};

}} // namespace
