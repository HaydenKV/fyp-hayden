// include/qcar_supervisor/transition_model.h
#pragma once
#include <Eigen/Core>

namespace qcar_nav {

class TransitionModel {
public:
  void setReferenceMatrix(const Eigen::Matrix2d& T_ref, double dt_ref);
  Eigen::Matrix2d getTransition(double dt) const;

private:
  Eigen::Matrix2d T_ref_{Eigen::Matrix2d::Identity()};
  double dt_ref_{1.0};
};

} // namespace qcar_nav
