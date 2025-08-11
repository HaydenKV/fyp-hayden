#pragma once
#include <Eigen/Dense>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

struct GyroMeas {
  using XVec = Eigen::Matrix<double,7,1>;
  using HVec = Eigen::Matrix<double,1,7>;
  using ZVec = Eigen::Matrix<double,1,1>;

  void predict(const XVec& x,
               const qcar_nav::ModelParams& p,
               ZVec& h,
               HVec& H) const;
};

} // namespace qcar_nav
