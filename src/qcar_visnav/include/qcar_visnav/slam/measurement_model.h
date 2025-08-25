#pragma once
#include <Eigen/Core>

namespace qcar_visnav { namespace slam {

class MeasurementModel {
public:
  struct Prediction {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2d zhat;
    Eigen::Matrix2d H;
  };

  MeasurementModel() = default;

  Prediction predict(double /*px*/, double /*py*/, double /*pyaw*/,
                     const Eigen::Vector2d& lm) const
  {
    Prediction p;
    p.zhat = lm;
    p.H.setIdentity();
    return p;
  }
};

}} // namespace
