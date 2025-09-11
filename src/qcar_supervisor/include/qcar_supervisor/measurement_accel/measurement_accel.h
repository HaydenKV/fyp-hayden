#pragma once
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

class MeasurementAccelerometer {
public:
  MeasurementAccelerometer();

  Eigen::Vector2d predict(const KinematicModel::StateVec& x,
                          const KinematicModel::ModelInput& input,
                          const KinematicModel::StateVec& dxdt,
                          const KinematicModel::StateMat& ddxdtdx,
                          double gravity);

  Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE> jacobian(
                          const KinematicModel::StateVec& x,
                          const KinematicModel::StateMat& ddxdtdx,
                          double gravity);

  Eigen::Matrix2d noiseCovariance(int z_acc_status) const;

private:
  Eigen::Vector3d rMBb_;  // lever arm
  Eigen::Matrix3d Rbm_;   // sensor-to-body rotation
};

} // namespace qcar_nav
