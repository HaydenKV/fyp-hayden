// measurement_accel.h
#pragma once
#include "qcar_visnav/estimation/model/kinematic_model.h"
#include <string>

namespace qcar_nav {

enum AccStatus { ACC_HEALTHY = 0, ACC_FAULTY = 1 };

class MeasurementAccelerometer {
public:
  MeasurementAccelerometer();

  Eigen::Vector2d predict(const KinematicModel::StateVec& x,
                          const KinematicModel::ModelInput& input,
                          const KinematicModel::StateVec& dxdt,
                          const KinematicModel::StateMat& ddxdtdx,
                          double gravity,
                          int z_acc_status) const;

  Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE>
  jacobian(const KinematicModel::StateVec& x,
           const KinematicModel::StateMat& ddxdtdx,
           double gravity) const;

  Eigen::Matrix2d noiseCovariance(int z_acc_status) const;

  // CHANGE: expose number of modes (helps checks elsewhere)
  inline int getStatusCount() const { return 2; }  // healthy, faulty

private:
  Eigen::Vector3d rMBb_;
  Eigen::Matrix3d Rbm_;

  // CHANGE: name these clearly as *standard deviations* (lab uses σ_acc=0.1 m/s^2)
  static constexpr double SIGMA_HEALTHY = 0.10; // m/s^2
  static constexpr double FAULT_SCALE   = 10.0; // R_faulty = 10^2 * R_healthy (tune)
};

inline std::string accStatusName(int s) {
  switch (s) { case ACC_HEALTHY: return "Healthy";
              case ACC_FAULTY:  return "Faulty";
              default:          return "Unknown"; }
}

} // namespace qcar_nav
