#include "qcar_supervisor/measurement_accel/measurement_accel.h"
#include <cmath>

namespace qcar_nav {

MeasurementAccelerometer::MeasurementAccelerometer() {
  rMBb_ << 1.0, 0.0, -0.5;
  Rbm_ = Eigen::Matrix3d::Identity();
}

Eigen::Vector2d MeasurementAccelerometer::predict(
    const KinematicModel::StateVec& x,
    const KinematicModel::ModelInput& input,
    const KinematicModel::StateVec& dxdt,
    const KinematicModel::StateMat& ddxdtdx,
    double gravity) {

  double psi = x(2);
  double vx = x(0), vy = x(1), r = x(2);
  double bg = x(3), bax = x(4), bay = x(5);

  Eigen::Vector3d gn(0, 0, gravity);
  Eigen::Matrix3d Rnb;
  Rnb << std::cos(psi), 0, std::sin(psi),
         0,             1, 0,
        -std::sin(psi), 0, std::cos(psi);

  Eigen::Vector3d omegaBNb(0, r, 0);
  Eigen::Matrix3d SomegaBNb = KinematicModel::skew(omegaBNb);
  Eigen::Matrix3d SrMBb = KinematicModel::skew(rMBb_);

  Eigen::Vector3d v(vx, 0, vy);
  Eigen::Vector3d vdot(dxdt(0), 0, dxdt(1));
  Eigen::Vector3d omegaBNbdot(0, dxdt(2), 0);

  Eigen::Vector3d aM2Nb = vdot
                        + SomegaBNb * v
                        - SrMBb * omegaBNbdot
                        - SomegaBNb * SrMBb * omegaBNb;

  Eigen::Vector3d acc_body = Rbm_.transpose() * (aM2Nb - Rnb.transpose() * gn);
  Eigen::Vector2d h;
  h << acc_body(0), acc_body(2); // x and z axes

  return h;
}

Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE>
MeasurementAccelerometer::jacobian(const KinematicModel::StateVec& x,
                                   const KinematicModel::StateMat& ddxdtdx,
                                   double gravity) {
  Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE> dhdx;
  dhdx.setZero();

  double vx = x(0), vy = x(1), r = x(2);
  double psi = x(2);

  Eigen::Matrix<double, 3, KinematicModel::STATE_SIZE> daM2Nbdx;
  daM2Nbdx.setZero();

  daM2Nbdx(0, 1) = r;
  daM2Nbdx(0, 2) = vy;
  daM2Nbdx(2, 0) = -r;
  daM2Nbdx(2, 2) = -vx;

  daM2Nbdx.row(0) += ddxdtdx.row(0) - 0.5 * ddxdtdx.row(2);
  daM2Nbdx.row(2) += ddxdtdx.row(1) - ddxdtdx.row(2);

  Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE> dgdx;
  dgdx.setZero();
  dgdx(0, 2) = gravity * std::cos(psi);
  dgdx(1, 2) = gravity * std::sin(psi);

  dhdx.row(0) = daM2Nbdx.row(0) + dgdx.row(0);
  dhdx.row(1) = daM2Nbdx.row(2) + dgdx.row(1);

  return dhdx;
}

Eigen::Matrix2d MeasurementAccelerometer::noiseCovariance(int z_acc_status) const {
  if (z_acc_status == 1) {
    return Eigen::Matrix2d::Identity() * 0.01; // (0.1 m/s^2)^2
  } else {
    throw std::runtime_error("Unknown accelerometer status");
  }
}

} // namespace qcar_nav
