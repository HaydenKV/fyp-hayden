// measurement_accel.cpp
#include "qcar_supervisor/measurement_accel/measurement_accel.h"
#include "qcar_supervisor/helpers/skew.h"
#include <cmath>

namespace qcar_nav {

MeasurementAccelerometer::MeasurementAccelerometer() {
  rMBb_ << 0.1278, 0.0223, 0.0895;
  Rbm_  << 0,0,1,
           1,0,0,
           0,1,0; // IMU->Body
}

Eigen::Vector2d MeasurementAccelerometer::predict(
    const KinematicModel::StateVec& x,
    const KinematicModel::ModelInput& input,
    const KinematicModel::StateVec& dxdt,
    const KinematicModel::StateMat& ddxdtdx,
    double gravity,
    int z_acc_status) const
{
  // indices (adjust to your KinematicModel if different):
  const double vx  = x(0);
  const double vy  = x(1);
  const double r   = x(2);   // yaw rate
  const double psi = x(3);   // yaw
  const double bax = x(4);   // <-- accel bias x
  const double bay = x(5);   // <-- accel bias y

  Eigen::Vector3d gn(0, 0, gravity);
  Eigen::Matrix3d Rnb;
  Rnb <<  std::cos(psi), 0, std::sin(psi),
           0,            1, 0,
         -std::sin(psi), 0, std::cos(psi);

  const Eigen::Vector3d omegaBNb(0, r, 0);
  const Eigen::Matrix3d SomegaBNb = helpers::skew(omegaBNb);
  const Eigen::Matrix3d SrMBb     = helpers::skew(rMBb_);

  const Eigen::Vector3d v(vx, 0, vy);
  const Eigen::Vector3d vdot(dxdt(0), 0, dxdt(1));
  const Eigen::Vector3d omegaBNbdot(0, dxdt(2), 0);

  const Eigen::Vector3d aM2Nb =
      vdot
    + SomegaBNb * v
    - SrMBb * omegaBNbdot
    - SomegaBNb * SrMBb * omegaBNb;

  const Eigen::Vector3d acc_body = Rbm_.transpose() * (aM2Nb - Rnb.transpose() * gn);

  Eigen::Vector2d h;
  // CHANGE: output (x, y) **including biases**
  h << acc_body(0) + bax,
       acc_body(1) + bay;

  (void)z_acc_status; // mean not changed per mode; fault handled in noiseCovariance()
  return h;
}

Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE>
MeasurementAccelerometer::jacobian(const KinematicModel::StateVec& x,
                                   const KinematicModel::StateMat& ddxdtdx,
                                   double gravity) const
{
  Eigen::Matrix<double, 2, KinematicModel::STATE_SIZE> dhdx;
  dhdx.setZero();

  // indices (adjust if yours differ)
  const double vx  = x(0);
  const double vy  = x(1);
  const double r   = x(2);
  const double psi = x(3);
  // bax = x(4), bay = x(5)

  Eigen::Matrix<double,3,KinematicModel::STATE_SIZE> daM2Nbdx;
  daM2Nbdx.setZero();

  // same structure you had (ensure it matches your model)
  daM2Nbdx(0,1) = r;
  daM2Nbdx(0,2) = vy;
  daM2Nbdx(2,0) = -r;
  daM2Nbdx(2,2) = -vx;

  // include ∂ẋ/∂x terms
  daM2Nbdx.row(0) += ddxdtdx.row(0) - 0.5 * ddxdtdx.row(2);
  daM2Nbdx.row(2) += ddxdtdx.row(1) -        ddxdtdx.row(2);

  // Gravity contribution derivative for (x,y) output:
  // With your Rbm_ (IMU→body), the selected outputs imply:
  //   ax has **no** direct gravity term derivative wrt ψ,
  //   ay has  d/dψ(-g cos ψ) = +g sin ψ.
  Eigen::Matrix<double,2,KinematicModel::STATE_SIZE> dgdx;
  dgdx.setZero();
  // CHANGE: x-row gravity derivative is 0
  dgdx(0,3) = 0.0;
  // CHANGE: y-row gravity derivative is +g * sin(psi)
  dgdx(1,3) = gravity * std::sin(psi);

  // Map the 3D derivatives to your 2D output (x from 0-row, y from 2-row)
  dhdx.row(0) = daM2Nbdx.row(0) + dgdx.row(0); // ∂ax/∂x
  dhdx.row(1) = daM2Nbdx.row(2) + dgdx.row(1); // ∂ay/∂x

  // CHANGE: bias partials — ∂(ax)/∂bax = 1, ∂(ay)/∂bay = 1
  dhdx(0, 4) = 1.0;  // bax column index
  dhdx(1, 5) = 1.0;  // bay column index

  return dhdx;
}

Eigen::Matrix2d MeasurementAccelerometer::noiseCovariance(int z_acc_status) const {
  // CHANGE: “same mean, bigger R” fault model (much easier to detect)
  const double s  = SIGMA_HEALTHY;
  const double sf = FAULT_SCALE * SIGMA_HEALTHY;
  switch (z_acc_status) {
    case ACC_HEALTHY: {
      Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (s*s);
      return R;
    }
    case ACC_FAULTY: {
      Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (sf*sf);
      return R;
    }
    default: throw std::runtime_error("Unknown accelerometer status");
  }
}

} // namespace qcar_nav
