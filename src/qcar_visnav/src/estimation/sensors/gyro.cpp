#include "qcar_visnav/estimation/sensors/gyro.h"

namespace qcar_nav {

void GyroMeas::predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                       const KinematicModel::ModelParams& /* params */,
                       ZVec& h, HVec& H) {
  // Gyro measurement: h = r + bg (yaw rate + gyro bias)
  double r = state(2);   // yaw rate
  double bg = state(3);  // gyro bias
  
  h(0) = r + bg;
  
  // Jacobian: dh/dx = [0, 0, 1, 1, 0, 0] for [vx, vy, r, bg, bax, bay]
  H.setZero();
  H(0, 2) = 1.0;    // dh/dr = 1
  H(0, 3) = 1.0;    // dh/dbg = 1
}

} // namespace qcar_nav