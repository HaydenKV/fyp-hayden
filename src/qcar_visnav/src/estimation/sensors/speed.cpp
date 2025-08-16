#include "qcar_visnav/estimation/sensors/speed.h"

namespace qcar_nav {

void SpeedMeas::predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                       const KinematicModel::ModelParams& /* params */,
                       ZVec& h, HVec& H) {
  // Speed measurement: h = vx (forward velocity)
  h(0) = state(0);  // vx
  
  // Jacobian: dh/dx = [1, 0, 0, 0, 0, 0] for [vx, vy, r, bg, bax, bay]
  H.setZero();
  H(0, 0) = 1.0;    // dh/dvx = 1
}

} // namespace qcar_nav