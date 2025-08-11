#include "qcar_visnav/estimation/model/kinematic_model.h"
#include <cmath>

namespace qcar_nav {

KinematicModel::Vec KinematicModel::f(double, const Vec& x) const {
  const double X  = x(0), Y = x(1), psi = x(2);
  const double v  = x(3), r = x(4);
  const double bg = x(5), ba = x(6);
  (void)X; (void)Y; (void)bg;

  const double ax_lin = u_.a_meas_x - u_.g_x - ba;

  Vec fx; fx.setZero();
  fx(0) = v * std::cos(psi);   // Xdot
  fx(1) = v * std::sin(psi);   // Ydot
  fx(2) = r;                   // psidot
  fx(3) = ax_lin;              // vdot
  // r, bg, ba are random walks -> 0 here
  return fx;
}

void KinematicModel::jacobian(double, const Vec& x, Mat& F) const {
  F.setZero();
  const double psi = x(2), v = x(3);

  F(0,2) = -v * std::sin(psi);  // ∂Ẋ/∂ψ
  F(0,3) =  std::cos(psi);      // ∂Ẋ/∂v
  F(1,2) =  v * std::cos(psi);  // ∂Ẏ/∂ψ
  F(1,3) =  std::sin(psi);      // ∂Ẏ/∂v
  F(2,4) =  1.0;                // ∂ψ̇/∂r
  F(3,6) = -1.0;                // ∂v̇/∂b_a
}

void KinematicModel::processNoise(double dt,
                                  Eigen::Vector3d& sqrtQc,
                                  Eigen::Matrix<double,7,3>& L) const {
  const double sr  = std::sqrt(p_.q(4)); // r
  const double sbg = std::sqrt(p_.q(5)); // bg
  const double sba = std::sqrt(p_.q(6)); // ba
  sqrtQc << sr, sbg, sba;

  L.setZero();
  L(4,0) = 1.0; // r
  L(5,1) = 1.0; // bg
  L(6,2) = 1.0; // ba
}

} // namespace qcar_nav
