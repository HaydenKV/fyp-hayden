#include "qcar_visnav/estimation/integrators/rk4sde.h"
#include <cmath>
#include <algorithm>

namespace qcar_nav {

// ===== Helper: augmented dynamics ===========================================
// X = [ x | Jx | Jw ], where:
//  - x  ∈ R^7
//  - Jx ∈ R^{7x7}  = ∂x/∂x_k
//  - Jw ∈ R^{7x3}  = ∂x/∂w_k
// For additive noise on states (idxQ = r,bg,ba), the continuous-time relations are:
//   xdot  = f(x)
//   Jxdot = Fc * Jx
//   Jwdot = Fc * Jw + L
// where Fc = ∂f/∂x, and L maps the 3 noise channels into the state (7x3):
//   L = [0...; e_r; e_bg; e_ba]  (same as model.processNoise selector)
void RK4SDE::augmentedDynamics(const KinematicModel& model,
                               const Eigen::Matrix<double,7,15>& X,
                               Eigen::Matrix<double,7,15>& dXdt)
{
  // Unpack
  Vec x = X.block<7,1>(0,0);
  Mat Jx = X.block<7,7>(0,1);
  Eigen::Matrix<double,7,3> Jw = X.block<7,3>(0,8);

  // f(x) and Fc
  const Vec fx = model.f(0.0, x);
  KinematicModel::Mat Fc; Fc.setZero();
  model.jacobian(0.0, x, Fc);

  // L from model (7x3); sqrtQc not needed here
  Eigen::Vector3d sqrtQc; sqrtQc.setZero();
  Eigen::Matrix<double,7,3> L; L.setZero();
  model.processNoise(0.0, sqrtQc, L);

  // Derivatives
  Vec xdot = fx;
  Mat Jxdot = Fc * Jx;
  Eigen::Matrix<double,7,3> Jwdot = Fc * Jw + L;

  // Pack
  dXdt.block<7,1>(0,0)  = xdot;
  dXdt.block<7,7>(0,1)  = Jxdot;
  dXdt.block<7,3>(0,8)  = Jwdot;
}

// ===== RK4 core with sub-stepping and additive dw ============================
RK4SDE::Vec RK4SDE::step(const KinematicModel& model, double t,
                         const Vec& x0, double dt,
                         const std::array<int,3>& idxQ)
{
  (void)idxQ; // indices are encoded inside model.processNoise via L; not needed here

  // substeps to enforce dtMaxEst
  const double dtMax = model.params().dtMaxEst;
  const int nSub = std::max(1, static_cast<int>(std::ceil(dt / std::max(1e-9, dtMax))));
  const double h = dt / static_cast<double>(nSub);

  Vec x = x0;
  for (int j=0; j<nSub; ++j) {
    // standard RK4 on deterministic dynamics f(x)
    const Vec f1 = model.f(t, x);
    const Vec f2 = model.f(t + 0.5*h, x + 0.5*h * f1);
    const Vec f3 = model.f(t + 0.5*h, x + 0.5*h * f2);
    const Vec f4 = model.f(t + h,     x + h * f3);
    x += (f1 + 2.0*f2 + 2.0*f3 + f4) * (h/6.0);
    t += h;
  }
  return x;
}

void RK4SDE::stepWithJac(const KinematicModel& model, double t, const Vec& x0, double dt,
                         Vec& x_next, Mat& Jdx, Eigen::Matrix<double,7,3>& Jdw,
                         const std::array<int,3>& idxQ)
{
  // substeps
  const double dtMax = model.params().dtMaxEst;
  const int nSub = std::max(1, static_cast<int>(std::ceil(dt / std::max(1e-9, dtMax))));
  const double h = dt / static_cast<double>(nSub);

  // Build an "increment per substep" noise injection (conceptual dW)
  // Here we follow your MATLAB helper: each substep adds 'dW' (split across substeps).
  // We realize noise via the *mapping L* (preferred), but keep idxQ for consistency.
  Eigen::Matrix<double,7,3> L; Eigen::Vector3d sqrtQc;
  model.processNoise(0.0, sqrtQc, L);
  // We don't scale by sqrtQc here; Jdw maps unit dw. Cov is added outside as Jdw * Qw * Jdwᵀ.

  // Augmented state X = [ x | Jx | Jw ]  -> 7 x (1+7+3) = 7 x 15
  Eigen::Matrix<double,7,15> X;
  X.setZero();
  X.block<7,1>(0,0) = x0;           // x
  X.block<7,7>(0,1) = Mat::Identity(); // Jx = I
  // Jw starts at 0

  // A per-substep additive increment dW to emulate " + dW " in your helper:
  // In your MATLAB, dW was a 7x1 with nonzero at idxQ. Using L is equivalent and more general.
  const Eigen::Matrix<double,7,3> dW = L * (Eigen::Matrix<double,3,3>::Identity() / static_cast<double>(nSub));

  for (int j=0; j<nSub; ++j) {
    // Classic RK4 but with the additive dW in each stage:
    Eigen::Matrix<double,7,15> F1, F2, F3, F4;

    augmentedDynamics(model, X, F1);
    augmentedDynamics(model, X + 0.5*h*F1 + 0.5*dW.replicate<1,5>(), F2);
    augmentedDynamics(model, X + 0.5*h*F2 + 0.5*dW.replicate<1,5>(), F3);
    augmentedDynamics(model, X + h*F3 + dW.replicate<1,5>(),        F4);

    X += (F1 + 2.0*F2 + 2.0*F3 + F4) * (h/6.0) + dW.replicate<1,5>();
    t += h;
  }

  // Extract
  x_next = X.block<7,1>(0,0);
  Jdx    = X.block<7,7>(0,1);
  Jdw    = X.block<7,3>(0,8);

  // NOTE: Jdw maps unit-variance dw. To build Qk, do: Qk = Jdw * Qw * Jdwᵀ,
  // where Qw = diag([σ_r², σ_bg², σ_ba²]) * dt  (or tuned discrete values).
}

} // namespace qcar_nav
