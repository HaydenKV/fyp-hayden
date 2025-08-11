#pragma once
#include <Eigen/Dense>
#include <array>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

// RK4 integrator for xdot = f(x) with additive process noise increments dw
// State: x ∈ R^7  ( [X,Y,psi,v,r,bg,ba] )
// Noise: nq = 3   (affects indices idxQ = {4,5,6} by default -> r,bg,ba)
class RK4SDE {
public:
  using Vec = Eigen::Matrix<double,7,1>;
  using Mat = Eigen::Matrix<double,7,7>;

  // Simple RK4 step (no Jacobians)
  static Vec step(const KinematicModel& model, double t, const Vec& x0, double dt,
                  const std::array<int,3>& idxQ = {4,5,6});

  // RK4 step with Jacobians:
  // x_next = f_rk4(t,x,dt),  Jdx = ∂x_next/∂x,  Jdw = ∂x_next/∂dw
  // dw is an abstract 3x1 noise increment placed at state indices idxQ.
  static void stepWithJac(const KinematicModel& model, double t, const Vec& x0, double dt,
                          Vec& x_next, Mat& Jdx, Eigen::Matrix<double,7,3>& Jdw,
                          const std::array<int,3>& idxQ = {4,5,6});

private:
  // Augmented dynamics used by RK4 on [ x, dxdx, dxdw ]
  // Given X = [ x | Jx | Jw ], returns d/dt of the same structure.
  static void augmentedDynamics(const KinematicModel& model,
                                const Eigen::Matrix<double,7,15>& X,
                                Eigen::Matrix<double,7,15>& dXdt);
};

} // namespace qcar_nav
