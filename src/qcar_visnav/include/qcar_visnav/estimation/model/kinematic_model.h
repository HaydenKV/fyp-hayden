#pragma once
#include <Eigen/Dense>

namespace qcar_nav {

// ---------- Model params / input ----------
struct ModelParams {
  double dtMaxEst{0.01}; // internal substep if using RK
  // continuous-time process noise variances for [X,Y,psi,v,r,bg,ba]
  Eigen::Matrix<double,7,1> q{
    (Eigen::Matrix<double,7,1>() << 1e-8,1e-8,1e-6,1e-4,5e-3,1e-6,1e-6).finished()
  };
};

struct ModelInput {
  double a_meas_x{0.0}; // IMU accel X (m/s^2)
  double g_x{0.0};      // gravity projection on body-X
  double dt{0.0};
};

// ---------- CTRV + accel-input model ----------
class KinematicModel {
public:
  using Vec = Eigen::Matrix<double,7,1>;
  using Mat = Eigen::Matrix<double,7,7>;

  void setParams(const ModelParams& p) { p_ = p; }
  void setInput (const ModelInput&  u) { u_ = u; }
  const ModelParams& params() const { return p_; }

  // continuous-time dynamics xdot = f(x)
  Vec f(double t, const Vec& x) const;

  // continuous Jacobian F = ∂f/∂x
  void jacobian(double t, const Vec& x, Mat& F) const;

  // process noise mapping: L (7x3) for [r, bg, ba] random walks, and sqrtQc (3)
  void processNoise(double dt,
                    Eigen::Vector3d& sqrtQc,
                    Eigen::Matrix<double,7,3>& L) const;

private:
  ModelParams p_{};
  ModelInput  u_{};
};

} // namespace qcar_nav
