#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

KinematicModel::KinematicModel() {
  // Default parameters will be set via setParams()
}

void KinematicModel::setParams(const ModelParams& params) {
  params_ = params;
}

void KinematicModel::setInput(const ModelInput& input) {
  input_ = input;
}

KinematicModel::StateVec KinematicModel::predict(const StateVec& x_in, double t, double dt) const {
  // Use current input
  InputVec u;
  u << input_.a_meas_x, input_.a_meas_y, input_.delta;
  
  // Simple Euler integration (can upgrade to RK4 if needed)
  StateVec x_dot = stateDot(x_in, u);
  StateVec x_pred = x_in + dt * x_dot;

  // // CHANGE 1: Comment out const double and x_pred(2) to get back to old sol
  // Algebraic yaw-rate (overwrite, not integrate)
  const double r_alg = (x_pred(0) / params_.L) * std::tan(input_.delta);
  x_pred(2) = r_alg;

  return x_pred;
}

KinematicModel::StateVec KinematicModel::stateDot(const StateVec& x, const InputVec& u) const {
  // Extract states
  double vx = x(0);
  double vy = x(1);
  double r = x(2);
  double bg = x(3);
  double bax = x(4);
  double bay = x(5);
  
  // Extract inputs
  double a_meas_x = u(0);
  double a_meas_y = u(1);
  double delta = u(2);
  
  // State derivatives
  StateVec x_dot;
  
  // Velocity dynamics with bias-corrected accelerations
  x_dot(0) = (a_meas_x - bax) + r * vy;  // vx_dot
  x_dot(1) = (a_meas_y - bay) - r * vx;  // vy_dot
  
  // Yaw rate from bicycle model
  // // CHANGE 2: x_dot(2) = (vx / params_.L) * std::tan(delta);  // r_dot
  x_dot(2) = 0.0;
  
  // Bias random walks (0 drift; noise added via Q)
  x_dot(3) = 0.0;  // bg_dot (driven by process noise)
  x_dot(4) = 0.0;  // bax_dot (driven by process noise)
  x_dot(5) = 0.0;  // bay_dot (driven by process noise)
  
  return x_dot;
}

KinematicModel::StateMat KinematicModel::getProcessJacobian(const StateVec& x, double dt) const {
  // Extract states
  double vx = x(0);
  double vy = x(1);
  double r = x(2);
  
  // Extract inputs
  double delta = input_.delta;
  
  // Compute Jacobian F = I + dt * df/dx
  StateMat F = StateMat::Identity();
  
  // Row 0: d(vx_dot)/d[vx, vy, r, bg, bax, bay]
  F(0, 0) += 0.0;           // d(vx_dot)/dvx = 0
  F(0, 1) += dt * r;        // d(vx_dot)/dvy = r
  F(0, 2) += dt * vy;       // d(vx_dot)/dr = vy
  F(0, 3) += 0.0;           // d(vx_dot)/dbg = 0
  F(0, 4) += -dt;           // d(vx_dot)/dbax = -1
  F(0, 5) += 0.0;           // d(vx_dot)/dbay = 0
  
  // Row 1: d(vy_dot)/d[vx, vy, r, bg, bax, bay]
  F(1, 0) += -dt * r;       // d(vy_dot)/dvx = -r
  F(1, 1) += 0.0;           // d(vy_dot)/dvy = 0
  F(1, 2) += -dt * vx;      // d(vy_dot)/dr = -vx
  F(1, 3) += 0.0;           // d(vy_dot)/dbg = 0
  F(1, 4) += 0.0;           // d(vy_dot)/dbax = 0
  F(1, 5) += -dt;           // d(vy_dot)/dbay = -1
  
  // Row 2: d(r_dot)/d[vx, vy, r, bg, bax, bay]
  F(2, 0) += 0.0; // CHANGE 3: dt * std::tan(delta) / params_.L;  // d(r_dot)/dvx = tan(delta)/L
  F(2, 1) += 0.0;           // d(r_dot)/dvy = 0
  F(2, 2) += 0.0;           // d(r_dot)/dr = 0
  F(2, 3) += 0.0;           // d(r_dot)/dbg = 0
  F(2, 4) += 0.0;           // d(r_dot)/dbax = 0
  F(2, 5) += 0.0;           // d(r_dot)/dbay = 0
  
  // Rows 3-5: Biases are random walks (F is already identity for these)
  
  return F;
}

KinematicModel::InputMat KinematicModel::getInputJacobian(const StateVec& x, double dt) const {
  // Extract states
  double vx = x(0);
  
  // Extract inputs
  double delta = input_.delta;
  
  InputMat G = InputMat::Zero();
  
  // Row 0: d(vx_dot)/d[a_meas_x, a_meas_y, delta]
  G(0, 0) = dt;             // d(vx_dot)/da_meas_x = 1
  G(0, 1) = 0.0;            // d(vx_dot)/da_meas_y = 0
  G(0, 2) = 0.0;            // d(vx_dot)/ddelta = 0
  
  // Row 1: d(vy_dot)/d[a_meas_x, a_meas_y, delta]
  G(1, 0) = 0.0;            // d(vy_dot)/da_meas_x = 0
  G(1, 1) = dt;             // d(vy_dot)/da_meas_y = 1
  G(1, 2) = 0.0;            // d(vy_dot)/ddelta = 0
  
  // Row 2: d(r_dot)/d[a_meas_x, a_meas_y, delta]
  G(2, 0) = 0.0;            // d(r_dot)/da_meas_x = 0
  G(2, 1) = 0.0;            // d(r_dot)/da_meas_y = 0
  double sec_delta = 1.0 / std::cos(delta);
  G(2, 2) = dt * (vx / params_.L) * sec_delta * sec_delta;  // d(r_dot)/ddelta = vx/L * sec²(delta)
  
  // Rows 3-5: Biases don't depend on inputs
  
  return G;
}

KinematicModel::StateMat KinematicModel::getProcessNoise(double dt) const {
  StateMat Q = StateMat::Zero();
  
  // Interpret params_.q as continuous-time random-walk *intensities*
  // Discretization for one step: Qk = diag(q) * dt
  Q(0, 0) = params_.q(0) * dt;  // vx process noise
  Q(1, 1) = params_.q(1) * dt;  // vy process noise
  Q(2, 2) = params_.q(2) * dt;  // r  process noise
  Q(3, 3) = params_.q(3) * dt;  // bg random walk
  Q(4, 4) = params_.q(4) * dt;  // bax random walk
  Q(5, 5) = params_.q(5) * dt;  // bay random walk
  
  return Q;
}

Eigen::Matrix<double, KinematicModel::INPUT_SIZE, KinematicModel::INPUT_SIZE> 
KinematicModel::getInputNoise(double dt) const {
  // Input noise covariance (accelerometer and steering noise)
  Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> Q_u;
  Q_u.setZero();
  
  // Accelerometer noise (post gravity removal)
  Q_u(0, 0) = std::pow(0.2, 2);  // ax noise variance (m/s^2)^2
  Q_u(1, 1) = std::pow(0.2, 2);  // ay noise variance (m/s^2)^2
  
  // Steering angle noise (small)
  Q_u(2, 2) = std::pow(0.01, 2); // delta noise variance (rad)^2
  
  return Q_u;
}

// Legacy interface methods for RK4SDE compatibility
KinematicModel::StateVec KinematicModel::f(double t, const StateVec& x) const {
  // Convert to input vector format
  InputVec u;
  u << input_.a_meas_x, input_.a_meas_y, input_.delta;
  return stateDot(x, u);
}

void KinematicModel::jacobian(double t, const StateVec& x, StateMat& Jac) const {
  // For legacy interface, use current dt from input
  Jac = getProcessJacobian(x, input_.dt);
}

void KinematicModel::processNoise(double dt, Eigen::Vector3d& sqrtQc, 
                                 Eigen::Matrix<double, STATE_SIZE, 3>& L) const {
  // Legacy interface: return subset of process noise for RK4SDE
  // RK4SDE expects 3D noise for bias states [bg, bax, bay]
  sqrtQc(0) = std::sqrt(params_.q(3) * dt);  // bg
  sqrtQc(1) = std::sqrt(params_.q(4) * dt);  // bax
  sqrtQc(2) = std::sqrt(params_.q(5) * dt);  // bay
  
  // Distribution matrix L maps 3D noise to full state space
  L.setZero();
  L(3, 0) = 1.0;  // bg noise affects bg state
  L(4, 1) = 1.0;  // bax noise affects bax state
  L(5, 2) = 1.0;  // bay noise affects bay state
}

} // namespace qcar_nav