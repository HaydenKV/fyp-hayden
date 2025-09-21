#pragma once
#include <Eigen/Dense>
#include <utility>

namespace qcar_nav {

// Templated on state size and input size (input size reserved for future use).
template<int STATE_SIZE, int INPUT_SIZE>
class ExtendedKalmanFilter {
public:
  using StateVec = Eigen::Matrix<double, STATE_SIZE, 1>;
  using StateMat = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

  ExtendedKalmanFilter()
  : x_(StateVec::Zero()),
    P_(StateMat::Identity() * 1e3) {}  // Large initial uncertainty by default

  // Initialize or reset the EKF state and covariance.
  void setState(const StateVec& x, const StateMat& P) {
    x_ = x;
    P_ = sym(P);
  }

  // Optional: linearized predict step (only if/when you need it).
  // F: STATE_SIZE x STATE_SIZE, Q: STATE_SIZE x STATE_SIZE
  void predict(const StateMat& F, const StateMat& Q) {
    x_ = F * x_;
    P_ = F * P_ * F.transpose() + Q;
    P_ = sym(P_);
  }

  // Measurement update (M is compile-time measurement dimension).
  // z: Mx1
  // h(x): functor returning Eigen::Matrix<double, M, 1>
  // H(x): functor returning Eigen::Matrix<double, M, STATE_SIZE>
  // R: MxM
  template<int M, typename HFunctor, typename HJacFunctor>
  void update(const Eigen::Matrix<double, M, 1>& z,
              const HFunctor& h,
              const HJacFunctor& Hfun,
              const Eigen::Matrix<double, M, M>& R) {
    const auto y = z - h(x_);               // Innovation
    const auto H = Hfun(x_);                // Measurement Jacobian
    Eigen::Matrix<double, M, M> S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, STATE_SIZE, M> K = P_ * H.transpose() * S.inverse();

    x_ = x_ + K * y;

    // Joseph form for numerical stability
    StateMat I = StateMat::Identity();
    StateMat IKH = I - K * H;
    P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    P_ = sym(P_);
  }

  // Accessors
  const StateVec& x() const { return x_; }
  const StateMat& P() const { return P_; }
  std::pair<StateVec, StateMat> getState() const { return {x_, P_}; }

private:
  StateVec x_;
  StateMat P_;

  static StateMat sym(const StateMat& A) { return 0.5 * (A + A.transpose()); }
};

} // namespace qcar_nav
