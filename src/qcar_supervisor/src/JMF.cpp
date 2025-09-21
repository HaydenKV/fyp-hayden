// src/JMF.cpp
#include "qcar_supervisor/JMF.h"
#include "qcar_supervisor/ekf/extended_kalman_filter.h"
#include "qcar_supervisor/measurement_accel/measurement_accel.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Cholesky>
#include <ros/ros.h>  // for ROS_INFO_STREAM

namespace qcar_nav {
  // Helper: log Gaussian density for 2D (innovation y, covariance S)
inline double logGaussian(const Eigen::Vector2d& y, const Eigen::Matrix2d& S) {
  // Cholesky (LLT) for 2x2 positive-definite S
  Eigen::LLT<Eigen::Matrix2d> llt(S);
  if (llt.info() != Eigen::Success) {
    // tiny jitter if needed
    Eigen::Matrix2d S_eps = S + 1e-9 * Eigen::Matrix2d::Identity();
    llt.compute(S_eps);
  }
  Eigen::Matrix2d L = llt.matrixL();

  // quad form: yᵀ S⁻¹ y  via solves
  Eigen::Vector2d v = llt.solve(y);
  double quad = v.squaredNorm();

  // log|S| from Cholesky: logdet = 2 * (log L(0,0) + log L(1,1))
  double logdetS = 2.0 * (std::log(L(0,0)) + std::log(L(1,1)));

  constexpr double LOG_2PI = 1.8378770664093453; // log(2π)
  return -0.5 * (quad + logdetS + 2.0 * LOG_2PI);
}

JumpMarkovFilter::JumpMarkovFilter(int max_components)
  : max_components_(max_components) {}

void JumpMarkovFilter::initialize(const std::vector<JmfComponent>& initial) {
  if (initial.empty()) {
    throw std::runtime_error("JMF initialization requires at least one component");
  }
  components_ = initial;
}

void JumpMarkovFilter::step(double dt,
                            const KinematicModel::ModelInput& input,
                            const KinematicModel::StateVec& dxdt,
                            const KinematicModel::StateMat& ddxdtdx,
                            double gravity,
                            const TransitionModel& T_acc) {
  expandComponents(dt, input, dxdt, ddxdtdx, gravity, T_acc);
  normalizeWeights();
  reduceComponents();
}

void JumpMarkovFilter::expandComponents(double dt,
                                        const KinematicModel::ModelInput& input,
                                        const KinematicModel::StateVec& dxdt,
                                        const KinematicModel::StateMat& ddxdtdx,
                                        double gravity,
                                        const TransitionModel& T_acc)
{
  std::vector<JmfComponent> new_components;
  Eigen::Matrix2d T_acc_dt = T_acc.getTransition(dt);  // T(i,j) = Pr(z_k=i | z_{k-1}=j)

  for (const auto& comp : components_) {
    // For each previous component j, expand over all current accel modes i ∈ {1,2}
    for (int acc_status = ACC_HEALTHY; acc_status <= ACC_FAULTY; ++acc_status) {
      // CHANGE: correct indexing — use T(i,j), not T(j,i)
      const int i = acc_status;           // 0 or 1
      const int j = comp.status.acc;      // 0 or 1
      double p_acc = T_acc_dt(i, j);
      if (p_acc <= 0.0) p_acc = 1e-15;                 // numerical hygiene
      double log_trans = std::log(p_acc);

      // Set up EKF at the prior (comp.mean, comp.cov)
      JmfComponent new_comp = comp;
      new_comp.status.acc = acc_status;

      ExtendedKalmanFilter<KinematicModel::STATE_SIZE, KinematicModel::INPUT_SIZE> ekf;
      ekf.setState(comp.mean, comp.cov);

      MeasurementAccelerometer meas_acc;

      // Build measurement model functors (as you had)
      struct AccH {
        const MeasurementAccelerometer* meas;
        const KinematicModel::ModelInput* input;
        const KinematicModel::StateVec* dxdt;
        const KinematicModel::StateMat* ddxdtdx;
        double g;
        int acc_status;
        Eigen::Matrix<double,2,1> operator()(const KinematicModel::StateVec& x) const {
          return meas->predict(x, *input, *dxdt, *ddxdtdx, g, acc_status);
        }
      };
      struct AccHjac {
        const MeasurementAccelerometer* meas;
        const KinematicModel::StateMat* ddxdtdx;
        double g;
        Eigen::Matrix<double,2,KinematicModel::STATE_SIZE>
        operator()(const KinematicModel::StateVec& x) const {
          return meas->jacobian(x, *ddxdtdx, g);
        }
      };

      // Measured accel (z), mode-conditioned noise
      Eigen::Matrix<double,2,1> z_acc;
      z_acc << input.a_meas_x, input.a_meas_y;
      Eigen::Matrix2d R_acc = meas_acc.noiseCovariance(acc_status);

      // ---------- MEASUREMENT EVIDENCE (log β) ----------
      // CHANGE: compute innovation y and S at the PRIOR (comp) to form log β
      Eigen::Matrix<double,2,1> zhat = AccH{&meas_acc,&input,&dxdt,&ddxdtdx,gravity,acc_status}(comp.mean);
      Eigen::Matrix<double,2,KinematicModel::STATE_SIZE> H =
          AccHjac{&meas_acc,&ddxdtdx,gravity}(comp.mean);
      Eigen::Matrix<double,2,1> y = z_acc - zhat;
      Eigen::Matrix2d S = H * comp.cov * H.transpose() + R_acc;

      // Guard ill-conditioning
      Eigen::LLT<Eigen::Matrix2d> lltS(S);
      if (lltS.info() != Eigen::Success) {
        // Fallback: add a small jitter
        S += 1e-9 * Eigen::Matrix2d::Identity();
      }

      // log β = log N(y;0,S)
      double log_beta = logGaussian(y, S);            // helper below
      if (!std::isfinite(log_beta)) {
        // fallback: treat as extremely unlikely but finite
        log_beta = -1e12;
      }
      // --- DIAG: sanity print once every ~1s ---
      static double last_dbg = 0.0;
      double now = ros::Time::now().toSec();   // include <ros/ros.h> at top of file if not present
      if (now - last_dbg > 1.0) {
        ROS_INFO_STREAM("[ACC "
            << (acc_status == ACC_HEALTHY ? "HEALTHY" : "FAULTY")
            << "] log_beta=" << log_beta
            << "  trace(R)=" << R_acc.trace()
            << "  T(i|j)=" << p_acc
            << "  enum(H,F)=" << ACC_HEALTHY << "," << ACC_FAULTY);
        last_dbg = now;
      }

      // ---------- EKF UPDATE (posterior mean/cov for this branch) ----------
      ekf.update(z_acc,
                 AccH{&meas_acc,&input,&dxdt,&ddxdtdx,gravity,acc_status},
                 AccHjac{&meas_acc,&ddxdtdx,gravity},
                 R_acc);
      auto [x_upd, P_upd] = ekf.getState();
      new_comp.mean = x_upd;
      new_comp.cov  = P_upd;

      // ---------- COMBINE PRIOR + EVIDENCE ----------
      // CHANGE: add BOTH terms (log T + log β) to the new component weight
      new_comp.log_weight = comp.log_weight + log_trans + log_beta;

      new_components.push_back(new_comp);
    }
  }

  components_ = std::move(new_components);
}

void JumpMarkovFilter::normalizeWeights() {
  // 1) get max log-weight
  double max_log = -std::numeric_limits<double>::infinity();
  for (const auto& c : components_) {
    if (std::isfinite(c.log_weight) && c.log_weight > max_log) max_log = c.log_weight;
  }
  if (!std::isfinite(max_log)) {
    // all bad — reset to uniform tiny but finite
    const double uniform = -std::log(static_cast<double>(components_.size()));
    for (auto& c : components_) c.log_weight = uniform;
    return;
  }

  // 2) compute logsumexp = max_log + log(sum_i exp(logw_i - max_log))
  double sum_exp = 0.0;
  for (const auto& c : components_) {
    if (std::isfinite(c.log_weight)) sum_exp += std::exp(c.log_weight - max_log);
  }
  if (!(sum_exp > 0.0)) { // sum_exp is 0 or NaN
    const double uniform = -std::log(static_cast<double>(components_.size()));
    for (auto& c : components_) c.log_weight = uniform;
    return;
  }
  const double lse = max_log + std::log(sum_exp);

  // 3) normalised log-weights: log p_i = logw_i - logsumexp
  for (auto& c : components_) {
    if (std::isfinite(c.log_weight)) c.log_weight -= lse;
    else c.log_weight = -std::log(static_cast<double>(components_.size()));
  }
}


void JumpMarkovFilter::reduceComponents() {
  std::sort(components_.begin(), components_.end(),
            [](const JmfComponent& a, const JmfComponent& b) {
              return a.log_weight > b.log_weight;
            });

  if (components_.size() > max_components_) {
    components_.resize(max_components_);
  }
}

KinematicModel::StateVec JumpMarkovFilter::getMAPEstimate() const {
  auto it = std::max_element(components_.begin(), components_.end(),
                             [](const JmfComponent& a, const JmfComponent& b) {
                               return a.log_weight < b.log_weight;
                             });
  return it->mean;
}

std::vector<double> JumpMarkovFilter::getAccStatusMarginals() const {
  double pH = 0.0, pF = 0.0;
  for (const auto& c : components_) {
    const double w = std::exp(c.log_weight);
    if (c.status.acc == ACC_HEALTHY) pH += w;
    else if (c.status.acc == ACC_FAULTY) pF += w;
  }
  return {pH, pF};  // index 0: healthy, index 1: faulty
}

} // namespace qcar_nav
