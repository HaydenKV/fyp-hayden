#include <ros/ros.h>       // bring in ROS_INFO_STREAM, ROS_INFO_STREAM, etc.
#include <ros/console.h>   // (optional) for more console control
#include "qcar_supervisor/JMF.h"
#include <cmath>
#include <algorithm> 

namespace qcar_supervisor {
  constexpr int JumpMarkovFilter::M;
}

namespace qcar_supervisor {

JumpMarkovFilter::JumpMarkovFilter()
  : mode_prob_(ModeProb::Zero())
  , transition_mat_(TransMat::Identity())
{}

// Project symmetric matrix A to SPD by clamping eigenvalues to eps
template<typename Derived>
static inline void makeSPD(Eigen::MatrixBase<Derived>& A, double eps = 1e-6) {
    // force symmetry first
    A = 0.5 * (A + A.transpose());
    Eigen::SelfAdjointEigenSolver<typename Derived::PlainObject> es(A);
    Eigen::VectorXd vals = es.eigenvalues();
    typename Derived::PlainObject vecs = es.eigenvectors();
    for (int k = 0; k < vals.size(); ++k) {
    if (std::isnan(vals[k]) || vals[k] < eps) vals[k] = eps;
    }
    A = vecs * vals.asDiagonal() * vecs.transpose();
    // re-symmetrize to clean roundoff
    A = 0.5 * (A + A.transpose());
}

void JumpMarkovFilter::setModelParams(
  const std::array<ModelParams, M>& mode_params)
{
  mode_params_ = mode_params;
}

void JumpMarkovFilter::setInput(const ModelInput& input)
{
  input_ = input;
}

void JumpMarkovFilter::init(
  const StateVec& x0,
  const StateMat& P0,
  const ModeProb& init_mode_prob,
  const TransMat& transition_mat)
{
  mode_prob_      = init_mode_prob;
  transition_mat_ = transition_mat;
  ROS_INFO_STREAM("[JMF] transition_mat_ =\n" << transition_mat_);
  for(int i = 0; i < M; ++i) {
    x_upd_[i] = x0;
    P_upd_[i] = P0;
  }
}

void JumpMarkovFilter::predict(double t, double dt)
{
  // 1) IMM mixing
  std::array<StateVec, M> x_mix;
  std::array<StateMat, M> P_mix;

  for(int j = 0; j < M; ++j) {
    x_mix[j].setZero();
    P_mix[j].setZero();
    double norm = 0.0;

    for(int i = 0; i < M; ++i) {
      double w = transition_mat_(i,j) * mode_prob_(i);
      norm += w;
      x_mix[j] += w * x_upd_[i];
    }
    if(norm > 1e-8) x_mix[j] /= norm;

    for(int i = 0; i < M; ++i) {
      double w = transition_mat_(i,j) * mode_prob_(i);
      StateVec d = x_upd_[i] - x_mix[j];
      P_mix[j] += w * (P_upd_[i] + d * d.transpose());
    }
    if(norm > 1e-8) P_mix[j] /= norm;
    // ROS_INFO_STREAM(
    //   "mix j="<<j
    //   <<"  norm="<<norm
    //   <<"\n  x_mix["<<j<<"]="<<x_mix[j].transpose()
    //   <<"\n  trace(P_mix["<<j<<"])="<<P_mix[j].trace()
    // );
  }

  // 2) EKF predict for each mode using its own params
  for(int i = 0; i < M; ++i) {
    model_.setParams(mode_params_[i]);

    model_.setInput(input_);

    x_pred_[i] = model_.predict(x_mix[i], t, dt);

    StateMat F = model_.getProcessJacobian(x_mix[i], dt);
    StateMat Q = model_.getProcessNoise(dt);
    P_pred_[i] = F * P_mix[i] * F.transpose() + Q;

    // Robustify P_pred: project to SPD (clamps bad/negative eigenvalues)
    makeSPD(P_pred_[i], 1e-6);

    // ROS_INFO_STREAM(
    //   "pred i="<<i
    //   <<"  x_pred="<<x_pred_[i].transpose()
    //   <<"  trace(P_pred)="<<P_pred_[i].trace()
    // );
  }
}

void JumpMarkovFilter::update(
  const Eigen::VectorXd&  z,
  const HfunType&         h_fun,
  const HmatType&         H_fun,
  const Eigen::MatrixXd&  R)
{
  // Gate and numerical‐stability constants
  static constexpr double CHI2_95   = 5.991;    // 95% χ² threshold for 2-D
  static constexpr double JITTER    = 1e-8;     // small SPD guard
  static constexpr double LOG_S_MIN = -50.0;    // lower clamp on log|S|
  static constexpr double LOG_S_MAX = +50.0;    // upper clamp on log|S|

  int n = int(z.size());

  // 1) Compute log-prior over modes
  Eigen::VectorXd prior     = transition_mat_.transpose() * mode_prob_;
  Eigen::VectorXd log_prior = prior
    .array()
    .max(1e-12)     // avoid log(0)
    .log();

  // Prepare a buffer for log[prior * likelihood]
  Eigen::VectorXd log_joint(M);

  // 2) Loop over each mode
  for(int i = 0; i < M; ++i) {
    // 2a) Innovation model at the predicted state
    Eigen::VectorXd  h_pred = h_fun(x_pred_[i]);
    Eigen::MatrixXd  H      = H_fun(x_pred_[i]);
    Eigen::VectorXd  diff   = z - h_pred;

    // 2b) Innovation covariance + jitter for SPD
    Eigen::MatrixXd S = H * P_pred_[i] * H.transpose() + R;

    // Force SPD: symmetrize + SPD projection
    makeSPD(S, 1e-5);

    auto ldltS = S.ldlt();
    if (ldltS.info() != Eigen::Success) {
      ROS_ERROR_STREAM("[JMF] S not SPD on mode="<<i<<" after makeSPD, hardening further");
      // In the rarest case, harden diagonals and project again
      S += 1e-4 * Eigen::MatrixXd::Identity(S.rows(), S.cols());
      makeSPD(S, 1e-6);
      ldltS.compute(S);
    }

    // 2c) Mahalanobis distance
    double maha = diff.transpose() * ldltS.solve(diff);

    // 2d) χ²‐gate out extreme outliers
    // if(maha > CHI2_95) {
    //   ROS_WARN_STREAM("[JMF] mode="<<i
    //                   <<" maha="<<maha
    //                   <<" > "<<CHI2_95<<", skipping update");
    //   x_upd_[i]    = x_pred_[i];
    //   P_upd_[i]    = P_pred_[i];
    //   log_joint(i) = log_prior(i);
    //   continue;
    // }

    // 2e) EKF measurement update
    ekf_.setInitial(x_pred_[i], P_pred_[i]);
    ekf_.update(z, h_pred, H, R);
    x_upd_[i] = ekf_.mu();
    P_upd_[i] = ekf_.getCovariance();

    // 2f) Compute log-det|S| via LDLT and clamp
    // log|S| from LDLT (robust)
    double log_detS = ldltS.vectorD().array().log().sum();
    log_detS = std::max(LOG_S_MIN,
                std::min(LOG_S_MAX, log_detS));

    // 2g) Gaussian log-likelihood: –½ [n·ln(2π) + ln|S| + maha]
    double log_norm = -0.5 * (n * std::log(2.0*M_PI) + log_detS);
    double log_like = log_norm - 0.5 * maha;

    log_joint(i) = log_prior(i) + log_like;

    ROS_DEBUG_STREAM("[JMF] mode="<<i
                    <<" maha="<<maha
                    <<" log_like="<<log_like
                    <<" log_joint="<<log_joint(i));
  }

  // 3) Normalise via log-sum-exp
  double m    = log_joint.maxCoeff();
  Eigen::VectorXd w = (log_joint.array() - m).exp();
  double wsum = w.sum();

  if(wsum > 0.0) {
    mode_prob_ = w / wsum;
  } else {
    ROS_ERROR("[JMF] all weights zero → uniform fallback");
    mode_prob_.setConstant(1.0 / double(M));
  }
}


JumpMarkovFilter::StateVec JumpMarkovFilter::fusedState() const
{
  StateVec xf = StateVec::Zero();
  for(int i = 0; i < M; ++i) {
    xf += mode_prob_(i) * x_upd_[i];
  }
  return xf;
}

JumpMarkovFilter::StateMat JumpMarkovFilter::fusedCovar() const
{
  StateVec xf = fusedState();
  StateMat Pf = StateMat::Zero();
  for(int i = 0; i < M; ++i) {
    StateVec d = x_upd_[i] - xf;
    Pf += mode_prob_(i) * (P_upd_[i] + d * d.transpose());
  }
  return Pf;
}

JumpMarkovFilter::ModeProb JumpMarkovFilter::modeProb() const
{
  return mode_prob_;
}

int JumpMarkovFilter::mostLikelyMode() const
{
  Eigen::Index idx;
  mode_prob_.maxCoeff(&idx);
  return static_cast<int>(idx);
}

} // namespace qcar_supervisor
