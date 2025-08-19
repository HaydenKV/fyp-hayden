#include "qcar_supervisor/JMF/JMF.h"

namespace qcar_supervisor {

JumpMarkovFilter::JumpMarkovFilter()
: mode_prob_(ModeProb::Zero()),
  transition_mat_(TransMat::Identity())
{}

void JumpMarkovFilter::setModelParams(const ModelParams& params) {
  params_ = params;
}

void JumpMarkovFilter::setInput(const ModelInput& input) {
  input_ = input;
}

void JumpMarkovFilter::init(const StateVec& x0, 
                            const StateMat& P0,
                            const ModeProb& init_mode_prob,
                            const TransMat& trans_mat)
{
  mode_prob_      = init_mode_prob;
  transition_mat_ = trans_mat;
  for(int i = 0; i < M; ++i) {
    x_upd_[i] = x0;
    P_upd_[i] = P0;
  }
}

void JumpMarkovFilter::predict(double t, double dt) {
  // … mixing weights …
  for(int i = 0; i < M; ++i) {
    model_.setParams(params_);
    model_.setInput(input_);

    x_pred_[i] = model_.predict(x_upd_[i], t, dt);
    StateMat F = model_.getProcessJacobian(x_upd_[i], dt);
    StateMat Q = model_.getProcessNoise(dt);
    P_pred_[i] = F * P_upd_[i] * F.transpose() + Q;
  }
}

void JumpMarkovFilter::update(const Eigen::VectorXd& z,
                              const HfunType&        h_fun,
                              const HmatType&        H_fun,
                              const Eigen::MatrixXd& R)
{
  Eigen::Matrix<double, M, 1> likelihoods;
  for(int i = 0; i < M; ++i) {
    ekf_.init(x_pred_[i], P_pred_[i]);
    ekf_.update(z, h_fun, H_fun, R);
    x_upd_[i] = ekf_.state();
    P_upd_[i] = ekf_.covar();
    // compute likelihoods(i) …
  }
  // update mode_prob_ …
}

JumpMarkovFilter::StateVec JumpMarkovFilter::fusedState() const {
  StateVec xf = StateVec::Zero();
  for(int i=0; i<M; ++i) xf += mode_prob_(i) * x_upd_[i];
  return xf;
}

JumpMarkovFilter::StateMat JumpMarkovFilter::fusedCovar() const {
  StateVec xf = fusedState();
  StateMat Pf = StateMat::Zero();
  for(int i=0; i<M; ++i) {
    StateVec dx = x_upd_[i] - xf;
    Pf += mode_prob_(i) * (P_upd_[i] + dx * dx.transpose());
  }
  return Pf;
}

} // namespace qcar_supervisor

