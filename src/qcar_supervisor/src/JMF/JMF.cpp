#include <ros/ros.h>       // bring in ROS_INFO_STREAM, ROS_INFO_STREAM, etc.
#include <ros/console.h>   // (optional) for more console control
#include "qcar_supervisor/JMF/JMF.h"
#include <cmath>

namespace qcar_supervisor {

JumpMarkovFilter::JumpMarkovFilter()
  : mode_prob_(ModeProb::Zero())
  , transition_mat_(TransMat::Identity())
{}

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
    // ROS_INFO_STREAM(
    //   "pred i="<<i
    //   <<"  x_pred="<<x_pred_[i].transpose()
    //   <<"  trace(P_pred)="<<P_pred_[i].trace()
    // );
  }
}

void JumpMarkovFilter::updateWithPosition(
  const Eigen::Vector2d& meas,
  const Eigen::Matrix2d& R_pos)
{
  // measurement = [vx; vy]
  auto h_fun = [&](auto const& x_pred){
    Eigen::Vector2d zh;
    zh << x_pred(0), x_pred(1);
    return zh;
  };

  // Jacobian picks state indices 0,1
  auto H_fun = [&](auto const&){
    Eigen::Matrix<double,2,KinematicModel::STATE_SIZE> Hm;
    Hm.setZero();
    Hm(0,0) = 1.0;
    Hm(1,1) = 1.0;
    return Hm;
  };

  update(meas, h_fun, H_fun, R_pos);
}

void JumpMarkovFilter::update(
  const Eigen::VectorXd&  z,
  const HfunType&         h_fun,
  const HmatType&         H_fun,
  const Eigen::MatrixXd&  R)
{
  ModeProb likelihoods = ModeProb::Zero();

  for(int i = 0; i < M; ++i) {
    ekf_.setInitial(x_pred_[i], P_pred_[i]);

    Eigen::VectorXd  h   = h_fun(x_pred_[i]);
    Eigen::MatrixXd  H   = H_fun(x_pred_[i]);
    ekf_.update(z, h, H, R);

    x_upd_[i] = ekf_.mu();
    P_upd_[i] = ekf_.getCovariance();

    Eigen::VectorXd diff   = z - h;
    Eigen::MatrixXd S      = H * P_pred_[i] * H.transpose() + R;
    double maha            = diff.transpose() * S.inverse() * diff;
    double detS            = S.determinant();
    double norm_const      = 1.0 / (std::pow(2.0 * M_PI, z.size()/2.0)
                                 * std::sqrt(detS));
    double raw_like = norm_const * std::exp(-0.5 * maha);
    // floor it to avoid underflows
    likelihoods(i) = std::max(raw_like, 1e-12);


    ROS_INFO_STREAM(
      "update i="<<i
      <<"  maha="<<maha
      <<"  detS="<<detS
      <<"  like="<<likelihoods(i)
    );

  }

  // ROS_INFO_STREAM("raw likelihoods = "<<likelihoods.transpose());

  // Update mode probabilities
  ModeProb prior = transition_mat_.transpose() * mode_prob_;
  mode_prob_     = prior.cwiseProduct(likelihoods);

  double s = mode_prob_.sum();
  if(s > 1e-8) mode_prob_ /= s;
  else         mode_prob_.setConstant(1.0 / double(M));
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
