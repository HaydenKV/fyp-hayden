#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>
#include "qcar_supervisor/model/kinematic_model.h"
#include "qcar_supervisor/ekf.h"

namespace qcar_supervisor {

enum Mode {
  HEALTHY = 0,
  MOTOR_FAULT,
  STEERING_FAULT,
  IMU_FAULT,
  ENCODER_FAULT,
  NUM_MODES
};

class JumpMarkovFilter
{
public:
  static constexpr int M = Mode::NUM_MODES;

  //––– From KinematicModel
  using StateVec     = KinematicModel::StateVec;
  using StateMat     = KinematicModel::StateMat;
  using ModelParams  = KinematicModel::ModelParams;  // <— add this
  using ModelInput   = KinematicModel::ModelInput;   // <— and this

  //––– Mode probs & transition
  using ModeProb = Eigen::Matrix<double, M, 1>;
  using TransMat = Eigen::Matrix<double, M, M>;

  //––– Alias your EKF type and its funcs
  using EKFType   = ExtendedKalmanFilter;            // <— alias
  using HfunType  = EKFType::HfunType;                // measurement map h(x)
  using HmatType  = EKFType::HmatType;                // jacobian H(x)

  JumpMarkovFilter();

  // store params & input internally
  void setModelParams(const ModelParams& params);
  void setInput      (const ModelInput& input);

  // initialize filters & mode prior
  void init(const StateVec& x0,
            const StateMat& P0,
            const ModeProb& init_mode_prob,
            const TransMat& trans_mat);

  // JMF predict + update
  void predict(double t, double dt);
  void update(const Eigen::VectorXd& z,
              const HfunType&        h_fun,
              const HmatType&        H_fun,
              const Eigen::MatrixXd& R);

  // fused outputs
  StateVec fusedState() const;
  StateMat fusedCovar() const;
  ModeProb modeProb()     const { return mode_prob_; }
  int      mostLikelyMode() const { ModeProb::Index i; mode_prob_.maxCoeff(&i); return i; }

private:
  // internal storage for each mode
  std::array<StateVec, M> x_pred_, x_upd_;
  std::array<StateMat, M> P_pred_, P_upd_;

  ModeProb mode_prob_;
  TransMat transition_mat_;

  // keep model + ekf around
  KinematicModel model_;
  EKFType        ekf_;

  // remember user-supplied params/input
  ModelParams params_;
  ModelInput  input_;
};

} // namespace qcar_supervisor

