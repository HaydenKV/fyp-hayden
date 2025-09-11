#pragma once

#include <array>
#include <functional>
#include <Eigen/Dense>

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_visnav/estimation/ekf/ekf_core.h"

namespace qcar_supervisor {

using KinematicModel = qcar_nav::KinematicModel;

/// Enumerates the possible modes (faults + healthy)
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

  using StateVec    = KinematicModel::StateVec;
  using StateMat    = KinematicModel::StateMat;
  using ModelParams = KinematicModel::ModelParams;
  using ModelInput  = KinematicModel::ModelInput;

  using ModeProb = Eigen::Matrix<double, M, 1>;
  using TransMat = Eigen::Matrix<double, M, M>;

  using EKFType  = qcar_nav::EkfCore<KinematicModel::STATE_SIZE>;

  using HfunType = std::function<Eigen::VectorXd(const StateVec&)>;
  using HmatType = std::function<Eigen::MatrixXd(const StateVec&)>;

  JumpMarkovFilter();
  ~JumpMarkovFilter() = default;

  /// Set one ModelParams per mode
  void setModelParams(const std::array<ModelParams, M>& mode_params);

  /// Shared control input
  void setInput(const ModelInput& input);

  void init(const StateVec& x0,
            const StateMat& P0,
            const ModeProb& init_mode_prob,
            const TransMat& transition_mat);

  void predict(double t, double dt);

  void update(const Eigen::VectorXd&  z,
              const HfunType&         h_fun,
              const HmatType&         H_fun,
              const Eigen::MatrixXd&  R);

  StateVec   fusedState() const;
  StateMat   fusedCovar() const;
  ModeProb   modeProb()   const;
  int        mostLikelyMode() const;

private:
  std::array<StateVec, M> x_pred_, x_upd_;
  std::array<StateMat, M> P_pred_, P_upd_;

  ModeProb   mode_prob_;
  TransMat   transition_mat_;

  KinematicModel                    model_;
  EKFType                           ekf_;

  std::array<ModelParams, M>       mode_params_;
  ModelInput                        input_;
};

} // namespace qcar_supervisor
