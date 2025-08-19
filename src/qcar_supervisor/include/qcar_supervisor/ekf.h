#pragma once

#include <Eigen/Dense>
#include <functional>                       // <-- for std::function
#include <qcar_supervisor/model/kinematic_model.h>

namespace qcar_supervisor {

class ExtendedKalmanFilter
{
public:
  // Pull these static constants into local aliases
  static constexpr int n_x = KinematicModel::STATE_SIZE;
  static constexpr int n_u = KinematicModel::INPUT_SIZE;
  
  // shortcut aliases from your KinematicModel
  using StateVec  = typename KinematicModel::StateVec;
  using StateMat  = typename KinematicModel::StateMat;
  using InputMat  = typename KinematicModel::InputMat;
  using NoiseMat  = Eigen::Matrix<double, n_u, n_u>;
  
  // model-specific configuration & control types
  using ModelParams = typename KinematicModel::ModelParams;
  using ModelInput  = typename KinematicModel::ModelInput;

  ExtendedKalmanFilter();

  // initialize state and covariance
  void init(const StateVec& x0,
            const StateMat& P0);

  // configure the underlying model
  void setModelParams(const ModelParams& params);

  // feed in control/sensor input for this step
  void setInput(const ModelInput& input);

  // propagate state & covariance: time update
  void predict(double t, double dt);

    // — Measurement types & function signatures —
    using MeasVec   = Eigen::VectorXd;    // m×1 measurement
    using MeasMat   = Eigen::MatrixXd;    // m×n_x Jacobian
    using HfunType  = std::function<MeasVec(const StateVec&)>;   // h(x)
    using HmatType  = std::function<MeasMat(const StateVec&)>;   // H(x)

    // measurement update
    void update(
      const Eigen::VectorXd&                        z,
      std::function<Eigen::VectorXd(const StateVec&)> h,
      std::function<Eigen::MatrixXd(const StateVec&)> H,
      const Eigen::MatrixXd&                        R
    );

  // accessors
  const StateVec& state() const { return x_; }
  const StateMat& covar() const { return P_; }

private:
  KinematicModel model_;  // your process model
  StateVec       x_;      // current mean
  StateMat       P_;      // current covariance
  NoiseMat       R_u_;    // input/process noise from model_.getInputNoise()
};

} // namespace qcar_supervisor

