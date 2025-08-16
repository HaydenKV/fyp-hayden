#ifndef QCAR_VISNAV_ESTIMATION_MODEL_KINEMATIC_MODEL_H
#define QCAR_VISNAV_ESTIMATION_MODEL_KINEMATIC_MODEL_H

#include <Eigen/Dense>
#include <cmath>

namespace qcar_nav {

/**
 * @brief Velocity-based kinematic model for QCar
 * State: [vx, vy, r, bg, bax, bay]
 * - vx, vy: body-frame velocities (m/s)
 * - r: yaw rate (rad/s)
 * - bg: gyro bias (rad/s)
 * - bax, bay: accelerometer biases (m/s^2)
 */
class KinematicModel {
public:
  static constexpr int STATE_SIZE = 6;
  static constexpr int INPUT_SIZE = 3;
  
  using StateVec = Eigen::Matrix<double, STATE_SIZE, 1>;
  using InputVec = Eigen::Matrix<double, INPUT_SIZE, 1>;
  using StateMat = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;
  using InputMat = Eigen::Matrix<double, STATE_SIZE, INPUT_SIZE>;

  struct ModelParams {
    double L;           // wheelbase (m)
    double dtMaxEst;    // maximum time step for integration (s)
    Eigen::Matrix<double, STATE_SIZE, 1> q;  // process noise std devs
    
    ModelParams() : L(0.258), dtMaxEst(0.01) {  // QCar wheelbase
      q.setOnes();
    }
  };

  struct ModelInput {
    double a_meas_x;    // measured accel x (gravity-free) (m/s^2)
    double a_meas_y;    // measured accel y (gravity-free) (m/s^2)  
    double delta;       // steering angle (rad)
    double dt;          // time step (s)
    
    ModelInput() : a_meas_x(0), a_meas_y(0), delta(0), dt(0.01) {}
  };

  KinematicModel();
  ~KinematicModel() = default;

  /**
   * @brief Set model parameters
   */
  void setParams(const ModelParams& params);

  /**
   * @brief Set current input for the model
   */
  void setInput(const ModelInput& input);

  /**
   * @brief Predict state forward by dt
   * @param x_in Input state [vx, vy, r, bg, bax, bay]
   * @param t Current time (for interface compatibility)
   * @param dt Time step
   * @return Predicted state
   */
  StateVec predict(const StateVec& x_in, double t, double dt) const;

  /**
   * @brief Compute process Jacobian F = df/dx
   * @param x State vector
   * @param dt Time step
   * @return Process Jacobian matrix
   */
  StateMat getProcessJacobian(const StateVec& x, double dt) const;

  /**
   * @brief Compute input Jacobian G = df/du
   * @param x State vector  
   * @param dt Time step
   * @return Input Jacobian matrix
   */
  InputMat getInputJacobian(const StateVec& x, double dt) const;

  /**
   * @brief Get process noise covariance matrix Q
   * @param dt Time step
   * @return Process noise covariance
   */
  StateMat getProcessNoise(double dt) const;

  /**
   * @brief Get input noise covariance matrix
   * @param dt Time step
   * @return Input noise covariance
   */
  Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> getInputNoise(double dt) const;

  // Legacy interface compatibility for RK4SDE integrator
  /**
   * @brief State derivative function (legacy interface)
   * @param t Time (unused)
   * @param x State vector
   * @return State derivative
   */
  StateVec f(double t, const StateVec& x) const;

  /**
   * @brief Jacobian computation (legacy interface)
   * @param t Time (unused)
   * @param x State vector
   * @param Jac Output Jacobian matrix
   */
  void jacobian(double t, const StateVec& x, StateMat& Jac) const;

  /**
   * @brief Process noise (legacy interface)
   * @param dt Time step
   * @param sqrtQc Output noise standard deviations
   * @param L Output noise distribution matrix
   */
  void processNoise(double dt, Eigen::Vector3d& sqrtQc, Eigen::Matrix<double, STATE_SIZE, 3>& L) const;

  /**
   * @brief Get parameters (legacy interface)
   */
  const ModelParams& params() const { return params_; }

  // Getters
  const ModelParams& getParams() const { return params_; }
  const ModelInput& getInput() const { return input_; }

private:
  ModelParams params_;
  ModelInput input_;

  /**
   * @brief Continuous-time state derivative
   * @param x State vector
   * @param u Input vector [a_meas_x, a_meas_y, delta]
   * @return State derivative
   */
  StateVec stateDot(const StateVec& x, const InputVec& u) const;
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_MODEL_KINEMATIC_MODEL_H