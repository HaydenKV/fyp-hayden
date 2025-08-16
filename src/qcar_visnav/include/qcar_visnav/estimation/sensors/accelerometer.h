#ifndef QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H
#define QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H

#include <Eigen/Dense>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

/**
 * @brief Accelerometer measurement class
 * Note: In the velocity EKF, accelerometers are typically used as INPUTS to the process model
 * rather than measurements. This class is provided for completeness and potential future use.
 */
class AccelerometerMeas {
public:
  static constexpr int MEAS_SIZE = 2;  // [ax, ay] measurements
  static constexpr int STATE_SIZE = KinematicModel::STATE_SIZE;
  
  using ZVec = Eigen::Matrix<double, MEAS_SIZE, 1>;
  using HVec = Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE>;

  AccelerometerMeas() = default;
  ~AccelerometerMeas() = default;

  /**
   * @brief Predict accelerometer measurements given current state
   * This predicts what the accelerometer should read based on estimated motion
   * @param state Current EKF state [vx, vy, r, bg, bax, bay]
   * @param params Model parameters
   * @param h Output predicted measurement [ax_pred, ay_pred]
   * @param H Output measurement Jacobian
   */
  void predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
               const KinematicModel::ModelParams& params,
               ZVec& h, HVec& H);

  /**
   * @brief Predict accelerometer measurements with additional vehicle dynamics
   * Includes centripetal acceleration terms
   * @param state Current EKF state
   * @param params Model parameters  
   * @param current_steering Current steering angle (rad)
   * @param h Output predicted measurement
   * @param H Output measurement Jacobian
   */
  void predictWithDynamics(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                          const KinematicModel::ModelParams& params,
                          double current_steering,
                          ZVec& h, HVec& H);

private:
  /**
   * @brief Compute expected body-frame accelerations from velocity derivatives
   * @param vx Forward velocity (m/s)
   * @param vy Lateral velocity (m/s)  
   * @param r Yaw rate (rad/s)
   * @param vx_dot Forward acceleration (m/s^2)
   * @param vy_dot Lateral acceleration (m/s^2)
   * @param bax Forward accelerometer bias (m/s^2)
   * @param bay Lateral accelerometer bias (m/s^2)
   * @return Expected accelerometer readings [ax_meas, ay_meas]
   */
  ZVec computeExpectedAccel(double vx, double vy, double r,
                           double vx_dot, double vy_dot,
                           double bax, double bay) const;
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_SENSORS_ACCELEROMETER_H