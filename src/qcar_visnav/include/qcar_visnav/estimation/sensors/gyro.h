#ifndef QCAR_VISNAV_ESTIMATION_SENSORS_GYRO_H
#define QCAR_VISNAV_ESTIMATION_SENSORS_GYRO_H

#include <Eigen/Dense>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

/**
 * @brief Gyroscope measurement class for yaw rate
 * Measures: z_gyro = r + bg + noise
 */
class GyroMeas {
public:
  static constexpr int MEAS_SIZE = 1;  // Single measurement: yaw rate
  static constexpr int STATE_SIZE = KinematicModel::STATE_SIZE;
  
  using ZVec = Eigen::Matrix<double, MEAS_SIZE, 1>;
  using HVec = Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE>;

  GyroMeas() = default;
  ~GyroMeas() = default;

  /**
   * @brief Predict measurement and compute Jacobian
   * @param state Current EKF state [vx, vy, r, bg, bax, bay]
   * @param params Model parameters
   * @param h Output predicted measurement (r + bg)
   * @param H Output measurement Jacobian
   */
  void predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
               const KinematicModel::ModelParams& params,
               ZVec& h, HVec& H);
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_SENSORS_GYRO_H