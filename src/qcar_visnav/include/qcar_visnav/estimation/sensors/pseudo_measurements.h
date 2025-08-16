#ifndef QCAR_VISNAV_ESTIMATION_SENSORS_PSEUDO_MEASUREMENTS_H
#define QCAR_VISNAV_ESTIMATION_SENSORS_PSEUDO_MEASUREMENTS_H

#include <Eigen/Dense>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

/**
 * @brief No Horizontal Coupling (NHC) pseudo-measurement
 * Assumes vy ≈ 0 when vehicle is moving forward at sufficient speed
 * Helps constrain lateral velocity drift
 */
class NHCMeasurement {
public:
  static constexpr int MEAS_SIZE = 1;
  static constexpr int STATE_SIZE = KinematicModel::STATE_SIZE;
  
  using ZVec = Eigen::Matrix<double, MEAS_SIZE, 1>;
  using HVec = Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE>;

  struct NHCParams {
    double velocity_threshold;    // Minimum |vx| to apply NHC (m/s)
    double base_std;             // Base measurement std (m/s)
    double scaling_factor;       // Scale factor for |r*vx| term
    bool enabled;                // Enable/disable NHC
    
    NHCParams() 
      : velocity_threshold(1.0), base_std(0.05), scaling_factor(1.0), enabled(true) {}
  };

  NHCMeasurement() = default;
  ~NHCMeasurement() = default;

  /**
   * @brief Check if NHC should be applied given current state
   * @param state Current EKF state [vx, vy, r, bg, bax, bay]
   * @param params NHC parameters
   * @return true if NHC should be applied
   */
  static bool shouldApply(const Eigen::Matrix<double, STATE_SIZE, 1>& state, 
                         const NHCParams& params);

  /**
   * @brief Predict measurement and compute Jacobian
   * @param state Current EKF state
   * @param params Model parameters (unused for NHC)
   * @param h Output predicted measurement
   * @param H Output measurement Jacobian
   */
  void predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
               const KinematicModel::ModelParams& params,
               ZVec& h, HVec& H);

  /**
   * @brief Get measurement noise covariance
   * @param state Current state (for adaptive noise)
   * @param params NHC parameters
   * @return Measurement noise standard deviation
   */
  static double getMeasurementStd(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                                 const NHCParams& params);
};

/**
 * @brief Zero Velocity Update (ZUPT) pseudo-measurement
 * Assumes [vx, vy, r] ≈ [0, 0, 0] when vehicle is stationary
 * Applied when both velocity and yaw rate measurements are near zero
 */
class ZUPTMeasurement {
public:
  static constexpr int MEAS_SIZE = 3;  // [vx, vy, r]
  static constexpr int STATE_SIZE = KinematicModel::STATE_SIZE;
  
  using ZVec = Eigen::Matrix<double, MEAS_SIZE, 1>;
  using HVec = Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE>;

  struct ZUPTParams {
    double velocity_threshold;    // Max |vx| for ZUPT (m/s)
    double gyro_threshold;       // Max |gyro_z| for ZUPT (rad/s)
    double min_duration;         // Min time stationary before applying ZUPT (s)
    double velocity_std;         // Velocity measurement std during ZUPT (m/s)
    double yaw_rate_std;         // Yaw rate measurement std during ZUPT (rad/s)
    bool enabled;                // Enable/disable ZUPT
    
    ZUPTParams() 
      : velocity_threshold(0.1), gyro_threshold(0.05), min_duration(0.3),
        velocity_std(0.01), yaw_rate_std(0.01), enabled(true) {}
  };

  struct ZUPTState {
    bool is_stationary;          // Current stationary status
    ros::Time stationary_start;  // When stationary period began
    double stationary_duration;  // How long stationary (s)
    
    ZUPTState() : is_stationary(false), stationary_duration(0.0) {}
  };

  ZUPTMeasurement() = default;
  ~ZUPTMeasurement() = default;

  /**
   * @brief Update ZUPT state based on current measurements
   * @param velocity_measurement Current velocity magnitude (m/s)
   * @param gyro_measurement Current gyro z measurement (rad/s)
   * @param timestamp Current timestamp
   * @param params ZUPT parameters
   * @param zupt_state Input/output ZUPT state
   */
  static void updateState(double velocity_measurement, double gyro_measurement,
                         const ros::Time& timestamp, const ZUPTParams& params,
                         ZUPTState& zupt_state);

  /**
   * @brief Check if ZUPT should be applied
   * @param zupt_state Current ZUPT state
   * @param params ZUPT parameters
   * @return true if ZUPT should be applied
   */
  static bool shouldApply(const ZUPTState& zupt_state, const ZUPTParams& params);

  /**
   * @brief Predict measurement and compute Jacobian
   * @param state Current EKF state
   * @param params Model parameters (unused for ZUPT)
   * @param h Output predicted measurement [vx, vy, r]
   * @param H Output measurement Jacobian
   */
  void predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
               const KinematicModel::ModelParams& params,
               ZVec& h, HVec& H);

  /**
   * @brief Get measurement noise covariance matrix
   * @param params ZUPT parameters
   * @return 3x3 measurement noise covariance
   */
  static Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> getMeasurementCovariance(const ZUPTParams& params);
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_SENSORS_PSEUDO_MEASUREMENTS_H