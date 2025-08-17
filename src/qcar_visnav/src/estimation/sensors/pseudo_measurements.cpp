#include "qcar_visnav/estimation/sensors/pseudo_measurements.h"
#include <cmath>
#include <ros/ros.h>

namespace qcar_nav {

// ==================== NHC Measurement ====================

bool NHCMeasurement::shouldApply(const Eigen::Matrix<double, STATE_SIZE, 1>& state, 
                                 const NHCParams& params) {
  if (!params.enabled) {
    return false;
  }
  
  double vx = state(0);
  return std::abs(vx) >= params.velocity_threshold;
}

void NHCMeasurement::predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                            const KinematicModel::ModelParams& /* params */,
                            ZVec& h, HVec& H) {
  // NHC measurement: h = vy (lateral velocity should be ~0)
  h(0) = state(1);  // vy
  
  // Jacobian: dh/dx = [0, 1, 0, 0, 0, 0] for [vx, vy, r, bg, bax, bay]
  H.setZero();
  H(0, 1) = 1.0;    // dh/dvy = 1
}

double NHCMeasurement::getMeasurementStd(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                                        const NHCParams& params) {
  double vx = state(0);
  double r = state(2);
  
  // Adaptive noise: base + scaling * |r * vx|
  // When turning, allow more lateral velocity
  double adaptive_term = params.scaling_factor * std::abs(r * vx);
  return params.base_std + adaptive_term;
}

// ==================== ZUPT Measurement ====================

void ZUPTMeasurement::updateState(double velocity_measurement, double gyro_measurement,
                                 const ros::Time& timestamp, const ZUPTParams& params,
                                 ZUPTState& zupt_state) {
  bool currently_stationary = (std::abs(velocity_measurement) <= params.velocity_threshold) &&
                             (std::abs(gyro_measurement) <= params.gyro_threshold);
  
  if (currently_stationary) {
    if (!zupt_state.is_stationary) {
      // Just became stationary
      zupt_state.is_stationary = true;
      zupt_state.stationary_start = timestamp;
      zupt_state.stationary_duration = 0.0;
    } else {
      // Continue being stationary
      zupt_state.stationary_duration = (timestamp - zupt_state.stationary_start).toSec();
    }
  } else {
    // Not stationary
    zupt_state.is_stationary = false;
    zupt_state.stationary_duration = 0.0;
  }
}

bool ZUPTMeasurement::shouldApply(const ZUPTState& zupt_state, const ZUPTParams& params) {
  if (!params.enabled) {
    return false;
  }
  
  return zupt_state.is_stationary && (zupt_state.stationary_duration >= params.min_duration);
}

void ZUPTMeasurement::predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                             const KinematicModel::ModelParams& /* params */,
                             ZVec& h, HVec& H) {
  // ZUPT measurement: h = [vx, vy, r] (all should be ~0 when stationary)
  h(0) = state(0);  // vx
  h(1) = state(1);  // vy
  h(2) = state(2);  // r
  
  // Jacobian: dh/dx for [vx, vy, r, bg, bax, bay]
  H.setZero();
  H(0, 0) = 1.0;    // d(vx)/d(vx) = 1
  H(1, 1) = 1.0;    // d(vy)/d(vy) = 1
  H(2, 2) = 1.0;    // d(r)/d(r) = 1
}

Eigen::Matrix<double, ZUPTMeasurement::MEAS_SIZE, ZUPTMeasurement::MEAS_SIZE> 
ZUPTMeasurement::getMeasurementCovariance(const ZUPTParams& params) {
  Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R;
  R.setZero();
  
  R(0, 0) = params.velocity_std * params.velocity_std;      // vx variance
  R(1, 1) = params.velocity_std * params.velocity_std;      // vy variance
  R(2, 2) = params.yaw_rate_std * params.yaw_rate_std;      // r variance
  
  return R;
}

} // namespace qcar_nav