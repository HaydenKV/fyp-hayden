#include "qcar_visnav/estimation/sensors/accelerometer.h"
#include <cmath>

namespace qcar_nav {

void AccelerometerMeas::predict(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                               const KinematicModel::ModelParams& params,
                               ZVec& h, HVec& H) {
  // Extract state variables
  double vx = state(0);   // Forward velocity
  double vy = state(1);   // Lateral velocity  
  double r = state(2);    // Yaw rate
  double bg = state(3);   // Gyro bias (unused here)
  double bax = state(4);  // Forward accel bias
  double bay = state(5);  // Lateral accel bias

  // Predict body-frame accelerations based on velocity kinematics
  // For steady-state motion: ax = r * vy, ay = -r * vx (centripetal acceleration)
  // Plus biases: ax_meas = ax_true + bax, ay_meas = ay_true + bay
  
  h(0) = r * vy + bax;    // Forward acceleration measurement
  h(1) = -r * vx + bay;   // Lateral acceleration measurement

  // Compute Jacobian H = dh/dx for state [vx, vy, r, bg, bax, bay]
  H.setZero();
  
  // Row 0: d(ax_pred)/dx = [0, r, vy, 0, 1, 0]
  H(0, 0) = 0.0;   // d(ax)/dvx = 0
  H(0, 1) = r;     // d(ax)/dvy = r
  H(0, 2) = vy;    // d(ax)/dr = vy
  H(0, 3) = 0.0;   // d(ax)/dbg = 0
  H(0, 4) = 1.0;   // d(ax)/dbax = 1
  H(0, 5) = 0.0;   // d(ax)/dbay = 0
  
  // Row 1: d(ay_pred)/dx = [-r, 0, -vx, 0, 0, 1]
  H(1, 0) = -r;    // d(ay)/dvx = -r
  H(1, 1) = 0.0;   // d(ay)/dvy = 0
  H(1, 2) = -vx;   // d(ay)/dr = -vx
  H(1, 3) = 0.0;   // d(ay)/dbg = 0
  H(1, 4) = 0.0;   // d(ay)/dbax = 0
  H(1, 5) = 1.0;   // d(ay)/dbay = 1
}

void AccelerometerMeas::predictWithDynamics(const Eigen::Matrix<double, STATE_SIZE, 1>& state,
                                           const KinematicModel::ModelParams& params,
                                           double current_steering,
                                           ZVec& h, HVec& H) {
  // Extract state variables
  double vx = state(0);
  double vy = state(1);
  double r = state(2);
  double bax = state(4);
  double bay = state(5);

  // Compute expected velocity derivatives from kinematic model
  // vx_dot = r * vy (from centripetal effects)
  // vy_dot = -r * vx
  // These represent the "true" accelerations before bias
  double vx_dot = r * vy;
  double vy_dot = -r * vx;
  
  // For more sophisticated prediction, could include steering-based yaw acceleration:
  // r_dot = (vx / L) * tan(steering)
  // And corresponding acceleration coupling terms

  // Predicted measurements (true acceleration + bias)
  h = computeExpectedAccel(vx, vy, r, vx_dot, vy_dot, bax, bay);

  // Jacobian (same as simple case for now)
  H.setZero();
  H(0, 1) = r;     // d(ax)/dvy = r
  H(0, 2) = vy;    // d(ax)/dr = vy  
  H(0, 4) = 1.0;   // d(ax)/dbax = 1
  H(1, 0) = -r;    // d(ay)/dvx = -r
  H(1, 2) = -vx;   // d(ay)/dr = -vx
  H(1, 5) = 1.0;   // d(ay)/dbay = 1
}

AccelerometerMeas::ZVec AccelerometerMeas::computeExpectedAccel(double vx, double vy, double r,
                                                               double vx_dot, double vy_dot,
                                                               double bax, double bay) const {
  ZVec expected;
  
  // True body-frame accelerations plus biases
  expected(0) = vx_dot + bax;  // Forward acceleration measurement
  expected(1) = vy_dot + bay;  // Lateral acceleration measurement
  
  return expected;
}

} // namespace qcar_nav