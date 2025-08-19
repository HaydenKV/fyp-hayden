// src/main.cpp
#include "ekf.h"
#include "kinematic_model.h"
#include <iostream>

int main() {
  using namespace qcar_nav;

  // 1. Instantiate and initialize EKF
  ExtendedKalmanFilter ekf;
  KinematicModel::StateVec x0; 
  x0 << 0, 0, 0, 0, 0, 0;   // vx, vy, r, bg, bax, bay
  KinematicModel::StateMat P0 = 
    KinematicModel::StateMat::Identity() * 0.1;
  ekf.init(x0, P0);

  // 2. Set model parameters
  ModelParams params;
  params.L = 2.5;
  params.q << 0.01, 0.01, 0.001, 1e-6, 1e-6, 1e-6;
  ekf.setModelParams(params);

  // 3. Simulation loop (or real-time loop)
  double t = 0.0;
  double dt = 0.01;
  for (int i = 0; i < 100; ++i) {
    // a. Prepare and set input
    ModelInput input;
    input.a_meas_x = 0.2;
    input.a_meas_y = 0.0;
    input.delta    = 0.05;
    input.dt       = dt;
    ekf.setInput(input);

    // b. Predict step
    ekf.predict(t, dt);
    t += dt;

    // c. Example IMU measurement update
    Eigen::Vector2d z_imu(input.a_meas_x, input.delta);
    Eigen::Matrix2d R_imu = Eigen::Matrix2d::Identity() * 0.2;
    ekf.update(
      z_imu,
      [&](const auto& x){
        Eigen::Vector2d h;
        h << x(0) + x(4),  // vx + bias_x
             std::atan(x(2)); // yaw from r
        return h;
      },
      [&](const auto& x){
        Eigen::Matrix<double,2,6> H; 
        H.setZero();
        H(0,0) = 1; H(0,4) = 1;
        H(1,2) = 1.0/(1 + x(2)*x(2));
        return H;
      },
      R_imu
    );

    // d. Read out filtered state
    auto x_est = ekf.state();
    std::cout << "Step " << i << " | vx=" << x_est(0)
              << " vy=" << x_est(1) << " r=" << x_est(2)
              << std::endl;
  }

  return 0;
}

