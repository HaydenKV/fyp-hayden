// File: qcar_visnav/src/estimation/model/dynamics.cpp
// This file defines the 10-state dynamics model for the QCar.

#include "dynamics.h"
#include "parameters.h"
#include "wheel_slip.h"

#include <Eigen/Dense>
#include <cmath>

using namespace Eigen;

VectorXd computeDynamics(double t, const VectorXd &x, const VectorXd &u, const QCarParams &params) {
    // Extract state variables
    VectorXd v = x.segment<6>(0);         // [u, v, r, omegaF, omegaR, delta]
    double delta = v(5);
    double u_chassis = v(0);
    double v_chassis = v(1);
    double r_chassis = v(2);
    double psi = x(8);

    // Input
    double drive_input = u(0);
    double steering_input = u(1);

    // Steering dynamics
    double delta_dot = (steering_input - delta) / params.tau;

    // Wheel slip
    Matrix<double, 3, 6> AF, AR;
    Vector3d vWFNb, vWRNb;
    std::tie(vWFNb, vWRNb, AF, AR) = computeWheelSlip(x, params);

    // Damping
    double epsilon = 0.01;
    double betaF = params.muw * params.Wf / std::sqrt(vWFNb.dot(vWFNb) + epsilon * epsilon);
    double betaR = params.muw * params.Wr / std::sqrt(vWRNb.dot(vWRNb) + epsilon * epsilon);
    Matrix<double, 6, 6> DF = betaF * (AF.transpose() * AF);
    Matrix<double, 6, 6> DR = betaR * (AR.transpose() * AR);
    Matrix<double, 6, 6> D_total = params.Da + DF + DR;

    // Coriolis (precomputed scalar form)
    Matrix<double, 6, 6> C = params.CScalar;
    C.block<3,6>(0,0) *= r_chassis;

    // Dynamics
    VectorXd drive_acc = params.invMBa * drive_input - r_chassis * params.invMC * v - D_total * v;

    VectorXd v_dot(6);
    v_dot = drive_acc;
    v_dot(5) = delta_dot;  // Override delta dynamics

    // Pose kinematics
    double beta = std::atan((params.l / (params.l + params.l)) * std::tan(delta));
    double r_effective = (u_chassis * std::cos(beta)) / (2 * params.l) * std::tan(delta);

    Vector3d eta_dot;
    eta_dot(0) = u_chassis * std::cos(beta + psi);  // North
    eta_dot(1) = u_chassis * std::sin(beta + psi);  // East
    eta_dot(2) = r_effective;                       // Yaw

    // Assemble full derivative vector
    VectorXd f(9);
    f << v_dot, eta_dot;
    return f;
}
