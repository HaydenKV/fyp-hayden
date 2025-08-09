#pragma once

#include <Eigen/Dense>

namespace qcar {
namespace params {

// ------------------------
// QCar Physical Parameters
// ------------------------

constexpr double l   = 0.256 / 2.0;   // [m] Distance from CoM to axles
constexpr double rw  = 0.033;         // [m] Wheel radius
constexpr double mc  = 2.8;           // [kg] Chassis mass
constexpr double mw  = 0.05;          // [kg] Wheel mass
constexpr double Izz = (1.0 / 12.0) * 2.7 * (0.425 * 0.425 + 0.192 * 0.192); // [kg·m²] Yaw inertia
constexpr double J22 = 0.5 * mw * rw * rw + 2e-3; // [kg·m²] Wheel pitch inertia
constexpr double J33 = 0.25 * mw * rw * rw;      // [kg·m²] Wheel yaw inertia
constexpr double g   = 9.81;          // [m/s²] Gravity

// ------------------------
// Electrical Actuator Parameters
// ------------------------

constexpr double Km  = 0.0027;        // [N·m/A] Motor constant
constexpr double Ra  = 0.47;          // [Ohm] Motor resistance
constexpr double Ng  = (13.0 / 70.0) * (19.0 / 37.0) * 10.0;  // Gear ratio
constexpr double Ks  = 0.03;          // [N·m/rad] Steering spring constant
constexpr double Rs  = 0.5;           // [Ohm] Steering resistance
constexpr double tau = 0.16;          // [s] Steering time constant

// ------------------------
// Friction
// ------------------------

constexpr double muw = 0.9;           // Wheel friction coefficient
constexpr double epsilon = 0.01;      // Small clamp to avoid division by zero

// ------------------------
// Initial state values
// ------------------------

inline Eigen::VectorXd initialState() {
    Eigen::VectorXd x0(10);
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, -M_PI_2, 0.1;
    return x0;
}

} // namespace params
} // namespace qcar
