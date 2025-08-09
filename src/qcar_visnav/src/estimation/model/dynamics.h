#pragma once

#include <Eigen/Dense>
#include "parameters.h"

namespace qcar {
namespace dynamics {

/**
 * @brief Computes the state derivative f(x,u) for the QCar model.
 *
 * @param t Current time [s] (unused in time-invariant model but included for interface consistency)
 * @param x State vector (size 10)
 * @param u Control input vector (size 2)
 * @return State derivative vector (size 9)
 */
Eigen::VectorXd computeDynamics(double t, const Eigen::VectorXd& x, const Eigen::Vector2d& u);

} // namespace dynamics
} // namespace qcar
