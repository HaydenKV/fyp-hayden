#ifndef QCAR_JACOBIANS_H
#define QCAR_JACOBIANS_H

#include <Eigen/Dense>
#include "params.h"

namespace qcar {
namespace model {

/**
 * @brief Computes the Jacobian of the dynamics function with respect to the state vector x.
 * 
 * @param params Parameters of the QCar model.
 * @param x State vector [u; v; r; omegaF; omegaR; delta; N; E; psi].
 * @param v Velocity-related states (first 6 elements of x).
 * @return Eigen::MatrixXd The Jacobian matrix ∂f/∂x.
 */
Eigen::MatrixXd computeJacobianX(const Params &params, const Eigen::VectorXd &x, const Eigen::VectorXd &v);

/**
 * @brief Computes the Jacobian of the dynamics function with respect to the input vector u.
 * 
 * @param params Parameters of the QCar model.
 * @return Eigen::MatrixXd The Jacobian matrix ∂f/∂u.
 */
Eigen::MatrixXd computeJacobianU(const Params &params);

}  // namespace model
}  // namespace qcar

#endif  // QCAR_JACOBIANS_H
