#include "jacobians.h"
#include <cmath>

namespace qcar {
namespace model {

Eigen::MatrixXd computeJacobianX(const Params &params, const Eigen::VectorXd &x, const Eigen::VectorXd &v) {
    Eigen::MatrixXd Jx = Eigen::MatrixXd::Zero(9, 9);

    const double epsilon = 0.01;
    const double l = params.l;
    const double den = 2 * l;
    const double delta = v(5);
    const double psi = x(8);
    const double u = v(0);
    const double tan_d = std::tan(delta);
    const double sec_d = 1.0 / std::cos(delta);
    const double beta = std::atan((l / den) * tan_d);
    const double dBeta_dDelta = (l / den) * sec_d * sec_d / (1.0 + std::pow((l / den) * tan_d, 2));

    const double cosBpsi = std::cos(beta + psi);
    const double sinBpsi = std::sin(beta + psi);

    // Damping
    Eigen::MatrixXd D_total = dampingMatrix(params, x);

    // Coriolis term
    Eigen::MatrixXd C = coriolisMatrix(params, x);

    // Chassis dynamics
    Eigen::MatrixXd dMDvdvE = D_total;
    Eigen::MatrixXd dMCvdv = params.invMC * (v(2) * Eigen::MatrixXd::Identity(6, 6));
    dMCvdv.block(0, 2, 6, 1) += params.invMC * v;

    Jx.block(0, 0, 6, 6) = -dMCvdv - dMDvdvE;

    // Steering
    Jx(5, 5) = -1.0 / params.tau;

    // Pose derivatives
    Jx(6, 0) = cosBpsi;
    Jx(6, 5) = -u * sinBpsi * dBeta_dDelta;
    Jx(6, 8) = -u * sinBpsi;

    Jx(7, 0) = sinBpsi;
    Jx(7, 5) = u * cosBpsi * dBeta_dDelta;
    Jx(7, 8) = u * cosBpsi;

    Jx(8, 0) = (cosBpsi / den) * tan_d;
    Jx(8, 5) = (u * cosBpsi / den) * sec_d * sec_d +
               (-u * std::sin(beta) / den) * tan_d * dBeta_dDelta;

    return Jx;
}

Eigen::MatrixXd computeJacobianU(const Params &params) {
    Eigen::MatrixXd Ju = Eigen::MatrixXd::Zero(9, 2);
    Ju.block(0, 0, 6, 1) = params.invMBa;
    Ju(5, 1) = 1.0 / params.tau;
    return Ju;
}

}  // namespace model
}  // namespace qcar
