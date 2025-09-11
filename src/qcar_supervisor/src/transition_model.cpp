#include "qcar_supervisor/transition_model.h"
#include <Eigen/Eigenvalues>
#include <stdexcept>
#include <cmath>

namespace qcar_nav {

void TransitionModel::setReferenceMatrix(const Eigen::Matrix2d& T_ref, double dt_ref) {
  if ((T_ref.array() < 0.0).any()) {
    throw std::invalid_argument("Transition matrix contains negative entries");
  }
  if (!T_ref.rowwise().sum().isApprox(Eigen::Vector2d::Ones(), 1e-6)) {
    throw std::invalid_argument("Transition matrix rows must sum to 1");
  }

  T_ref_ = T_ref;
  dt_ref_ = dt_ref;
}

Eigen::Matrix2d TransitionModel::getTransition(double dt) const {
  if (dt <= 0.0) {
    throw std::invalid_argument("Time step must be positive");
  }

  // Compute T(dt) = T_ref ^ (dt / dt_ref)
  double scale = dt / dt_ref_;

  // Eigen decomposition: T_ref = V D V⁻¹
  Eigen::EigenSolver<Eigen::Matrix2d> solver(T_ref_);
  Eigen::Matrix2cd D = solver.eigenvalues().asDiagonal();
  Eigen::Matrix2cd V = solver.eigenvectors();
  Eigen::Matrix2cd V_inv = V.inverse();

  // Raise eigenvalues to fractional power
  for (int i = 0; i < 2; ++i) {
    D(i, i) = std::pow(D(i, i), scale);
  }

  // Reconstruct T(dt)
  Eigen::Matrix2cd T_dt_complex = V * D * V_inv;
  Eigen::Matrix2d T_dt = T_dt_complex.real();

  // Clip to [0, 1] and renormalize rows
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      T_dt(i, j) = std::clamp(T_dt(i, j), 0.0, 1.0);
    }
    double row_sum = T_dt.row(i).sum();
    if (row_sum > 1e-6) {
      T_dt.row(i) /= row_sum;
    }
  }

  return T_dt;
}

} // namespace qcar_nav
