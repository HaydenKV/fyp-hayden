// src/transition_model.cpp
#include "qcar_supervisor/transition_model.h"
#include <Eigen/Eigenvalues>   // eigen-decomp (2x2 here)
#include <algorithm>
#include <cmath>

namespace qcar_nav {

void TransitionModel::setReferenceMatrix(const Eigen::Matrix2d& T_ref, double dt_ref) {
  T_ref_  = T_ref;
  dt_ref_ = dt_ref;
}

Eigen::Matrix2d TransitionModel::getTransition(double dt) const {
  // Time-scale via fractional matrix power: T(dt) = (T_ref)^(dt/dt_ref)
  const double alpha = (dt_ref_ > 0.0) ? (dt / dt_ref_) : 1.0;

  // Eigendecomposition (fine for 2x2 stochastic matrices)
  Eigen::ComplexEigenSolver<Eigen::Matrix2d> ces(T_ref_);
  Eigen::Matrix2cd V = ces.eigenvectors();
  Eigen::Matrix2cd D = ces.eigenvalues().asDiagonal();

  // Raise eigenvalues to fractional power (avoid Unicode; use "lam")
  for (int i = 0; i < D.rows(); ++i) {
    std::complex<double> lam = D.diagonal()(i);
    // Guard: clamp tiny values to 0 to avoid nans from pow(≈0, alpha)
    if (std::abs(lam.real()) < 1e-15 && std::abs(lam.imag()) < 1e-15) {
      lam = std::complex<double>(0.0, 0.0);
    }
    D.diagonal()(i) = std::pow(lam, alpha);
  }

  Eigen::Matrix2cd Tcd = V * D * V.inverse();
  Eigen::Matrix2d  T   = Tcd.real();

  // Hygiene: clip tiny negatives, ensure column-stochastic (columns sum to 1)
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < 2; ++i) if (T(i, j) < 0.0) T(i, j) = 0.0;
    double s = T.col(j).sum();
    if (s > 1e-12) T.col(j) /= s;
  }
  return T;
}

} // namespace qcar_nav
