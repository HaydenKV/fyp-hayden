#include "qcar_visnav/estimation/ekf/ekf_core.h"
#include "qcar_visnav/estimation/model/kinematic_model.h"
#include <cmath>
#include <iostream>

namespace qcar_nav {

template<int STATE_SIZE>
EkfCore<STATE_SIZE>::EkfCore() {
  mu_.setZero();
  S_.setIdentity();
  S_ *= 0.1;  // Small initial uncertainty (overridden by setInitial)
}

template<int STATE_SIZE>
void EkfCore<STATE_SIZE>::setInitial(const Vec& mu0, const Mat& P0_in) {
  mu_ = mu0;

  // Make symmetric & clamp tiny negatives (robust to rounding)
  Mat P0 = regularizeCovariance(P0_in);

  // Try Cholesky (SPD expected)
  Eigen::LLT<Mat> llt(P0);
  if (llt.info() == Eigen::Success) {
    S_ = llt.matrixU();  // Upper-triangular so that P0 = S^T S
    return;
  }

  // Fallback: eigen factorization (handles PSD)
  Eigen::SelfAdjointEigenSolver<Mat> eig(P0);
  if (eig.info() == Eigen::Success) {
    auto evals = eig.eigenvalues().array().max(1e-12); // jitter
    Mat sqrt_lambda = evals.sqrt().matrix().asDiagonal();
    S_ = eig.eigenvectors() * sqrt_lambda;            // not guaranteed upper-triangular but OK
    return;
  }

  // Last resort
  S_.setIdentity();
  S_ *= 0.1;
}


template<int STATE_SIZE>
void EkfCore<STATE_SIZE>::predict(const KinematicModel& model, double t, double dt) {
  // 1. Propagate state: x_pred = f(x_prev, u, dt)
  // 2. Compute Jacobians: F = ∂f/∂x, G = ∂f/∂u  
  // 3. Update covariance: P⁻ = F*P*Fᵀ + G*Qu*Gᵀ + Qx
  // 4. Maintain square-root form: S where P = SᵀS

  // 1. Predict state using kinematic model
  auto x_pred = model.predict(mu_, t, dt);

  // 2. Compute Jacobians - these return the correct 6x6 and 6x3 matrices
  auto F = model.getProcessJacobian(mu_, dt);        // 6x6 
  auto G = model.getInputJacobian(mu_, dt);          // 6x3

  // 3. Compute noise covariances (discrete for this step)
  auto Qx = model.getProcessNoise(dt);               // 6x6 (built from YAML q * dt)
  auto Qu = model.getInputNoise(dt);                 // 3x3

  // 4. Propagate covariance: P^- = F*P*F^T + G*Qu*G^T + Qx
  Mat P = S_.transpose() * S_;                       // 6x6
  Mat P_pred = F * P * F.transpose() + G * Qu * G.transpose() + Qx;  // All 6x6

  // 5. Regularize and update
  P_pred = regularizeCovariance(P_pred);
  
  // 6. Compute new square-root via Cholesky decomposition
  Eigen::LLT<Mat> llt(P_pred);
  if (llt.info() == Eigen::Success) {
    S_ = llt.matrixU();  // Upper triangular
  } else {
    // Fallback: use eigenvalue decomposition for numerical stability
    Eigen::SelfAdjointEigenSolver<Mat> eigensolver(P_pred);
    if (eigensolver.info() == Eigen::Success) {
      auto eigenvals = eigensolver.eigenvalues().array().max(1e-12);
      Mat sqrt_lambda = eigenvals.sqrt().matrix().asDiagonal();
      S_ = eigensolver.eigenvectors() * sqrt_lambda;
    } else {
      // Last resort: identity with small scaling
      S_.setIdentity();
      S_ *= 0.1;
      std::cerr << "[EKF] Warning: Covariance decomposition failed, resetting to identity" << std::endl;
    }
  }

  // 7. Update state
  mu_ = x_pred;

  // No angle wrapping needed for velocity-only EKF
}

template<int STATE_SIZE>
void EkfCore<STATE_SIZE>::update(const Eigen::VectorXd& z,
                                 const Eigen::VectorXd& h,
                                 const Eigen::MatrixXd& H,
                                 const Eigen::MatrixXd& sqrtR) {
  // 1. Current covariance
  Mat P = S_.transpose() * S_;

  // 2. Innovation
  Eigen::VectorXd y = z - h;

  // 3. Innovation covariance
  Eigen::MatrixXd R = sqrtR.transpose() * sqrtR; // (std)^2
  Eigen::MatrixXd S_innov = H * P * H.transpose() + R;

  // 4. Kalman gain
  Eigen::MatrixXd K = P * H.transpose() * S_innov.inverse();

  // 5. State update
  mu_ += K * y;

  // 6. Covariance update using Joseph form for numerical stability
  Mat I = Mat::Identity();
  Mat P_updated = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  // 7. Regularize and update square-root
  P_updated = regularizeCovariance(P_updated);
  
  Eigen::LLT<Mat> llt(P_updated);
  if (llt.info() == Eigen::Success) {
    S_ = llt.matrixU();
  } else {
    // Fallback using eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Mat> eigensolver(P_updated);
    if (eigensolver.info() == Eigen::Success) {
      auto eigenvals = eigensolver.eigenvalues().array().max(1e-12);
      Mat sqrt_lambda = eigenvals.sqrt().matrix().asDiagonal();
      S_ = eigensolver.eigenvectors() * sqrt_lambda;
    }
  }
}

template<int STATE_SIZE>
typename EkfCore<STATE_SIZE>::Mat EkfCore<STATE_SIZE>::regularizeCovariance(const Mat& P) const {
  // Make symmetric
  Mat P_sym = 0.5 * (P + P.transpose());
  
  // Ensure positive definiteness by adding small diagonal regularization
  const double min_eigenval = 1e-12;
  Eigen::SelfAdjointEigenSolver<Mat> eigensolver(P_sym);
  
  if (eigensolver.info() == Eigen::Success) {
    auto eigenvals = eigensolver.eigenvalues();
    if (eigenvals.minCoeff() < min_eigenval) {
      // Add regularization
      auto regularized_eigenvals = eigenvals.array().max(min_eigenval);
      Mat Lambda_reg = regularized_eigenvals.matrix().asDiagonal();
      return eigensolver.eigenvectors() * Lambda_reg * eigensolver.eigenvectors().transpose();
    }
  }
  
  return P_sym;
}

// Explicit template instantiation for 6-state velocity EKF only
template class EkfCore<6>;

} // namespace qcar_nav