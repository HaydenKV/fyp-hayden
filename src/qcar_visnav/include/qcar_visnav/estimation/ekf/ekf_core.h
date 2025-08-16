#ifndef QCAR_VISNAV_ESTIMATION_EKF_EKF_CORE_H
#define QCAR_VISNAV_ESTIMATION_EKF_EKF_CORE_H

#include <Eigen/Dense>

namespace qcar_nav {

// Forward declaration
class KinematicModel;

/**
 * @brief Square-root Extended Kalman Filter for 6-state velocity model
 * Maintains state estimate as (mu, S) where P = S^T * S
 * State: [vx, vy, r, bg, bax, bay]
 */
template<int STATE_SIZE>
class EkfCore {
public:
  using Vec = Eigen::Matrix<double, STATE_SIZE, 1>;
  using Mat = Eigen::Matrix<double, STATE_SIZE, STATE_SIZE>;

  EkfCore();
  ~EkfCore() = default;

  /**
   * @brief Initialize EKF with initial state and covariance
   * @param mu0 Initial state estimate
   * @param S0 Initial square-root covariance (upper triangular)
   */
  void setInitial(const Vec& mu0, const Mat& S0);

  /**
   * @brief Get current state estimate
   */
  const Vec& mu() const { return mu_; }

  /**
   * @brief Get current square-root covariance
   */
  const Mat& S() const { return S_; }

  /**
   * @brief Get current covariance matrix P = S^T * S
   */
  Mat getCovariance() const { return S_.transpose() * S_; }

  /**
   * @brief Predict step using motion model
   * @param model Kinematic model with current inputs set
   * @param t Current time (for interface compatibility)
   * @param dt Time step
   */
  void predict(const KinematicModel& model, double t, double dt);

  /**
   * @brief Update step with measurement
   * @param z Measurement vector
   * @param h Predicted measurement
   * @param H Measurement Jacobian
   * @param sqrtR Square-root measurement noise covariance
   */
  void update(const Eigen::VectorXd& z,
              const Eigen::VectorXd& h,
              const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& sqrtR);

private:
  Vec mu_;  // State estimate
  Mat S_;   // Square-root covariance (upper triangular)

  /**
   * @brief Ensure covariance matrix is symmetric and positive definite
   * @param P Covariance matrix to regularize
   * @return Regularized covariance matrix
   */
  Mat regularizeCovariance(const Mat& P) const;
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_EKF_EKF_CORE_H