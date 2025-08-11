#pragma once
#include <Eigen/Dense>
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_nav {

// Square-root EKF storing (mu, S) with upper-triangular S: P = S' S
class SREKF {
public:
  static constexpr int NX = 7;
  using Vec = Eigen::Matrix<double,NX,1>;
  using Mat = Eigen::Matrix<double,NX,NX>;

  SREKF();

  void setInitial(const Vec& mu0, const Mat& S0);
  const Vec& mu() const { return mu_; }
  const Mat& S () const { return S_;  }

  // Predict using linearization at x_k (Euler discretization)
  void predict(const qcar_nav::KinematicModel& model, double t, double dt);

  // Generic square-root update (any measurement size)
  void update(const Eigen::VectorXd& z,
              const Eigen::VectorXd& h,
              const Eigen::MatrixXd& H,
              const Eigen::MatrixXd& sqrtR);

private:
  Vec mu_;
  Mat S_; // upper-triangular
};

} // namespace qcar_nav
