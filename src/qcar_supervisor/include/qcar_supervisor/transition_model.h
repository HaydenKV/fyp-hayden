#ifndef QCAR_VISNAV_ESTIMATION_JMF_TRANSITION_MODEL_H
#define QCAR_VISNAV_ESTIMATION_JMF_TRANSITION_MODEL_H

#include <Eigen/Dense>

namespace qcar_nav {

/**
 * @brief Handles time-varying Markov transition matrices for sensor fault modes
 */
class TransitionModel {
public:
  /**
   * @brief Set reference transition matrix and reference time step
   * @param T_ref Reference transition matrix (e.g. at dt = 1.0 s)
   * @param dt_ref Reference time step (seconds)
   */
  void setReferenceMatrix(const Eigen::Matrix2d& T_ref, double dt_ref);

  /**
   * @brief Get transition matrix for arbitrary time step
   * @param dt Time step (seconds)
   * @return Transition matrix T(dt)
   */
  Eigen::Matrix2d getTransition(double dt) const;

private:
  Eigen::Matrix2d T_ref_;  // Reference transition matrix
  double dt_ref_;          // Reference time step
};

} // namespace qcar_nav

#endif // QCAR_VISNAV_ESTIMATION_JMF_TRANSITION_MODEL_H
