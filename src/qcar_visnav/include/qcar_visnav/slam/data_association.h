#pragma once
#include <Eigen/Core>
#include <vector>
#include "particle.h"
#include "landmark.h"
#include "measurement_model.h"

namespace qcar_visnav { namespace slam {

/** Result of trying to associate one measurement to the particle map. */
struct AssocResult {
  int lm_index;                 // -1 => create a new landmark
  double d2;                    // Mahalanobis distance

  Eigen::Matrix2d S;            // innovation covariance
  Eigen::Matrix2d H;            // measurement Jacobian
  Eigen::Vector2d nu;           // innovation [dr, db]
  double r_hat{0.0};
  double b_hat{0.0};
};

class DataAssociation {
public:
  explicit DataAssociation(double chi2_gate, double euclid_gate = 0.3)
    : chi2_gate_(chi2_gate), euclid_gate_(euclid_gate) {}

  /**
   * Associate one RB measurement with the particle's map.
   *
   * Inputs:
   *  - p: particle (pose and map)
   *  - z: measurement (r, b, variances) in LiDAR frame
   *  - ex: LiDAR extrinsics
   *  - R: measurement covariance (2x2, diag[r_var, b_var])
   */
  AssocResult associate(const Particle& p,
                        const MeasRB& z,
                        const LidarExtrinsics& ex,
                        const Eigen::Matrix2d& R) const;

private:
  double chi2_gate_;
  double euclid_gate_;
};

}} // ns
