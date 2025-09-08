// Enhanced data_association.h
#pragma once
#include <Eigen/Core>
#include <vector>
#include <limits>
#include <utility>
#include "particle.h"
#include "landmark.h"
#include "measurement_model.h"

namespace qcar_visnav { namespace slam {

/** Per-pair cached stats (optional, not required by current pipeline) */
struct AssocResult {
  int lm_index{-1};             // -1 => new landmark / unmatched
  double d2{std::numeric_limits<double>::infinity()}; // Mahalanobis distance
  double log_likelihood{0.0};   // Log-likelihood for this pair

  Eigen::Matrix2d S{Eigen::Matrix2d::Identity()};
  Eigen::Matrix2d H{Eigen::Matrix2d::Zero()};
  Eigen::Vector2d nu{Eigen::Vector2d::Zero()};
  double r_hat{0.0};
  double b_hat{0.0};

  // Quality metrics
  double compatibility_score{0.0};  // exp(-0.5 * d2)
  bool is_ambiguous{false};
};

/** Global assignment result */
struct GlobalAssignment {
  std::vector<std::pair<int,int>> matches;  // (measurement_idx, landmark_idx) pairs
  std::vector<int> unmatched_measurements;  // measurement indices with no match
  std::vector<int> unmatched_landmarks;     // landmark indices not matched
  double total_cost{0.0};
  double average_compatibility{0.0};        // mean of exp(-0.5 d2) over chosen matches
};

class DataAssociation {
public:
  explicit DataAssociation(double chi2_gate = 9.21,
                           double ambiguity_threshold = 0.8,
                           bool use_hungarian = true)
    : chi2_gate_(chi2_gate),
      ambiguity_threshold_(ambiguity_threshold),
      use_hungarian_(use_hungarian) {}

  /**
   * Global optimal assignment using Hungarian algorithm (one-to-one).
   * Includes dummy "unassigned" columns with cost = chi2_gate_ so a meas
   * can remain unmatched if all real matches are worse than the gate.
   */
  GlobalAssignment associateGlobal(const Particle& p,
                                  const std::vector<MeasRB>& measurements,
                                  const LidarExtrinsics& ex,
                                  const Eigen::Matrix2d& /*base_R, unused now */) const;

  /**
   * Fast greedy assignment (sorted by d², exclusive one-to-one).
   */
  GlobalAssignment associateGreedy(const Particle& p,
                                   const std::vector<MeasRB>& measurements,
                                   const LidarExtrinsics& ex,
                                   const Eigen::Matrix2d& /*base_R, unused now */) const;

  /**
   * Joint Compatibility Branch and Bound (JCBB).
   */
  GlobalAssignment associateJCBB(const Particle& p,
                                 const std::vector<MeasRB>& measurements,
                                 const LidarExtrinsics& ex,
                                 const Eigen::Matrix2d& /*base_R, unused now */) const;

private:
  double chi2_gate_;
  double ambiguity_threshold_;
  bool   use_hungarian_;   // kept for compatibility; selection is done by caller

  // Hungarian (Munkres) solver for rectangular matrices (rows <= cols).
  // Returns a column index for each row (size = n_rows). -1 means unassigned.
  std::vector<int> solveHungarian(const std::vector<std::vector<double>>& cost) const;

  // JCBB joint-compatibility test for a hypothesis
  bool isJointlyCompatible(const std::vector<std::pair<int,int>>& hypothesis,
                           const Particle& p,
                           const std::vector<MeasRB>& measurements,
                           const LidarExtrinsics& ex,
                           const Eigen::Matrix2d& /*base_R, unused now */) const;

  // Quality assessment utility
  double computeCompatibilityScore(const Eigen::Vector2d& innovation,
                                   const Eigen::Matrix2d& S) const;
};

}} // namespace
