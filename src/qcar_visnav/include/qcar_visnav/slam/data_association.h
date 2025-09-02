// Enhanced data_association.h
#pragma once
#include <Eigen/Core>
#include <vector>
#include <limits>
#include "particle.h"
#include "landmark.h"
#include "measurement_model.h"

namespace qcar_visnav { namespace slam {

/** Enhanced result of data association with quality metrics */
struct AssocResult {
  int lm_index;                 // -1 => create a new landmark
  double d2;                    // Mahalanobis distance
  double log_likelihood;        // Log-likelihood of this association
  
  Eigen::Matrix2d S;            // innovation covariance
  Eigen::Matrix2d H;            // measurement Jacobian
  Eigen::Vector2d nu;           // innovation [dr, db]
  double r_hat{0.0};
  double b_hat{0.0};
  
  // Additional quality metrics
  double compatibility_score{0.0};
  bool is_ambiguous{false};
};

/** Global assignment result */
struct GlobalAssignment {
  std::vector<std::pair<int,int>> matches;  // (measurement_idx, landmark_idx) pairs
  std::vector<int> unmatched_measurements;
  std::vector<int> unmatched_landmarks; 
  double total_cost{0.0};
  double average_compatibility{0.0};
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
   * Global optimal assignment using Hungarian algorithm
   */
  GlobalAssignment associateGlobal(const Particle& p,
                                  const std::vector<MeasRB>& measurements,
                                  const LidarExtrinsics& ex,
                                  const Eigen::Matrix2d& base_R) const;

  /**
   * Fast greedy assignment (fallback for computational constraints)
   */
  GlobalAssignment associateGreedy(const Particle& p,
                                  const std::vector<MeasRB>& measurements,
                                  const LidarExtrinsics& ex,
                                  const Eigen::Matrix2d& base_R) const;

  /**
   * Joint Compatibility Branch and Bound (JCBB) for highest quality
   */
  GlobalAssignment associateJCBB(const Particle& p,
                                 const std::vector<MeasRB>& measurements,
                                 const LidarExtrinsics& ex,
                                 const Eigen::Matrix2d& base_R) const;

private:
  double chi2_gate_;
  double ambiguity_threshold_;  // Threshold for detecting ambiguous associations
  bool use_hungarian_;
  
  // Hungarian algorithm implementation
  std::vector<int> solveHungarian(const std::vector<std::vector<double>>& cost_matrix) const;
  
  // Compatibility test for JCBB
  bool isJointlyCompatible(const std::vector<std::pair<int,int>>& hypothesis,
                          const Particle& p,
                          const std::vector<MeasRB>& measurements,
                          const LidarExtrinsics& ex,
                          const Eigen::Matrix2d& base_R) const;
                          
  // Quality assessment
  double computeCompatibilityScore(const Eigen::Vector2d& innovation,
                                  const Eigen::Matrix2d& S) const;
};

}} // namespace