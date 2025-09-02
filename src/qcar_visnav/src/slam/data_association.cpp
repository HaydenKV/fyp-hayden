// Enhanced data_association.cpp
#include "qcar_visnav/slam/data_association.h"
#include <algorithm>
#include <cmath>
#include <ros/ros.h>

namespace qcar_visnav { namespace slam {

GlobalAssignment DataAssociation::associateGlobal(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& base_R) const
{
  if (use_hungarian_) {
    return associateJCBB(p, measurements, ex, base_R);  // Use JCBB for best quality
  } else {
    return associateGreedy(p, measurements, ex, base_R);
  }
}

GlobalAssignment DataAssociation::associateJCBB(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& base_R) const
{
  const int M = measurements.size();
  const int L = p.map.size();
  
  GlobalAssignment best_assignment;
  best_assignment.total_cost = std::numeric_limits<double>::infinity();
  
  // Build compatibility matrix first
  std::vector<std::vector<bool>> compatible(M, std::vector<bool>(L, false));
  std::vector<std::vector<double>> costs(M, std::vector<double>(L, 0.0));
  std::vector<std::vector<AssocResult>> assoc_cache(M, std::vector<AssocResult>(L));
  
  for (int m = 0; m < M; ++m) {
    const auto& meas = measurements[m];
    Eigen::Matrix2d R = base_R;
    R(0,0) *= meas.r_var / base_R(0,0);
    R(1,1) *= meas.b_var / base_R(1,1);
    
    for (int l = 0; l < L; ++l) {
      const auto& landmark = p.map[l];
      
      double r_hat, b_hat;
      Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);
      
      Eigen::Vector2d innovation;
      innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);
      
      Eigen::Matrix2d S = H * landmark.Sigma * H.transpose() + R;
      
      // Compatibility test
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;
      
      double d2 = innovation.transpose() * llt.solve(innovation);
      
      if (d2 <= chi2_gate_) {
        compatible[m][l] = true;
        costs[m][l] = d2;
        
        // Cache association result
        assoc_cache[m][l].lm_index = l;
        assoc_cache[m][l].d2 = d2;
        assoc_cache[m][l].S = S;
        assoc_cache[m][l].H = H;
        assoc_cache[m][l].nu = innovation;
        assoc_cache[m][l].r_hat = r_hat;
        assoc_cache[m][l].b_hat = b_hat;
        assoc_cache[m][l].compatibility_score = computeCompatibilityScore(innovation, S);
        assoc_cache[m][l].log_likelihood = -0.5 * (d2 + std::log(S.determinant()) + 2.0 * std::log(2.0 * M_PI));
      }
    }
  }
  
  // JCBB branch and bound search
  std::function<void(int, std::vector<std::pair<int,int>>&, double)> 
    branch_and_bound = [&](int depth, std::vector<std::pair<int,int>>& current_hyp, double current_cost) {
    
    if (current_cost >= best_assignment.total_cost) return;  // Bound
    
    if (depth == M) {
      // Complete hypothesis found
      if (current_cost < best_assignment.total_cost) {
        best_assignment.total_cost = current_cost;
        best_assignment.matches = current_hyp;
        
        // Compute average compatibility
        double total_compat = 0.0;
        for (const auto& match : current_hyp) {
          total_compat += assoc_cache[match.first][match.second].compatibility_score;
        }
        best_assignment.average_compatibility = total_compat / std::max(1.0, (double)current_hyp.size());
      }
      return;
    }
    
    // Try measurement 'depth' unassociated
    branch_and_bound(depth + 1, current_hyp, current_cost + chi2_gate_);  // Penalty for new landmark
    
    // Try associating measurement 'depth' with each compatible landmark
    std::vector<bool> used_landmarks(L, false);
    for (const auto& match : current_hyp) {
      used_landmarks[match.second] = true;
    }
    
    for (int l = 0; l < L; ++l) {
      if (!compatible[depth][l] || used_landmarks[l]) continue;
      
      current_hyp.push_back({depth, l});
      
      // Joint compatibility test
      if (isJointlyCompatible(current_hyp, p, measurements, ex, base_R)) {
        branch_and_bound(depth + 1, current_hyp, current_cost + costs[depth][l]);
      }
      
      current_hyp.pop_back();
    }
  };
  
  std::vector<std::pair<int,int>> initial_hyp;
  branch_and_bound(0, initial_hyp, 0.0);
  
  // Fill unmatched measurements
  std::vector<bool> matched_measurements(M, false);
  for (const auto& match : best_assignment.matches) {
    matched_measurements[match.first] = true;
  }
  
  for (int m = 0; m < M; ++m) {
    if (!matched_measurements[m]) {
      best_assignment.unmatched_measurements.push_back(m);
    }
  }
  
  return best_assignment;
}

bool DataAssociation::isJointlyCompatible(
    const std::vector<std::pair<int,int>>& hypothesis,
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& base_R) const
{
  if (hypothesis.size() <= 1) return true;  // Single associations always compatible
  
  // Build joint innovation vector and covariance
  const int n = hypothesis.size();
  Eigen::VectorXd joint_innovation(2 * n);
  Eigen::MatrixXd joint_S(2 * n, 2 * n);
  joint_S.setZero();
  
  for (int i = 0; i < n; ++i) {
    const int m_idx = hypothesis[i].first;
    const int l_idx = hypothesis[i].second;
    
    const auto& meas = measurements[m_idx];
    const auto& landmark = p.map[l_idx];
    
    Eigen::Matrix2d R = base_R;
    R(0,0) *= meas.r_var / base_R(0,0);
    R(1,1) *= meas.b_var / base_R(1,1);
    
    double r_hat, b_hat;
    Eigen::Matrix2d H;
    predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);
    
    Eigen::Vector2d innovation;
    innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);
    
    joint_innovation.segment<2>(2 * i) = innovation;
    joint_S.block<2,2>(2 * i, 2 * i) = H * landmark.Sigma * H.transpose() + R;
  }
  
  // Chi-squared test for joint compatibility
  Eigen::LLT<Eigen::MatrixXd> llt(joint_S);
  if (llt.info() != Eigen::Success) return false;
  
  double joint_d2 = joint_innovation.transpose() * llt.solve(joint_innovation);
  double joint_chi2_gate = chi2_gate_ * n;  // Scale gate with number of measurements
  
  return joint_d2 <= joint_chi2_gate;
}

GlobalAssignment DataAssociation::associateGreedy(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& base_R) const
{
  // Fallback to original greedy implementation for speed
  GlobalAssignment result;
  
  struct Candidate {
    int m_idx, l_idx;
    double cost;
    AssocResult assoc;
  };
  
  std::vector<Candidate> candidates;
  
  // Build all valid associations
  for (int m = 0; m < (int)measurements.size(); ++m) {
    const auto& meas = measurements[m];
    
    for (int l = 0; l < (int)p.map.size(); ++l) {
      const auto& landmark = p.map[l];
      
      double r_hat, b_hat;
      Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);
      
      Eigen::Vector2d innovation;
      innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);
      
      Eigen::Matrix2d R = base_R;
      R(0,0) *= meas.r_var / base_R(0,0);
      R(1,1) *= meas.b_var / base_R(1,1);
      
      Eigen::Matrix2d S = H * landmark.Sigma * H.transpose() + R;
      
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;
      
      double d2 = innovation.transpose() * llt.solve(innovation);
      
      if (d2 <= chi2_gate_) {
        Candidate cand;
        cand.m_idx = m;
        cand.l_idx = l;
        cand.cost = d2;
        
        cand.assoc.lm_index = l;
        cand.assoc.d2 = d2;
        cand.assoc.S = S;
        cand.assoc.H = H;
        cand.assoc.nu = innovation;
        cand.assoc.r_hat = r_hat;
        cand.assoc.b_hat = b_hat;
        
        candidates.push_back(cand);
      }
    }
  }
  
  // Greedy selection
  std::sort(candidates.begin(), candidates.end(),
           [](const Candidate& a, const Candidate& b) { return a.cost < b.cost; });
  
  std::vector<bool> used_measurements(measurements.size(), false);
  std::vector<bool> used_landmarks(p.map.size(), false);
  
  for (const auto& cand : candidates) {
    if (used_measurements[cand.m_idx] || used_landmarks[cand.l_idx]) continue;
    
    result.matches.emplace_back(cand.m_idx, cand.l_idx);
    result.total_cost += cand.cost;
    used_measurements[cand.m_idx] = true;
    used_landmarks[cand.l_idx] = true;
  }
  
  // Collect unmatched measurements
  for (int m = 0; m < (int)measurements.size(); ++m) {
    if (!used_measurements[m]) {
      result.unmatched_measurements.push_back(m);
    }
  }
  
  return result;
}

double DataAssociation::computeCompatibilityScore(
    const Eigen::Vector2d& innovation,
    const Eigen::Matrix2d& S) const
{
  // Compute normalized compatibility score (higher = better match)
  Eigen::LLT<Eigen::Matrix2d> llt(S);
  if (llt.info() != Eigen::Success) return 0.0;
  
  double d2 = innovation.transpose() * llt.solve(innovation);
  return std::exp(-0.5 * d2);  // Convert Mahalanobis distance to probability-like score
}

}} // namespace