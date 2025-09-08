// Enhanced data_association.cpp
#include "qcar_visnav/slam/data_association.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <ros/ros.h>

namespace qcar_visnav { namespace slam {

// ---------- utilities ----------
static inline double clamp_min(double v, double minv) {
  return (v < minv) ? minv : v;
}

// ---------- Hungarian (Munkres) implementation ----------
std::vector<int> DataAssociation::solveHungarian(const std::vector<std::vector<double>>& cost) const {
  // Assumes rows <= cols (we construct it that way).
  const int n_rows = static_cast<int>(cost.size());
  const int n_cols = n_rows ? static_cast<int>(cost[0].size()) : 0;
  if (n_rows == 0 || n_cols == 0) return {};

  // Copy into mutable matrix
  std::vector<std::vector<double>> C = cost;

  // Step 1: Row reduction
  for (int i = 0; i < n_rows; ++i) {
    double rmin = *std::min_element(C[i].begin(), C[i].end());
    for (int j = 0; j < n_cols; ++j) C[i][j] -= rmin;
  }
  // Step 2: Column reduction
  for (int j = 0; j < n_cols; ++j) {
    double cmin = C[0][j];
    for (int i = 1; i < n_rows; ++i) cmin = std::min(cmin, C[i][j]);
    for (int i = 0; i < n_rows; ++i) C[i][j] -= cmin;
  }

  // Starred/primed zeros bookkeeping
  std::vector<int> star_row_of_col(n_cols, -1);  // for each col, which row has a star
  std::vector<int> star_col_of_row(n_rows, -1);  // for each row, which col is starred
  std::vector<int> prime_col_of_row(n_rows, -1); // for each row, which col is primed

  // Step 3: Star zeros (one per row/col greedily)
  std::vector<bool> row_covered(n_rows, false), col_covered(n_cols, false);
  for (int i = 0; i < n_rows; ++i) {
    for (int j = 0; j < n_cols; ++j) {
      if (C[i][j] == 0 && !row_covered[i] && !col_covered[j]) {
        star_col_of_row[i] = j;
        star_row_of_col[j] = i;
        row_covered[i] = true;
        col_covered[j] = true;
      }
    }
  }
  std::fill(row_covered.begin(), row_covered.end(), false);
  std::fill(col_covered.begin(), col_covered.end(), false);

  auto cover_starred_columns = [&]() {
    int count = 0;
    std::fill(col_covered.begin(), col_covered.end(), false);
    for (int j = 0; j < n_cols; ++j) {
      if (star_row_of_col[j] != -1) { col_covered[j] = true; ++count; }
    }
    return count;
  };

  // Step 4: cover columns with starred zeros
  int covered_cols = cover_starred_columns();
  while (covered_cols < n_rows) {
    // Step 5: Find an uncovered zero, prime it
    int z_row = -1, z_col = -1;
    while (true) {
      bool found = false;
      for (int i = 0; i < n_rows && !found; ++i) {
        if (row_covered[i]) continue;
        for (int j = 0; j < n_cols; ++j) {
          if (col_covered[j]) continue;
          if (C[i][j] == 0) { z_row = i; z_col = j; found = true; break; }
        }
      }
      if (!found) {
        // Step 6: Adjust the matrix (no uncovered zero found)
        double minval = std::numeric_limits<double>::infinity();
        for (int i = 0; i < n_rows; ++i) {
          if (row_covered[i]) continue;
          for (int j = 0; j < n_cols; ++j) {
            if (!col_covered[j]) minval = std::min(minval, C[i][j]);
          }
        }
        if (!std::isfinite(minval)) break; // degenerate
        for (int i = 0; i < n_rows; ++i) {
          for (int j = 0; j < n_cols; ++j) {
            if (row_covered[i]) C[i][j] += minval;
            if (!col_covered[j]) C[i][j] -= minval;
          }
        }
        continue; // look again for uncovered zero
      }

      prime_col_of_row[z_row] = z_col;

      if (star_col_of_row[z_row] == -1) {
        // Step 5c: augmenting path starting from (z_row, z_col)
        // Build alternating sequence of primed and starred zeros
        std::vector<std::pair<int,int>> path;
        path.emplace_back(z_row, z_col);

        // Find sequence: (row, col) where rows alternate between those with
        // starred zeros in the column and primed zeros in the row.
        while (true) {
          int r = -1;
          // find starred zero in column 'col' (if any)
          int last_col = path.back().second;
          r = star_row_of_col[last_col];
          if (r == -1) break;
          path.emplace_back(r, last_col);

          // find primed zero in row 'r' (guaranteed to exist)
          int c = prime_col_of_row[r];
          path.emplace_back(r, c);
        }

        // Flip stars along the path
        for (const auto& rc : path) {
          int i = rc.first, j = rc.second;
          if (star_col_of_row[i] == j) {
            // was starred -> unstar
            star_col_of_row[i] = -1;
            star_row_of_col[j] = -1;
          } else if (prime_col_of_row[i] == j) {
            // was primed -> star it
            star_col_of_row[i] = j;
            star_row_of_col[j] = i;
          }
        }

        // Clear covers and primes
        std::fill(row_covered.begin(), row_covered.end(), false);
        std::fill(col_covered.begin(), col_covered.end(), false);
        std::fill(prime_col_of_row.begin(), prime_col_of_row.end(), -1);

        // Cover columns with stars again
        covered_cols = cover_starred_columns();
        break; // restart loop condition
      } else {
        // Step 5b: cover this row and uncover the column of the star
        int jstar = star_col_of_row[z_row];
        row_covered[z_row] = true;
        col_covered[jstar] = false;
      }
    }
  }

  // Build assignment: for each row, assigned column (or -1 if none)
  std::vector<int> assignment(n_rows, -1);
  for (int i = 0; i < n_rows; ++i) {
    assignment[i] = star_col_of_row[i];
  }
  return assignment;
}

// ---------- associateGlobal (Hungarian) ----------
GlobalAssignment DataAssociation::associateGlobal(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& /*base_R*/) const
{
  const int M = static_cast<int>(measurements.size());
  const int L = static_cast<int>(p.map.size());

  GlobalAssignment result;
  if (M == 0) { // trivial
    // all landmarks unmatched
    result.unmatched_landmarks.resize(L);
    std::iota(result.unmatched_landmarks.begin(), result.unmatched_landmarks.end(), 0);
    result.total_cost = 0.0;
    result.average_compatibility = 0.0;
    return result;
  }

  // Build cost matrix (rows = measurements, cols = landmarks + M dummy for "unmatched")
  const int Ccols = L + M;
  std::vector<std::vector<double>> cost(M, std::vector<double>(Ccols, chi2_gate_));
  std::vector<std::vector<double>> compat(M, std::vector<double>(L, 0.0)); // for stats

  const double BIG = 1e6;

  for (int m = 0; m < M; ++m) {
    const auto& meas = measurements[m];

    // Per-measurement covariance (direct)
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = clamp_min(meas.r_var, 1e-10);
    R(1,1) = clamp_min(meas.b_var, 1e-12);

    for (int l = 0; l < L; ++l) {
      const auto& landmark = p.map[l];

      double r_hat=0, b_hat=0;
      Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);

      Eigen::Vector2d innovation;
      innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);

      Eigen::Matrix2d S = H * landmark.Sigma * H.transpose() + R;

      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) {
        cost[m][l] = BIG; // ill-conditioned -> disallow
        continue;
      }

      const Eigen::Matrix2d S_inv = llt.solve(Eigen::Matrix2d::Identity());
      const double d2 = (innovation.transpose() * S_inv * innovation)(0,0);

      if (d2 <= chi2_gate_) {
        cost[m][l] = d2;
        compat[m][l] = std::exp(-0.5 * d2);
      } else {
        cost[m][l] = BIG; // outside gate
      }
    }

    // Dummy column for "unmatched m": index = L + m, cost = chi2_gate_
    cost[m][L + m] = chi2_gate_;
  }

  // Solve linear assignment
  std::vector<int> assign = solveHungarian(cost); // size M, each is column index in [0, Ccols)

  // Decode assignment
  double sum_compat = 0.0;
  int    compat_cnt = 0;
  std::vector<bool> lm_used(L, false);

  for (int m = 0; m < M; ++m) {
    int c = (m < (int)assign.size()) ? assign[m] : -1;
    if (c >= 0 && c < L) {
      result.matches.emplace_back(m, c);
      result.total_cost += cost[m][c];
      lm_used[c] = true;
      sum_compat += compat[m][c];
      ++compat_cnt;
    } else {
      result.unmatched_measurements.push_back(m);
      if (c >= L && c < L + M) result.total_cost += cost[m][c];
    }
  }

  // Fill unmatched landmarks
  for (int l = 0; l < L; ++l) if (!lm_used[l]) result.unmatched_landmarks.push_back(l);

  result.average_compatibility = (compat_cnt > 0) ? (sum_compat / compat_cnt) : 0.0;
  return result;
}

// ---------- associateJCBB ----------
GlobalAssignment DataAssociation::associateJCBB(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& /*base_R*/) const
{
  const int M = (int)measurements.size();
  const int L = (int)p.map.size();

  GlobalAssignment best_assignment;
  best_assignment.total_cost = std::numeric_limits<double>::infinity();

  // Precompute compatibility and costs
  std::vector<std::vector<bool>> compatible(M, std::vector<bool>(L, false));
  std::vector<std::vector<double>> costs(M, std::vector<double>(L, 0.0));
  std::vector<std::vector<double>> compat_score(M, std::vector<double>(L, 0.0));

  for (int m = 0; m < M; ++m) {
    const auto& meas = measurements[m];
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = clamp_min(meas.r_var, 1e-10);
    R(1,1) = clamp_min(meas.b_var, 1e-12);

    for (int l = 0; l < L; ++l) {
      const auto& landmark = p.map[l];

      double r_hat=0, b_hat=0; Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);

      Eigen::Vector2d innovation;
      innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);

      Eigen::Matrix2d S = H * landmark.Sigma * H.transpose() + R;

      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;

      const double d2 = (innovation.transpose() * llt.solve(innovation))(0,0);

      if (d2 <= chi2_gate_) {
        compatible[m][l] = true;
        costs[m][l] = d2;
        compat_score[m][l] = std::exp(-0.5 * d2);
      }
    }
  }

  // Branch & bound JCBB
  std::function<void(int, std::vector<std::pair<int,int>>&, double,
                     std::vector<bool>&, double, int)> dfs;
  dfs = [&](int depth,
            std::vector<std::pair<int,int>>& hyp,
            double cur_cost,
            std::vector<bool>& used_lm,
            double cur_compat_sum,
            int cur_compat_cnt)
  {
    if (cur_cost >= best_assignment.total_cost) return;
    if (depth == M) {
      if (cur_cost < best_assignment.total_cost) {
        best_assignment.total_cost = cur_cost;
        best_assignment.matches = hyp;
        best_assignment.average_compatibility =
          (cur_compat_cnt > 0) ? (cur_compat_sum / cur_compat_cnt) : 0.0;

        // fill unmatched later
      }
      return;
    }

    // Option 1: leave measurement 'depth' unmatched (penalize with chi2_gate_)
    dfs(depth + 1, hyp, cur_cost + chi2_gate_, used_lm, cur_compat_sum, cur_compat_cnt);

    // Option 2: try all compatible, unused landmarks
    for (int l = 0; l < L; ++l) {
      if (!compatible[depth][l] || used_lm[l]) continue;

      // Temporarily add match
      hyp.emplace_back(depth, l);
      used_lm[l] = true;

      // Joint compatibility test
      if (isJointlyCompatible(hyp, p, measurements, ex, Eigen::Matrix2d::Identity())) {
        dfs(depth + 1, hyp, cur_cost + costs[depth][l], used_lm,
            cur_compat_sum + compat_score[depth][l], cur_compat_cnt + 1);
      }

      // Backtrack
      used_lm[l] = false;
      hyp.pop_back();
    }
  };

  std::vector<std::pair<int,int>> hyp0;
  std::vector<bool> used_lm(L, false);
  dfs(0, hyp0, 0.0, used_lm, 0.0, 0);

  // Build unmatched sets
  std::vector<bool> meas_used(M, false), lm_used(L, false);
  for (const auto& pr : best_assignment.matches) {
    meas_used[pr.first] = true;
    lm_used[pr.second] = true;
  }
  for (int m = 0; m < M; ++m) if (!meas_used[m]) best_assignment.unmatched_measurements.push_back(m);
  for (int l = 0; l < L; ++l) if (!lm_used[l]) best_assignment.unmatched_landmarks.push_back(l);

  if (!std::isfinite(best_assignment.total_cost)) {
    // fall back to all unmatched if JCBB found nothing
    best_assignment.total_cost = chi2_gate_ * M;
    best_assignment.matches.clear();
    best_assignment.unmatched_measurements.clear();
    for (int m = 0; m < M; ++m) best_assignment.unmatched_measurements.push_back(m);
    best_assignment.unmatched_landmarks.resize(L);
    std::iota(best_assignment.unmatched_landmarks.begin(), best_assignment.unmatched_landmarks.end(), 0);
    best_assignment.average_compatibility = 0.0;
  }

  return best_assignment;
}

// ---------- isJointlyCompatible ----------
bool DataAssociation::isJointlyCompatible(
    const std::vector<std::pair<int,int>>& hypothesis,
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& /*base_R*/) const
{
  if (hypothesis.size() <= 1) return true;

  const int n = static_cast<int>(hypothesis.size());
  Eigen::VectorXd joint_innov(2 * n);
  Eigen::MatrixXd joint_S(2 * n, 2 * n);
  joint_S.setZero();

  for (int i = 0; i < n; ++i) {
    const int m_idx = hypothesis[i].first;
    const int l_idx = hypothesis[i].second;

    const auto& meas = measurements[m_idx];
    const auto& landmark = p.map[l_idx];

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = clamp_min(meas.r_var, 1e-10);
    R(1,1) = clamp_min(meas.b_var, 1e-12);

    double r_hat=0, b_hat=0; Eigen::Matrix2d H;
    predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);

    Eigen::Vector2d innovation;
    innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);

    joint_innov.segment<2>(2 * i) = innovation;
    joint_S.block<2,2>(2 * i, 2 * i) = H * landmark.Sigma * H.transpose() + R;
  }

  Eigen::LLT<Eigen::MatrixXd> llt(joint_S);
  if (llt.info() != Eigen::Success) return false;

  const double d2 = (joint_innov.transpose() * llt.solve(joint_innov))(0,0);
  const double joint_gate = chi2_gate_ * n;  // simple linear scaling with #meas
  return (d2 <= joint_gate);
}

// ---------- associateGreedy ----------
GlobalAssignment DataAssociation::associateGreedy(
    const Particle& p,
    const std::vector<MeasRB>& measurements,
    const LidarExtrinsics& ex,
    const Eigen::Matrix2d& /*base_R*/) const
{
  GlobalAssignment result;

  struct Cand {
    int m_idx{-1}, l_idx{-1};
    double cost{0.0};
    double compat{0.0};
  };
  std::vector<Cand> cands;
  cands.reserve(measurements.size() * std::max<size_t>(1, p.map.size()));

  for (int m = 0; m < (int)measurements.size(); ++m) {
    const auto& meas = measurements[m];
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = clamp_min(meas.r_var, 1e-10);
    R(1,1) = clamp_min(meas.b_var, 1e-12);

    for (int l = 0; l < (int)p.map.size(); ++l) {
      const auto& landmark = p.map[l];

      double r_hat=0, b_hat=0; Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, landmark.mu, ex, r_hat, b_hat, H);

      Eigen::Vector2d innovation;
      innovation << (meas.r - r_hat), wrapToPi(meas.b - b_hat);

      Eigen::Matrix2d S = H * landmark.Sigma * H.transpose() + R;
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;

      const double d2 = (innovation.transpose() * llt.solve(innovation))(0,0);
      if (d2 <= chi2_gate_) {
        Cand c;
        c.m_idx = m; c.l_idx = l;
        c.cost = d2;
        c.compat = std::exp(-0.5 * d2);
        cands.push_back(c);
      }
    }
  }

  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b){ return a.cost < b.cost; });

  std::vector<bool> used_m(measurements.size(), false);
  std::vector<bool> used_l(p.map.size(), false);

  double sum_compat = 0.0; int cnt = 0;

  for (const auto& c : cands) {
    if (used_m[c.m_idx] || used_l[c.l_idx]) continue;
    result.matches.emplace_back(c.m_idx, c.l_idx);
    result.total_cost += c.cost;
    used_m[c.m_idx] = true; used_l[c.l_idx] = true;
    sum_compat += c.compat; ++cnt;
  }

  for (int m = 0; m < (int)measurements.size(); ++m)
    if (!used_m[m]) { result.unmatched_measurements.push_back(m); result.total_cost += chi2_gate_; }
  for (int l = 0; l < (int)p.map.size(); ++l)
    if (!used_l[l]) result.unmatched_landmarks.push_back(l);

  result.average_compatibility = (cnt > 0) ? (sum_compat / cnt) : 0.0;
  return result;
}

// ---------- quality score ----------
double DataAssociation::computeCompatibilityScore(
    const Eigen::Vector2d& innovation,
    const Eigen::Matrix2d& S) const
{
  Eigen::LLT<Eigen::Matrix2d> llt(S);
  if (llt.info() != Eigen::Success) return 0.0;
  const double d2 = (innovation.transpose() * llt.solve(innovation))(0,0);
  return std::exp(-0.5 * d2);
}

}} // namespace
