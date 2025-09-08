#include "qcar_visnav/slam/data_association.h"
#include <algorithm>
#include <limits>
#include <unordered_map>

namespace qcar_visnav { namespace slam {

static inline double clamp_min(double v, double m) { return (v < m) ? m : v; }

DataAssociation::Result
DataAssociation::associateById(const Particle& p,
                               const std::vector<MeasRB>& meas_vec) const
{
  Result R;
  // Build landmark index by ID
  std::unordered_map<int,int> lm_by_id;
  lm_by_id.reserve(p.map.size());
  for (int i = 0; i < (int)p.map.size(); ++i) {
    if (p.map[i].id >= 0) lm_by_id[p.map[i].id] = i;
  }

  std::vector<bool> lm_used(p.map.size(), false);
  for (int m = 0; m < (int)meas_vec.size(); ++m) {
    const auto& z = meas_vec[m];
    if (!z.has_id) { R.unmatched_meas.push_back(m); continue; }

    auto it = lm_by_id.find(z.id);
    if (it != lm_by_id.end() && !lm_used[it->second]) {
      R.matches.push_back({m, it->second});
      lm_used[it->second] = true;
    } else {
      R.unmatched_meas.push_back(m);
    }
  }
  return R;
}

DataAssociation::Result
DataAssociation::associateNNRB(const Particle& p,
                               const std::vector<MeasRB>& meas_vec,
                               const LidarExtrinsics& ex) const
{
  struct Cand { int m{-1}; int l{-1}; double d2{0.0}; };
  std::vector<Cand> cands; cands.reserve(meas_vec.size() * std::max<size_t>(1, p.map.size()));

  // Build candidates within gate
  for (int m = 0; m < (int)meas_vec.size(); ++m) {
    const auto& z = meas_vec[m];
    Eigen::Matrix2d Rm = Eigen::Matrix2d::Zero();
    Rm(0,0) = clamp_min(z.r_var, 1e-10);
    Rm(1,1) = clamp_min(z.b_var, 1e-12);

    for (int l = 0; l < (int)p.map.size(); ++l) {
      const auto& lm = p.map[l];
      double r_hat = 0.0, b_hat = 0.0;
      Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_hat, b_hat, H);

      Eigen::Vector2d nu;
      nu << (z.r - r_hat), wrapToPi(z.b - b_hat);

      const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Rm;
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;

      const double d2 = (nu.transpose() * llt.solve(nu))(0,0);
      if (d2 <= chi2_gate_rb_) cands.push_back({m, l, d2});
    }
  }

  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b){ return a.d2 < b.d2; });

  Result R;
  std::vector<bool> used_m(meas_vec.size(), false);
  std::vector<bool> used_l(p.map.size(), false);

  for (const auto& c : cands) {
    if (used_m[c.m] || used_l[c.l]) continue;
    used_m[c.m] = true; used_l[c.l] = true;
    R.matches.push_back({c.m, c.l});
  }

  for (int m = 0; m < (int)meas_vec.size(); ++m)
    if (!used_m[m]) R.unmatched_meas.push_back(m);

  return R;
}

}} // namespace
