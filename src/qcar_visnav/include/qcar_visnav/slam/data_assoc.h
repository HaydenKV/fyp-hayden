#pragma once
#include <Eigen/Dense>
#include <limits>
#include <utility>
#include <vector>

namespace qcar_visnav {
namespace slam {

// Returns (best_idx, best_d2); best_idx == -1 if none within gate
inline std::pair<int,double> nn_gated(
    const std::vector<Eigen::Vector2d>& means,
    const std::vector<Eigen::Matrix2d>& covs,
    const Eigen::Vector2d& z,
    const Eigen::Matrix2d& R,
    double chi2_gate)
{
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)means.size(); ++i) {
    Eigen::Matrix2d S = covs[i] + R;        // H=I
    Eigen::Vector2d v = z - means[i];
    Eigen::LLT<Eigen::Matrix2d> llt(S);
    if (llt.info() != Eigen::Success) continue;
    double d2 = v.dot( llt.solve(v) );
    if (std::isfinite(d2) && d2 < best_d2 && d2 <= chi2_gate) {
      best_d2 = d2; best = i;
    }
  }
  return {best, best_d2};
}

} // namespace slam
} // namespace qcar_visnav
