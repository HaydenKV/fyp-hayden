#include "qcar_visnav/slam/data_association.h"

using namespace qcar_visnav::slam;

AssocResult DataAssociation::associate(const Particle& p,
                                       const Eigen::Vector2d& z,
                                       const Eigen::Matrix2d& R) const {
  int best = -1;
  double best_d2 = 1e12;

  for (int j = 0; j < (int)p.map.size(); ++j) {
    const auto& lm = p.map[j];
    if (!lm.confirmed) continue; // only match confirmed landmarks

    Eigen::Vector2d nu = z - lm.mu;
    Eigen::Matrix2d S = lm.Sigma + R;
    double d2 = nu.transpose() * S.inverse() * nu;
    if (d2 < chi2_gate_ && d2 < best_d2) {
      best_d2 = d2;
      best = j;
    }
  }

  return {best, best_d2};
}

