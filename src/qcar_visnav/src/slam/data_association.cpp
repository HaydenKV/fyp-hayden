#include "qcar_visnav/slam/data_association.h"
#include <limits>

namespace qcar_visnav { namespace slam {

AssocResult DataAssociation::associate(const Particle& p,
                                       const MeasRB& z,
                                       const LidarExtrinsics& ex,
                                       const Eigen::Matrix2d& R) const
{
  AssocResult best;
  best.lm_index = -1;
  best.d2 = std::numeric_limits<double>::infinity();

  for (int i = 0; i < (int)p.map.size(); ++i) {
    const auto& lm = p.map[i];

    double r_hat, b_hat;
    Eigen::Matrix2d H;
    predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_hat, b_hat, H);

    Eigen::Vector2d zhat(r_hat, b_hat);
    Eigen::Vector2d zobs(z.r, z.b);
    Eigen::Vector2d nu = zobs - zhat;
    nu(1) = wrapToPi(nu(1));

    Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + R;
    Eigen::LLT<Eigen::Matrix2d> llt(S);
    if (llt.info() != Eigen::Success) continue;

    double d2 = nu.transpose() * llt.solve(nu);

    if (d2 < chi2_gate_ && d2 < best.d2) {
      best.lm_index = i;
      best.d2 = d2;
      best.S = S;
      best.H = H;
      best.nu = nu;
      best.r_hat = r_hat;
      best.b_hat = b_hat;
    }
  }

  return best;
}

}} // ns
