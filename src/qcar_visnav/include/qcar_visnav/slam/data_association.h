#pragma once
#include "particle.h"
#include "measurement_model.h"

namespace qcar_visnav { namespace slam {

struct AssocResult {
  int lm_index;     // -1 => new landmark
  double d2;
};

class DataAssociation {
public:
  explicit DataAssociation(double chi2_gate): chi2_gate_(chi2_gate) {}
  AssocResult associate(const Particle& p,
                        const Eigen::Vector2d& z, const Eigen::Matrix2d& R) const;
  // TODO: implement Mahalanobis gating
private:
  double chi2_gate_;
};

}} // ns
