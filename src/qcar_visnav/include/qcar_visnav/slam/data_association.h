#pragma once
#include <vector>
#include "qcar_visnav/slam/particle.h"
#include "qcar_visnav/slam/measurement_model.h"

namespace qcar_visnav { namespace slam {

struct AssocMatch {
  int meas_idx{-1};
  int lm_idx{-1};
};

class DataAssociation {
public:
  explicit DataAssociation(double chi2_gate_rb) : chi2_gate_rb_(chi2_gate_rb) {}

  struct Result {
    std::vector<AssocMatch> matches;   // pairs (measurement -> landmark)
    std::vector<int>        unmatched_meas;
  };

  // Mode A: by tracker IDs (exact match).
  Result associateById(const Particle& p,
                       const std::vector<MeasRB>& meas_vec) const;

  // Mode B: nearest-neighbor in RB space using Mahalanobis distance with χ² gate.
  Result associateNNRB(const Particle& p,
                       const std::vector<MeasRB>& meas_vec,
                       const LidarExtrinsics& ex) const;

private:
  double chi2_gate_rb_{5.99};
};

}} // namespace
