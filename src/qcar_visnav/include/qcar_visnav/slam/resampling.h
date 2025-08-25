#pragma once
#include <vector>
#include "particle.h"

namespace qcar_visnav { namespace slam {

/**
 * Systematic resampling for a discrete distribution represented by particles with weights.
 * Input:  P[i].weight must be non-negative and sum to ~1.
 * Output: Particles are resampled in-place; all weights reset to 1/N.
 */
void systematicResample(std::vector<Particle>& P);

}} // namespace
