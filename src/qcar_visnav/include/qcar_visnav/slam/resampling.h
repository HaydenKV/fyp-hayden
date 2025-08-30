#pragma once
#include <vector>
#include <random>
#include "particle.h"

namespace qcar_visnav { namespace slam {

/**
 * Systematic resampling with caller-provided RNG (deterministic under a seed).
 */
void systematicResample(std::vector<Particle>& P, std::mt19937& rng);

}} // namespace
