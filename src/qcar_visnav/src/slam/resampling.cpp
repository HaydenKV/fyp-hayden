#include "qcar_visnav/slam/resampling.h"
#include <random>
#include <numeric>
#include <algorithm>

namespace qcar_visnav { namespace slam {

void systematicResample(std::vector<Particle>& P)
{
  const size_t N = P.size();
  if (N == 0) return;

  // Ensure weights sum to 1 (tolerate tiny drift)
  double wsum = 0.0;
  for (const auto& p : P) wsum += p.weight;
  if (wsum <= 0.0) {
    const double w0 = 1.0 / static_cast<double>(N);
    for (auto& p : P) p.weight = w0;
    wsum = 1.0;
  } else {
    for (auto& p : P) p.weight /= wsum;
  }

  // Build cumulative distribution function (CDF)
  std::vector<double> cdf(N, 0.0);
  double acc = 0.0;
  for (size_t i = 0; i < N; ++i) {
    acc += P[i].weight;
    cdf[i] = acc;
  }
  // Guard last element to exactly 1.0 (avoid edge cases)
  cdf.back() = 1.0;

  // Systematic resampling:
  // u0 ~ U[0, 1/N), then u_k = u0 + k/N
  static std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> uni(0.0, 1.0 / static_cast<double>(N));
  const double u0 = uni(rng);

  std::vector<Particle> newP;
  newP.reserve(N);

  size_t i = 0;
  for (size_t k = 0; k < N; ++k) {
    const double u = u0 + static_cast<double>(k) / static_cast<double>(N);
    while (i + 1 < N && u > cdf[i]) ++i;
    newP.push_back(P[i]);  // copy particle & its map
    newP.back().weight = 1.0 / static_cast<double>(N);  // reset weight
  }

  P.swap(newP);
}

}} // namespace
