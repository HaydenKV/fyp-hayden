#pragma once
#include "types.h"
#include <vector>
#include <random>
#include <numeric>

namespace qcar_visnav {
namespace slam {

// Systematic resampling; returns new set with normalized weights
inline std::vector<Particle> systematic_resample(
    const std::vector<Particle>& in,
    std::mt19937& gen)
{
  const int N = (int)in.size();
  std::vector<double> w(N);
  double sumw = 0.0;
  for (int i=0;i<N;++i){ w[i]=in[i].weight; sumw+=w[i]; }
  if (sumw <= 0.0) {
    std::vector<Particle> out = in;
    for (auto& p: out) p.weight = 1.0 / N;
    return out;
  }
  for (double& wi: w) wi /= sumw;

  std::vector<double> cdf(N, 0.0);
  std::partial_sum(w.begin(), w.end(), cdf.begin());

  std::uniform_real_distribution<double> U(0.0, 1.0 / N);
  double r0 = U(gen);
  int i = 0;
  std::vector<Particle> out; out.reserve(N);
  for (int m = 0; m < N; ++m) {
    const double u = r0 + (double)m / N;
    while (u > cdf[i] && i < N-1) ++i;
    out.push_back(in[i]);      // shallow copy of maps (OK)
    out.back().weight = 1.0 / N;
  }
  return out;
}

inline double neff(const std::vector<Particle>& P) {
  double sumw = 0.0, sumw2 = 0.0;
  for (const auto& p : P) { sumw += p.weight; sumw2 += p.weight*p.weight; }
  if (sumw2 <= 1e-12) return (double)P.size();
  return (sumw*sumw) / sumw2;
}

} // namespace slam
} // namespace qcar_visnav
