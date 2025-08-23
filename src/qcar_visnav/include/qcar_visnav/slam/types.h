#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <cmath>

namespace qcar_visnav {
namespace slam {

struct Landmark {
  Eigen::Vector2d mu{Eigen::Vector2d::Zero()};
  Eigen::Matrix2d Sigma{Eigen::Matrix2d::Identity()};
  int hits{0};
  int misses{0};
  int color{0};          // optional: 0=unknown
  double color_conf{0.0};
};

struct Particle {
  Eigen::Vector3d pose{Eigen::Vector3d::Zero()}; // x,y,yaw (odom)
  double weight{1.0};
  std::vector<Landmark> map;
};

inline Eigen::Matrix2d Rot2(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix2d R; R << c, -s, s,  c; return R;
}

} // namespace slam
} // namespace qcar_visnav
