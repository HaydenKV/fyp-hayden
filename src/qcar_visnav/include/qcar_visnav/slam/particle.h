#pragma once
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "landmark.h"

namespace qcar_visnav { namespace slam {

struct Particle {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double weight{0.0};
  int    id{0};

  // Landmarks live in the map/odom frame
  std::vector<Landmark, Eigen::aligned_allocator<Landmark>> map;
};

}} // namespace
