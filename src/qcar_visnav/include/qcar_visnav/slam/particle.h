#pragma once
#include <vector>
#include <Eigen/Dense>
#include "qcar_visnav/slam/landmark.h"

namespace qcar_visnav { namespace slam {

struct Particle {
  // Pose in map_frame (world)
  double x{0.0}, y{0.0}, yaw{0.0};

  // Importance weight (and log form for stability)
  double weight{0.0};
  double log_w{0.0};

  int    id{0};

  // Map
  std::vector<Landmark> map;  // EKF landmarks in world frame
};

}} // namespace
