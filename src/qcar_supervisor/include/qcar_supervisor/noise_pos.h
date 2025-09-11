#pragma once
#include <Eigen/Dense>
#include <random>

namespace qcar_supervisor {

class PositionNoiser {
public:
    PositionNoiser(double stddev_x, double stddev_y)
        : gen_(std::random_device{}()),
          dist_x_(0.0, stddev_x),
          dist_y_(0.0, stddev_y) {}

    Eigen::Vector2d addNoise(const Eigen::Vector2d& pos) {
        Eigen::Vector2d noisy = pos;
        noisy[0] += dist_x_(gen_);
        noisy[1] += dist_y_(gen_);
        return noisy;
    }

private:
    std::default_random_engine gen_;
    std::normal_distribution<double> dist_x_;
    std::normal_distribution<double> dist_y_;
};

} // namespace qcar_supervisor