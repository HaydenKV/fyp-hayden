#include "params.h"
#include <Eigen/Dense>

namespace qcar {
namespace model {

using namespace qcar::params;

void computeWheelSlip(
    const Eigen::Matrix<double, 6, 1>& x,
    Eigen::Matrix<double, 3, 6>& AF,
    Eigen::Matrix<double, 3, 6>& AR,
    Eigen::Matrix<double, 3, 1>& vWFNb,
    Eigen::Matrix<double, 3, 1>& vWRNb
) {
    const double delta = x(5);  // steering angle (δ)

    // Base front matrix (before rotation)
    Eigen::Matrix<double, 3, 6> AF0;
    AF0 << 1, 0, 0, -rw, 0, 0,
           0, 1, l,   0, 0, 0,
           0, 0, 0,   0, 0, 0;

    // Steering rotation matrix
    Eigen::Matrix3d R_delta;
    R_delta << std::cos(delta), -std::sin(delta), 0,
               std::sin(delta),  std::cos(delta), 0,
               0,                0,               1;

    // Apply steering to front wheel matrix
    AF = R_delta * AF0;

    // Rear matrix
    AR << 1, 0, 0, 0, -rw, 0,
          0, 1, -l, 0,  0, 0,
          0, 0, 0, 0,  0, 0;

    // Compute slip velocities
    vWFNb = AF * x;
    vWRNb = AR * x;
}

} // namespace model
} // namespace qcar
