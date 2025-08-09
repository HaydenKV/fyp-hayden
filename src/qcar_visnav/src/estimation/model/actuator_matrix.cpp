#include "params.h"
#include <Eigen/Dense>

namespace qcar {
namespace model {

Eigen::Matrix<double, 6, 1> actuatorConfigurationMatrix(const Params &params) {
    Eigen::Matrix<double, 6, 1> Ba = Eigen::Matrix<double, 6, 1>::Zero();

    // Set the drive (rear motor) component
    Ba(4, 0) = 2.0 * (params.Km * params.Ng) / params.Ra;

    // The steering component is not set here because it is handled separately in the EKF or controller

    return Ba;
}

} // namespace model
} // namespace qcar
