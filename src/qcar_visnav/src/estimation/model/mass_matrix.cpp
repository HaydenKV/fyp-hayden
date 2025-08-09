#include "params.h"
#include <Eigen/Dense>

namespace qcar {
namespace model {

using namespace qcar::params;

Eigen::Matrix<double, 6, 6> computeMassMatrix() {
    Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Zero();

    // Mass terms
    double I_chassis = Izz;
    double I_w_pitch = J22;
    double I_w_yaw   = J33;

    M(0, 0) = mc + 2 * mw;
    M(1, 1) = mc + 2 * mw;
    M(2, 2) = I_chassis + 2 * mw * l * l;
    M(3, 3) = I_w_pitch;
    M(4, 4) = I_w_pitch;
    M(5, 5) = (Ks * Ks) / Rs;

    // Off-diagonal coupling terms
    M(0, 2) = -mc * 0;  // -mc * c; but c = 0
    M(1, 2) = -mc * l;
    M(2, 0) = M(0, 2);
    M(2, 1) = M(1, 2);

    return M;
}

} // namespace model
} // namespace qcar
