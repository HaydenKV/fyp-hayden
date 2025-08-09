#include "params.h"
#include <Eigen/Dense>

namespace qcar {
namespace model {

using namespace qcar::params;

Eigen::Matrix<double, 6, 6> computeCoriolisMatrix(const Eigen::Matrix<double, 6, 1>& x) {
    Eigen::Matrix<double, 6, 6> C = Eigen::Matrix<double, 6, 6>::Zero();

    const double r = x(2);  // yaw rate

    const double C_scalar_5[5][5] = {
        {  0,        -(mc + 2 * mw), -mc * c,  0, 0 },
        {  mc + 2 * mw,         0,       0,  0, 0 },
        {  mc * c,              0,       0,  0, 0 },
        {  0,                   0,       0,  0, 0 },
        {  0,                   0,       0,  0, 0 }
    };

    // Fill the top-left 5x5 block with r*C_scalar_5
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            C(i, j) = r * C_scalar_5[i][j];

    return C;
}

} // namespace model
} // namespace qcar
