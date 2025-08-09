#include "params.h"
#include "wheel_slip.h"
#include <Eigen/Dense>
#include <cmath>

namespace qcar {
namespace model {

using namespace qcar::params;

Eigen::Matrix<double, 6, 6> computeDampingMatrix(const Eigen::Matrix<double, 6, 1>& x) {
    Eigen::Matrix<double, 3, 6> AF, AR;
    Eigen::Matrix<double, 3, 1> vWFNb, vWRNb;

    computeWheelSlip(x, AF, AR, vWFNb, vWRNb);

    const double epsilon = 0.01;

    double betaF = muw * Wf / std::sqrt(vWFNb.transpose() * vWFNb + epsilon * epsilon);
    double betaR = muw * Wr / std::sqrt(vWRNb.transpose() * vWRNb + epsilon * epsilon);

    Eigen::Matrix<double, 6, 6> DF = betaF * (AF.transpose() * AF);
    Eigen::Matrix<double, 6, 6> DR = betaR * (AR.transpose() * AR);

    Eigen::Matrix<double, 6, 6> D = Eigen::Matrix<double, 6, 6>::Zero();
    D(3, 3) = (Ng * Ng * Km * Km) / Ra;
    D(5, 5) = (Ks * Ks) / Rs;

    return D + DF + DR;
}

}  // namespace model
}  // namespace qcar
