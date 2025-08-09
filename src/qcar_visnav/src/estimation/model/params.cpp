#include "params.h"
#include <cmath>

namespace qcar_visnav {

void QCarParams::computeDerivedConstants() {
    using namespace Eigen;

    // Mass Matrix M (6x6)
    M.setZero();
    M(0, 0) = mc + 2 * mw;            // u
    M(1, 1) = mc + 2 * mw;            // v
    M(2, 2) = mc * c * c + 2 * mw * l * l + I33; // r
    M(3, 3) = 2 * J22;               // omegaF
    M(4, 4) = 2 * J22;               // omegaR
    M(5, 5) = Ks * Ks / Rs;          // delta (steering inertia as damping)

    invM = M.inverse();

    // Actuator configuration matrix Ba (6x1)
    Ba.setZero();
    Ba(4, 0) = 2 * (Km * Ng) / Ra;

    // invMBa = inv(M) * Ba
    invMBa = invM * Ba;

    // Damping matrix Da
    Da.setZero();
    Da(3, 3) = (Ng * Ng * Km * Km) / Ra;
    Da(5, 5) = (Ks * Ks) / Rs;
    invMDa = invM * Da;

    // Coriolis scalar matrix (for invMC)
    Matrix<double, 5, 5> C5;
    C5 <<  0,        -mc - 2 * mw, -c * mc, 0, 0,
           mc + 2 * mw, 0,          0,     0, 0,
           c * mc,      0,          0,     0, 0,
           0,           0,          0,     0, 0,
           0,           0,          0,     0, 0;

    CScalar.setZero();
    CScalar.block<5, 5>(0, 0) = C5;
    invMC = invM * CScalar;

    // Placeholder for AF and AR (computed from wheelSlip later)
    AF.setZero();
    AR.setZero();

    // Vertical forces Wf and Wr
    Wf = (mw + 0.5 * mc) * g;
    Wr = (mw + 0.5 * mc) * g;
    muw_wF = muw * Wf;
    muw_wR = muw * Wr;

    // AFAF and ARAR are filled in once AF/AR known
    AFAF = AF.transpose() * AF;
    ARAR = AR.transpose() * AR;

    invMmuw_wF_AFAF = (invM * AFAF) * muw_wF;
    invMmuw_wR_ARAR = (invM * ARAR) * muw_wR;
}

} // namespace qcar_visnav
