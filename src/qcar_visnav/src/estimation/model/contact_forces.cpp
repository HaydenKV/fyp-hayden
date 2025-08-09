#include "params.h"

namespace qcar {
namespace model {

// Returns vertical contact forces on front and rear wheels
void contactNormalForces(const Params &params, double &Wf, double &Wr) {
    const double g = params.g;
    const double mc = params.mc;
    const double mw = params.mw;
    const double l = params.l;

    const double total_mass = mc + 2.0 * mw;
    const double W = total_mass * g;

    // Uniform distribution of weight across front and rear wheels
    // Since l is symmetric (Lf = Lr), both get half the weight
    Wf = W / 2.0;
    Wr = W / 2.0;
}

}  // namespace model
}  // namespace qcar
