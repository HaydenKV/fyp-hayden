#include "qcar_visnav/estimation/sensors/speed.h"

namespace qcar_nav {

void SpeedMeas::predict(const XVec& x,
                        const qcar_nav::ModelParams& /*p*/,
                        ZVec& h,
                        HVec& H) const
{
  h(0,0) = x(3);  // v
  H.setZero();
  H(0,3) = 1.0;   // ∂h/∂v
}

} // namespace qcar_nav
