#include "qcar_visnav/estimation/sensors/gyro.h"

namespace qcar_nav {

void GyroMeas::predict(const XVec& x,
                       const qcar_nav::ModelParams& /*p*/,
                       ZVec& h,
                       HVec& H) const
{
  h(0,0) = x(4) + x(5); // r + bg
  H.setZero();
  H(0,4) = 1.0;
  H(0,5) = 1.0;
}

} // namespace qcar_nav
