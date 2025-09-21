#include "qcar_supervisor/helpers/skew.h"

namespace qcar_nav {
namespace helpers {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<     0, -v(2),  v(1),
        v(2),     0, -v(0),
       -v(1),  v(0),     0;
  return S;
}

} // namespace helpers
} // namespace qcar_nav
