#include "qcar_visnav/slam/measurement_model.h"

namespace qcar_visnav { namespace slam {

static inline Eigen::Matrix2d R(double a) {
  const double c = std::cos(a), s = std::sin(a);
  Eigen::Matrix2d M;
  M << c,-s,
       s, c;
  return M;
}

// World landmark -> lidar-relative Cartesian (and C = d v_lidar / d m_world)
void worldToLidarDelta(double x, double y, double yaw,
                       const Eigen::Vector2d& m_world,
                       const LidarExtrinsics& ex,
                       double& lx, double& ly, Eigen::Matrix2d& C)
{
  const Eigen::Vector2d pw(x, y);
  const Eigen::Vector2d ex_b(ex.x, ex.y);

  const Eigen::Matrix2d Rwb = R(yaw);
  const Eigen::Vector2d p_wl = pw + Rwb * ex_b;   // lidar pos in world
  const double yaw_wl = yaw + ex.yaw;

  // world -> lidar
  const Eigen::Vector2d d_w = m_world - p_wl;
  const Eigen::Matrix2d Rlw = R(yaw_wl).transpose();

  const Eigen::Vector2d v_l = Rlw * d_w;
  lx = v_l.x();
  ly = v_l.y();

  // Jacobian wrt world landmark state
  C = Rlw;
}

// Predict (r_hat, b_hat) and H = d h / d m_world (2x2)
void predictMeasurementRB(double x, double y, double yaw,
                          const Eigen::Vector2d& m_world,
                          const LidarExtrinsics& ex,
                          double& r_hat, double& b_hat,
                          Eigen::Matrix2d& H)
{
  double lx, ly;
  Eigen::Matrix2d C;
  worldToLidarDelta(x, y, yaw, m_world, ex, lx, ly, C);

  const double eps = 1e-9;
  const double r = std::sqrt(std::max(eps, lx*lx + ly*ly));
  r_hat = r;
  b_hat = std::atan2(ly, lx);

  // J_rb wrt (lx, ly)
  Eigen::Matrix2d Jrb;
  Jrb << lx/r,  ly/r,
        -ly/(r*r), lx/(r*r);

  // Chain rule: H = Jrb * C
  H = Jrb * C;
}

// Convert one RB meas to world landmark; J = d m_world / d z
void measRBToWorld(double x, double y, double yaw,
                   const LidarExtrinsics& ex,
                   double r, double b,
                   Eigen::Vector2d& m_world,
                   Eigen::Matrix2d& J)
{
  const Eigen::Vector2d pw(x, y);
  const Eigen::Vector2d ex_b(ex.x, ex.y);

  const Eigen::Matrix2d Rwb = R(yaw);
  const Eigen::Vector2d p_wl = pw + Rwb * ex_b;
  const double yaw_wl = yaw + ex.yaw;

  const Eigen::Matrix2d Rwl = R(yaw_wl);

  const double cb = std::cos(b), sb = std::sin(b);
  const Eigen::Vector2d v_l(r * cb, r * sb);

  // World landmark position
  m_world = p_wl + Rwl * v_l;

  // J = d m_world / d [r, b]
  Eigen::Matrix2d Jlocal;
  Jlocal << cb, -r*sb,
            sb,  r*cb;
  J = Rwl * Jlocal;
}

// Skew-symmetric 2D helper
static inline Eigen::Matrix2d Skew2() {
  Eigen::Matrix2d S; S << 0, -1, 1, 0; return S;
}

// Predict (r_hat, b_hat) plus Jacobians wrt pose (Gx: 2x3) and landmark (H: 2x2)
void predictRBWithJacobians(double x, double y, double yaw,
                            const Eigen::Vector2d& m_world,
                            const LidarExtrinsics& ex,
                            double& r_hat, double& b_hat,
                            Eigen::Matrix<double,2,3>& Gx,
                            Eigen::Matrix2d& H)
{
  // Reuse existing pieces
  double lx, ly; Eigen::Matrix2d C;
  worldToLidarDelta(x, y, yaw, m_world, ex, lx, ly, C); // C = d v_lidar / d m_world

  const double eps = 1e-9;
  const double r = std::sqrt(std::max(eps, lx*lx + ly*ly));
  r_hat = r;
  b_hat = std::atan2(ly, lx);

  // J_rb wrt (lx, ly)
  Eigen::Matrix2d Jrb;
  Jrb << lx/r,  ly/r,
        -ly/(r*r), lx/(r*r);

  // Landmark jacobian
  H = Jrb * C;

  // Pose jacobian Gx
  // v_l = Rlw * (m - p_wl), with:
  //   Rlw = R(yaw + ex.yaw)^T,  p_wl = [x;y] + R(yaw) * [ex.x; ex.y]
  const Eigen::Vector2d ex_b(ex.x, ex.y);
  const Eigen::Matrix2d Rwb = R(yaw);
  const Eigen::Vector2d p_wl = Eigen::Vector2d(x, y) + Rwb * ex_b;
  const double yaw_wl = yaw + ex.yaw;
  const Eigen::Matrix2d Rlw = R(yaw_wl).transpose();
  const Eigen::Vector2d d_w = m_world - p_wl;

  // Derivatives of v_l wrt pose:
  // dv/dx = -Rlw * [1;0]
  // dv/dy = -Rlw * [0;1]
  // dv/dyaw = (-Rlw*Skew2()) * d_w - Rlw * (R(yaw)*Skew2()*ex_b)
  const Eigen::Vector2d dv_dx = - Rlw * Eigen::Vector2d::UnitX();
  const Eigen::Vector2d dv_dy = - Rlw * Eigen::Vector2d::UnitY();
  const Eigen::Vector2d dv_dyaw =
      (-Rlw * Skew2()) * d_w - Rlw * (Rwb * (Skew2() * ex_b));

  // Chain to RB
  Gx.col(0) = Jrb * dv_dx;
  Gx.col(1) = Jrb * dv_dy;
  Gx.col(2) = Jrb * dv_dyaw;
}

}} // ns
