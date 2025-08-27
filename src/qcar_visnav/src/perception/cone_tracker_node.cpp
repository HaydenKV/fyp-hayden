// ============================================================================
// cone_tracker_node.cpp
//
// Global nearest-neighbor assignment (Hungarian) + EKF smoothing of cone detections.
// - Subscribes:  /cones (qcar_visnav/ConeArray)  — raw detections in LiDAR frame
// - Publishes:   /tracked_cones (qcar_visnav/ConeArray) — tracked cones in LiDAR frame
//
// DESIGN CHOICES (important):
//   • Internal state space: ODOM. We keep track means/covariances in a fixed world-like frame
//     (odom) for stable static-landmark filtering.
//   • I/O frames: Inputs arrive in LiDAR; we transform detections → ODOM for filtering.
//                 Outputs are converted back to LiDAR and published IN THE LIDAR FRAME
//                 (polar). This keeps the SLAM front-end consuming LiDAR-frame ConeArray,
//                 per your preference, while avoiding moving-frame filter instability.
//
// DATA FLOW
//   1) For each detection (r,theta, Σ_rθ) in LiDAR: compute (x,y,Σ_xy) in ODOM via TF.
//   2) Predict each track with small Q (static landmarks).
//   3) Build gated Mahalanobis costs and run Hungarian assignment (global NN).
//   4) Matched → EKF update (H=I). Unmatched detections → new tracks. Unmatched tracks → misses++.
//   5) Publish CONFIRMED tracks back in LiDAR frame (polar + covariance).
//
// PARAMETERS (private ns ~cone_tracker)
//   detections_topic [string] : input ConeArray (default "/cones")
//   tracks_topic     [string] : output ConeArray (default "/tracked_cones")
//   odom_frame       [string] : world-like frame to hold state (default "odom")
//   base_frame       [string] : kept for compatibility; NOT used for publishing now
//   gate_chi2        [double] : chi^2(2) gate (e.g. 5.99, 7.38)
//   init_hits        [int]    : confirm after this many consecutive hits
//   max_misses       [int]    : drop after this many consecutive misses
//   init_cov         [double] : initial covariance (m^2)
//   q_xy             [double] : process noise spectral density (m^2/s) for static jitter
//   r_scale          [double] : scale factor for detector measurement covariance
//   max_dt           [double] : cap dt during prediction
//   debug            [int]    : 0/1 verbosity
//
// Author: you (+ minimal code edits to publish in LiDAR + comment upgrades)
// ============================================================================

#include <ros/ros.h>
#include <qcar_visnav/ConeArray.h>
#include <qcar_visnav/Cone.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>
#include <memory>

// ============================
// Params (loaded via private namespace)
// ============================
struct Params {
  std::string detections_topic{"/cones"};         // input: from cone_detector_node
  std::string tracks_topic{"/tracked_cones"};     // output: confirmed tracks
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};       // kept for compatibility (not used for publishing)
  double chi2_gate{5.99};     // 95% @ 2DOF
  int    init_hits{2};        // promote to confirmed after this many consecutive hits
  int    max_misses{5};       // drop track after this many consecutive misses
  double init_cov{0.25};      // initial covariance (m^2)
  double q_xy{0.01};          // process noise spectral density (m^2/s) for static-landmark jitter
  double r_scale{1.0};        // scales detector measurement covariance
  double max_dt{0.2};         // cap dt in prediction for stability
  int    debug{1};
} P;

// ============================
// Track structure (state in ODOM)
// ============================
struct Track { // Persistent tracks (survive across frames)
  Eigen::Vector2d x{Eigen::Vector2d::Zero()};       // [X,Y] in ODOM (WHERE WE THINK THE CONE IS)
  Eigen::Matrix2d P{Eigen::Matrix2d::Identity()};   // covariance in ODOM (HOW UNCERTAIN WE ARE)
  uint32_t id{0}; // UNIQUE PERSISTENT ID
  int hits{0};          // consecutive hits
  int misses{0};        // consecutive misses (HOW MANY FRAMES SINCE LAST SEEN)
  bool confirmed{false}; // IS IT RELIABLE ENOUGH?
  ros::Time last_stamp; // time of last predict/update
  int color{0};
  double color_conf{0.0};
};

// ============================
// Globals
// ============================
static ros::Subscriber sub_dets; // subscribes to raw detections
static ros::Publisher  pub_tracks; // publishes confirmed tracks
static std::vector<Track> g_tracks; // THE PERSISTENT MEMORY
static uint32_t g_next_id = 1;  

static std::unique_ptr<tf2_ros::Buffer> tf_buffer;
static std::unique_ptr<tf2_ros::TransformListener> tf_listener;

// ----------------------------
// Math / TF helpers
// ----------------------------
static inline Eigen::Matrix2d Rot2(double yaw) {
  const double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix2d R; R << c, -s, s, c; return R;
}

static double yawFromTF(const geometry_msgs::TransformStamped &tf) {
  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

static inline Eigen::Vector2d polarToXY(double r, double th) {
  return { r*std::cos(th), r*std::sin(th) };
}

// Polar (r,th) covariance -> Cartesian (x,y) in same frame
static Eigen::Matrix2d polarCovToCart(double r, double th, double r_var, double th_var) {
  Eigen::Matrix2d J;
  J << std::cos(th), -r*std::sin(th),
       std::sin(th),  r*std::cos(th);
  Eigen::Matrix2d Rp = Eigen::Matrix2d::Zero();
  Rp(0,0) = r_var;
  Rp(1,1) = th_var;
  return J * Rp * J.transpose();
}

// Rotate covariance: S2 = R S R^T
static inline Eigen::Matrix2d rotateCov(const Eigen::Matrix2d &S, double yaw) {
  Eigen::Matrix2d R = Rot2(yaw);
  return R * S * R.transpose();
}

// Cartesian (x,y) covariance -> polar (r,th) at point p
static Eigen::Matrix2d cartCovToPolar(const Eigen::Vector2d &p, const Eigen::Matrix2d &Sxy) {
  const double x = p.x(), y = p.y();
  const double r2 = x*x + y*y;
  const double r  = std::sqrt(std::max(r2, 1e-12));
  Eigen::Matrix2d H; // d[r,th]/d[x,y]
  H <<  x/r,  y/r,
       -y/r2, x/r2;
  return H * Sxy * H.transpose();
}

// Transform a 2D point (x,y,0) from src->dst at given stamp; also return yaw(src->dst)
static bool transformXY(const Eigen::Vector2d &p_src,
                        const std::string &src_frame,
                        const std::string &dst_frame,
                        const ros::Time &stamp,
                        Eigen::Vector2d &p_dst,
                        double &yaw_src_to_dst)
{
  geometry_msgs::PointStamped ps_in, ps_out;
  ps_in.header.stamp = stamp;
  ps_in.header.frame_id = src_frame;
  ps_in.point.x = p_src.x();
  ps_in.point.y = p_src.y();
  ps_in.point.z = 0.0;

  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer->lookupTransform(dst_frame, src_frame, stamp, ros::Duration(0.05));
    tf_buffer->transform(ps_in, ps_out, dst_frame, ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[cone_tracker] TF %s->%s at t=%.3f failed: %s",
                      src_frame.c_str(), dst_frame.c_str(), stamp.toSec(), ex.what());
    return false;
  }
  p_dst = { ps_out.point.x, ps_out.point.y };
  yaw_src_to_dst = yawFromTF(tf);
  return true;
}

// ----------------------------
// Hungarian (min-cost assignment) on a square matrix
// ----------------------------
static std::vector<int> hungarian(const std::vector<std::vector<double>>& C,
                                  double unmatched_cost)
{
  const int n = static_cast<int>(C.size());
  const double INF = 1e18;

  // 1-indexed potentials and matching
  std::vector<double> u(n+1, 0.0), v(n+1, 0.0);
  std::vector<int> p(n+1, 0), way(n+1, 0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n+1, INF);
    std::vector<char> used(n+1, false);

    do {
      used[j0] = true;
      int i0 = p[j0];
      double delta = INF;
      int j1 = 0;

      for (int j = 1; j <= n; ++j) if (!used[j]) {
        double cur = C[i0-1][j-1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
        else { minv[j] -= delta; }
      }
      j0 = j1;
    } while (p[j0] != 0);

    // augment
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  // Build assignment by row (0..n-1)
  std::vector<int> assignment(n, -1);
  for (int j = 1; j <= n; ++j) {
    if (p[j] != 0) {
      int i = p[j] - 1;
      int jj = j - 1;
      if (C[i][jj] >= unmatched_cost - 1e-9) assignment[i] = -1;
      else assignment[i] = jj;
    }
  }
  return assignment;
}

// ============================
// Callback
// ============================
void conesCb(const qcar_visnav::ConeArray::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp;
  const std::string lidar_frame = msg->header.frame_id.empty() ? "lidar" : msg->header.frame_id;

  // TF we need:
  //  (1) lidar -> odom (to convert detections into ODOM for filtering)
  geometry_msgs::TransformStamped tf_lidar_odom;
  try {
    tf_lidar_odom = tf_buffer->lookupTransform(P.odom_frame, lidar_frame, stamp, ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[cone_tracker] Cannot get TF %s->%s: %s",
                      lidar_frame.c_str(), P.odom_frame.c_str(), ex.what());
    return;
  }
  const double yaw_lidar_to_odom = yawFromTF(tf_lidar_odom);
  const Eigen::Matrix2d R_lo = Rot2(yaw_lidar_to_odom);
  const Eigen::Vector2d t_lo(tf_lidar_odom.transform.translation.x,
                             tf_lidar_odom.transform.translation.y);

  // Build detections in ODOM
  struct Det {
    Eigen::Vector2d z_odom;
    Eigen::Matrix2d R_odom;
    int color; double color_conf; int32_t det_id;
  };
  std::vector<Det> dets; dets.reserve(msg->cones.size());

  for (const auto &c : msg->cones) {
    const double r = c.range, th = c.bearing;
    if (!std::isfinite(r) || !std::isfinite(th)) continue;

    // LiDAR polar -> LiDAR Cartesian
    Eigen::Vector2d p_lidar = polarToXY(r, th);

    // LiDAR -> ODOM: p_odom = R_lo * p_lidar + t_lo
    Eigen::Vector2d p_odom = R_lo * p_lidar + t_lo;

    // Covariance: polar (LiDAR) -> Cartesian (LiDAR) -> rotate into ODOM
    const double r_var  = std::max(1e-8, c.r_var) * P.r_scale;
    const double th_var = std::max(1e-8, c.bearing_var) * P.r_scale;
    Eigen::Matrix2d S_lidar = polarCovToCart(r, th, r_var, th_var);
    Eigen::Matrix2d S_odom  = R_lo * S_lidar * R_lo.transpose();

    dets.push_back({ p_odom, S_odom, c.color, c.color_conf, c.id });
  }

  const int Nd = static_cast<int>(dets.size());
  const int Nt = static_cast<int>(g_tracks.size());

  // Predict (static landmarks): x_k|k-1 = x_{k-1},  P += Q*dt
  for (auto &t : g_tracks) {
    double dt = (stamp - t.last_stamp).toSec();
    if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
    dt = std::min(dt, P.max_dt);
    const Eigen::Matrix2d Q = (std::max(0.0, P.q_xy) * std::max(dt, 1e-3)) * Eigen::Matrix2d::Identity();
    t.P += Q;
  }

  // ----------------------------
  // Global assignment via Hungarian
  // ----------------------------
  const int n = std::max(Nd, Nt);
  const double UNMATCHED_COST = P.chi2_gate + 10.0; // larger than any gated pair

  // Build square cost matrix with dummy rows/cols
  std::vector<std::vector<double>> C(n, std::vector<double>(n, UNMATCHED_COST));

  // Fill real pair costs with gated Mahalanobis distance^2
  for (int i = 0; i < Nd; ++i) {
    for (int j = 0; j < Nt; ++j) {
      const Track &t = g_tracks[j];
      Eigen::Vector2d v = dets[i].z_odom - t.x;
      Eigen::Matrix2d S = t.P + dets[i].R_odom;
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;
      double d2 = v.dot( llt.solve(v) );
      if (std::isfinite(d2) && d2 <= P.chi2_gate) {
        C[i][j] = d2;
      }
    }
  }

  // Solve assignment
  std::vector<int> assign = hungarian(C, UNMATCHED_COST);

  // Decode: pairs, unmatched dets, unmatched tracks
  std::vector<std::pair<int,int>> pairs;
  std::vector<char> det_used(Nd, 0), trk_used(Nt, 0);

  for (int i = 0; i < Nd; ++i) {
    int j = assign[i];
    if (j >= 0 && j < Nt) {
      if (C[i][j] < UNMATCHED_COST - 1e-9) {
        pairs.emplace_back(i, j);
        det_used[i] = 1;
        trk_used[j] = 1;
      }
    }
  }

  // Update matched tracks (EKF with H=I)
  for (const auto &pr : pairs) {
    const int i = pr.first, j = pr.second;
    Track &t = g_tracks[j];
    const auto &d = dets[i];

    Eigen::Matrix2d S = t.P + d.R_odom;
    Eigen::Matrix2d K = t.P * S.inverse();
    t.x += K * (d.z_odom - t.x);
    t.P  = (Eigen::Matrix2d::Identity() - K) * t.P;

    t.misses = 0;
    t.hits   = std::min(t.hits + 1, 1000000);
    t.last_stamp = stamp;

    if (d.color_conf > 0.0) {
      t.color_conf = 0.7*t.color_conf + 0.3*d.color_conf;
      if (t.color_conf >= 0.5) t.color = d.color;
    }
    if (!t.confirmed && t.hits >= P.init_hits) t.confirmed = true;
  }

  // Missed tracks
  for (int j = 0; j < Nt; ++j) {
    if (trk_used[j]) continue;
    Track &t = g_tracks[j];
    t.misses++;
    t.hits = 0;
    t.last_stamp = stamp;
  }

  // New tracks from unmatched detections
  for (int i = 0; i < Nd; ++i) {
    if (det_used[i]) continue;
    const auto &d = dets[i];

    Track t;
    t.x = d.z_odom;
    t.P = Eigen::Matrix2d::Identity() * P.init_cov;
    t.id = g_next_id++;
    t.hits = 1;
    t.misses = 0;
    t.confirmed = (P.init_hits <= 1);
    t.last_stamp = stamp;
    t.color = d.color;
    t.color_conf = d.color_conf;

    g_tracks.emplace_back(std::move(t));
  }

  // Prune stale
  g_tracks.erase(std::remove_if(g_tracks.begin(), g_tracks.end(),
                [](const Track &t){ return t.misses > P.max_misses; }),
                g_tracks.end());

  // ------------------------------------------------------------------------
  // Publish confirmed tracks as ConeArray IN THE LIDAR FRAME (polar).
  // We currently have each track in ODOM. Convert to LiDAR at 'stamp':
  //
  //   x_lidar = R_lo^T * (x_odom - t_lo)
  //   P_lidar = R_lo^T *  P_odom * R_lo
  //
  // Then convert to (range,bearing) + polar covariance.
// ------------------------------------------------------------------------
  qcar_visnav::ConeArray out;
  out.header.stamp = stamp;
  out.header.frame_id = lidar_frame;

  size_t n_pub = 0;
  const Eigen::Matrix2d R_ol = R_lo.transpose();   // ODOM->LiDAR rotation

  for (const auto &t : g_tracks) {
    if (!t.confirmed) continue;

    // ODOM -> LiDAR
    Eigen::Vector2d p_lidar = R_ol * (t.x - t_lo);
    Eigen::Matrix2d S_lidar = R_ol * t.P * R_ol.transpose();

    const double r  = std::hypot(p_lidar.x(), p_lidar.y());
    const double th = std::atan2(p_lidar.y(), p_lidar.x());
    Eigen::Matrix2d S_polar = cartCovToPolar(p_lidar, S_lidar);

    qcar_visnav::Cone c;
    c.range = r;
    c.bearing = th;
    c.r_var = std::max(1e-10, S_polar(0,0));
    c.bearing_var = std::max(1e-12, S_polar(1,1));
    c.color = t.color;
    c.color_conf = t.color_conf;
    c.id = static_cast<int32_t>(t.id);

    out.cones.push_back(c);
    ++n_pub;
  }

  if (P.debug) {
    ROS_INFO_THROTTLE(0.5,
      "[cone_tracker] det=%d matched=%zu new=%d active=%zu published=%zu frame=%s",
      Nd, pairs.size(), Nd - (int)pairs.size(), g_tracks.size(), n_pub, lidar_frame.c_str());
  }

  pub_tracks.publish(out); // Publish to /tracked_cones topic
}

// ============================
// Main
// ============================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Load params
  pnh.param("detections_topic", P.detections_topic, P.detections_topic);
  pnh.param("tracks_topic",     P.tracks_topic,     P.tracks_topic);
  pnh.param("odom_frame",       P.odom_frame,       P.odom_frame);
  pnh.param("base_frame",       P.base_frame,       P.base_frame); // not used for publishing
  pnh.param("gate_chi2",        P.chi2_gate,        P.chi2_gate);
  pnh.param("init_hits",        P.init_hits,        P.init_hits);
  pnh.param("max_misses",       P.max_misses,       P.max_misses);
  pnh.param("init_cov",         P.init_cov,         P.init_cov);
  pnh.param("q_xy",             P.q_xy,             P.q_xy);
  pnh.param("r_scale",          P.r_scale,          P.r_scale);
  pnh.param("max_dt",           P.max_dt,           P.max_dt);
  pnh.param("debug",            P.debug,            P.debug);

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

  pub_tracks = nh.advertise<qcar_visnav::ConeArray>(P.tracks_topic, 1, false);
  sub_dets   = nh.subscribe<qcar_visnav::ConeArray>(P.detections_topic, 1, &conesCb);

  ROS_INFO("[cone_tracker] up. subs='%s' -> pubs='%s', state_frame='%s', out_frame='LiDAR input frame'",
           P.detections_topic.c_str(), P.tracks_topic.c_str(), P.odom_frame.c_str());

  ros::spin();
  return 0;
}
