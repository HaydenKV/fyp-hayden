#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <qcar_visnav/ConeArray.h>
#include <qcar_visnav/Cone.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>

// ============================
// Params
// ============================
struct Params {
  std::string detections_topic{"/cones"};   // from cone_detector_node.cpp
  std::string tracks_topic{"/tracked_cones"};           // tracked + ID
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};     // EKF base frame
  double chi2_gate{5.99};                       // 95% @ 2DOF
  int init_hits{2};                             
  int max_misses{5};
  double init_cov{0.25};                        // P0 = init_cov * I (m^2)
  double q_xy{0.01};                            // process noise base (m^2/s)
  double r_scale{1.0};                          // scale detector R if needed
  double max_dt{0.2};                           // clamp dt for stability
  bool use_hungarian{false};                    // (greedy by default)
  int debug{1};
} P;

// ============================
// Track struct
// ============================
struct Track {
  Eigen::Vector2d x{Eigen::Vector2d::Zero()};    // in odom: [X,Y]
  Eigen::Matrix2d P{Eigen::Matrix2d::Identity()}; // covariance in odom
  uint32_t id{0};
  int hits{0};             // consecutive hits
  int misses{0};           // consecutive misses
  bool confirmed{false};
  ros::Time last_stamp;    // time of last update/predict
  int color{0};
  double color_conf{0.0};
};

// ============================
// Globals
// ============================
static ros::Subscriber sub_dets;
static ros::Publisher pub_tracks;
static std::vector<Track> g_tracks;
static uint32_t g_next_id = 1;

static std::unique_ptr<tf2_ros::Buffer> tf_buffer;
static std::unique_ptr<tf2_ros::TransformListener> tf_listener;

// ----------------------------
// Helpers
// ----------------------------

// 2D rotation matrix from yaw
static inline Eigen::Matrix2d Rot2(double yaw) {
  double c = std::cos(yaw), s = std::sin(yaw);
  Eigen::Matrix2d R; R << c, -s, s, c; return R;
}

// Extract yaw from geometry_msgs::TransformStamped (odom->frame)
static double yawFromTF(const geometry_msgs::TransformStamped &tf) {
  tf2::Quaternion q;
  tf2::fromMsg(tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// Convert detector polar (in lidar frame) -> Cartesian in lidar
static inline Eigen::Vector2d polarToXY(double r, double th) {
  return Eigen::Vector2d(r * std::cos(th), r * std::sin(th));
}

// Polar covariance (r,theta) -> Cartesian covariance (x,y) in *same* frame
static Eigen::Matrix2d polarCovToCart(double r, double th, double r_var, double th_var) {
  // J = d[x,y]/d[r,th] = [[cos th, -r sin th],
  //                       [sin th,  r cos th]]
  Eigen::Matrix2d J;
  J << std::cos(th), -r * std::sin(th),
       std::sin(th),  r * std::cos(th);
  Eigen::Matrix2d Rp = Eigen::Matrix2d::Zero();
  Rp(0,0) = r_var;
  Rp(1,1) = th_var;
  return J * Rp * J.transpose();
}

// Rotate covariance: Σ2 = R Σ R^T
static inline Eigen::Matrix2d rotateCov(const Eigen::Matrix2d &Sigma, double yaw) {
  Eigen::Matrix2d R = Rot2(yaw);
  return R * Sigma * R.transpose();
}

// Cartesian covariance (x,y) -> polar covariance (r,theta) at point (x,y)
static Eigen::Matrix2d cartCovToPolar(const Eigen::Vector2d &p, const Eigen::Matrix2d &Sxy) {
  const double x = p.x(), y = p.y();
  const double r2 = x*x + y*y;
  const double r  = std::sqrt(std::max(r2, 1e-12));
  // H = d[r,theta]/d[x,y] = [[x/r, y/r],
  //                          [-y/r^2, x/r^2]]
  Eigen::Matrix<double,2,2> H;
  H << x/r, y/r,
      -y/(r2), x/(r2);
  return H * Sxy * H.transpose();
}

// Transform a 2D point (x,y,0) from src->dst at given stamp; also return yaw of rotation
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
  p_dst = Eigen::Vector2d(ps_out.point.x, ps_out.point.y);
  yaw_src_to_dst = yawFromTF(tf);
  return true;
}

// Greedy 1-1 assignment from a set of (i_det, j_track, dist) sorted by dist
struct Candidate { int i; int j; double d; };
static std::vector<std::pair<int,int>> greedyAssign(std::vector<Candidate> cand, int Nd, int Nt)
{
  std::sort(cand.begin(), cand.end(), [](const Candidate &a, const Candidate &b){ return a.d < b.d;});
  std::vector<char> det_used(Nd, 0), trk_used(Nt, 0);
  std::vector<std::pair<int,int>> pairs;
  pairs.reserve(std::min(Nd,Nt));
  for (const auto &c : cand) {
    if (!det_used[c.i] && !trk_used[c.j]) {
      det_used[c.i] = trk_used[c.j] = 1;
      pairs.emplace_back(c.i, c.j);
    }
  }
  return pairs;
}

// ============================
// Callback
// ============================
void conesCb(const qcar_visnav::ConeArray::ConstPtr &msg)
{
  const ros::Time stamp = msg->header.stamp;
  const std::string lidar_frame = msg->header.frame_id.empty() ? "lidar" : msg->header.frame_id;

  // Precompute TFs used in this callback:
  // 1) lidar -> odom (for detections to odom)
  geometry_msgs::TransformStamped tf_lidar_odom;
  try {
    tf_lidar_odom = tf_buffer->lookupTransform(P.odom_frame, lidar_frame, stamp, ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[cone_tracker] Cannot get TF %s->%s: %s",
                      lidar_frame.c_str(), P.odom_frame.c_str(), ex.what());
    return;
  }
  const double yaw_lidar_to_odom = yawFromTF(tf_lidar_odom); // for covariance rotation

  // 2) odom -> base (for publishing polar from base_footprint)
  geometry_msgs::TransformStamped tf_odom_base;
  try {
    tf_odom_base = tf_buffer->lookupTransform(P.base_frame, P.odom_frame, stamp, ros::Duration(0.05));
  } catch (const tf2::TransformException &ex) {
    ROS_WARN_THROTTLE(1.0, "[cone_tracker] Cannot get TF %s->%s: %s",
                      P.odom_frame.c_str(), P.base_frame.c_str(), ex.what());
    return;
  }
  const double yaw_odom_to_base = yawFromTF(tf_odom_base);

  // Build detection list in ODOM coordinates + R in ODOM
  struct Det {
    Eigen::Vector2d z_odom;
    Eigen::Matrix2d R_odom;
    int color;
    double color_conf;
    int32_t det_id;
  };
  std::vector<Det> dets; dets.reserve(msg->cones.size());

  for (const auto &c : msg->cones) {
    double r = c.range;
    double th = c.bearing;
    if (!std::isfinite(r) || !std::isfinite(th)) continue;

    // 1) polar->cart in LIDAR
    Eigen::Vector2d p_lidar = polarToXY(r, th);
    // 2) point to ODOM
    Eigen::Vector2d p_odom;
    double yaw_tmp = 0.0;
    if (!transformXY(p_lidar, lidar_frame, P.odom_frame, stamp, p_odom, yaw_tmp)) continue;

    // 3) covariance: polar->cart in LIDAR, then rotate to ODOM
    const double r_var = std::max(1e-8, c.r_var) * P.r_scale;
    const double th_var = std::max(1e-8, c.bearing_var) * P.r_scale;
    Eigen::Matrix2d S_lidar = polarCovToCart(r, th, r_var, th_var);
    Eigen::Matrix2d S_odom  = rotateCov(S_lidar, yaw_lidar_to_odom);

    dets.push_back( Det{ p_odom, S_odom, c.color, c.color_conf, c.id } );
  }

  const int Nd = static_cast<int>(dets.size());
  const int Nt = static_cast<int>(g_tracks.size());

  // Predict tracks (static in odom): x_pred = x;  P_pred += Q*dt
  // dt per-track from its last_stamp to current stamp
  for (auto &t : g_tracks) {
    double dt = (stamp - t.last_stamp).toSec();
    if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
    dt = std::min(dt, P.max_dt);
    const double q = std::max(0.0, P.q_xy);
    const Eigen::Matrix2d Q = (q * std::max(dt, 1e-3)) * Eigen::Matrix2d::Identity();
    t.P = t.P + Q;
    // x unchanged (static landmarks in odom)
  }

  // Build gated candidate list
  std::vector<Candidate> candidates; candidates.reserve(Nd * Nt);
  for (int i = 0; i < Nd; ++i) {
    for (int j = 0; j < Nt; ++j) {
      Track &t = g_tracks[j];
      // Innovation in ODOM
      Eigen::Vector2d v = dets[i].z_odom - t.x;
      Eigen::Matrix2d S = t.P + dets[i].R_odom;
      // Mahalanobis distance^2
      Eigen::LLT<Eigen::Matrix2d> llt(S);
      if (llt.info() != Eigen::Success) continue;
      Eigen::Vector2d y = llt.solve(v);
      double d2 = v.dot(y);
      if (d2 <= P.chi2_gate && std::isfinite(d2))
        candidates.push_back({i, j, d2});
    }
  }

  // Assign (greedy by distance)
  std::vector<std::pair<int,int>> pairs = greedyAssign(candidates, Nd, Nt);
  std::vector<char> det_used(Nd, 0), trk_used(Nt, 0);
  for (const auto &pr : pairs) { det_used[pr.first] = 1; trk_used[pr.second] = 1; }

  // Updates for matched tracks
  for (const auto &pr : pairs) {
    const int i = pr.first, j = pr.second;
    Track &t = g_tracks[j];
    const auto &d = dets[i];

    // EKF update with H=I
    Eigen::Matrix2d S = t.P + d.R_odom;
    Eigen::Matrix2d K = t.P * S.inverse();
    t.x = t.x + K * (d.z_odom - t.x);
    t.P = (Eigen::Matrix2d::Identity() - K) * t.P;
    t.misses = 0;
    t.hits   = std::min(t.hits + 1, 1000000);
    t.last_stamp = stamp;

    // colour smoothing (EWMA)
    if (d.color_conf > 0.0) {
      t.color_conf = 0.7 * t.color_conf + 0.3 * d.color_conf;
      t.color = (t.color_conf >= 0.5) ? d.color : t.color; // crude: update only if confident
    }

    if (!t.confirmed && t.hits >= P.init_hits) t.confirmed = true;
  }

  // Handle unmatched tracks (miss)
  for (int j = 0; j < Nt; ++j) {
    if (trk_used[j]) continue;
    Track &t = g_tracks[j];
    t.misses++;
    t.hits = 0; // reset consecutive hit counter
    t.last_stamp = stamp;
  }

  // Create new tentative tracks from unmatched detections
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

  // Remove stale tracks
  g_tracks.erase(std::remove_if(g_tracks.begin(), g_tracks.end(),
                [](const Track &t){ return t.misses > P.max_misses; }),
                g_tracks.end());

  // Publish confirmed tracks as ConeArray in base frame (range/bearing from base)
  qcar_visnav::ConeArray out;
  out.header.stamp = stamp;
  out.header.frame_id = P.base_frame;

  // We need odom->base transform to compute base-relative polar & rotate covariance
  const double yaw_b = yaw_odom_to_base; // rotation from odom into base
  Eigen::Matrix2d R_ob = Rot2(yaw_b);    // rot odom->base
  Eigen::Vector2d t_ob(tf_odom_base.transform.translation.x,
                       tf_odom_base.transform.translation.y);

  size_t n_pub = 0;
  for (const auto &t : g_tracks) {
    if (!t.confirmed) continue;

    // point in base: p_base = R_ob * (p_odom) + t_ob
    Eigen::Vector2d p_base = R_ob * t.x + t_ob;

    // covariance in base: S_base = R_ob * P * R_ob^T
    Eigen::Matrix2d S_base = R_ob * t.P * R_ob.transpose();

    const double r = std::hypot(p_base.x(), p_base.y());
    const double th = std::atan2(p_base.y(), p_base.x());
    Eigen::Matrix2d S_polar = cartCovToPolar(p_base, S_base);

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
      "[cone_tracker] det=%d matched=%zu new=%d active=%zu confirmed_published=%zu",
      Nd, pairs.size(), Nd - (int)pairs.size(), g_tracks.size(), n_pub);
  }

  pub_tracks.publish(out);
}

// ============================
// Main
// ============================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // params
  pnh.param("detections_topic", P.detections_topic, P.detections_topic);
  pnh.param("tracks_topic", P.tracks_topic, P.tracks_topic);
  pnh.param("odom_frame", P.odom_frame, P.odom_frame);
  pnh.param("base_frame", P.base_frame, P.base_frame);
  pnh.param("gate_chi2", P.chi2_gate, P.chi2_gate);
  pnh.param("init_hits", P.init_hits, P.init_hits);
  pnh.param("max_misses", P.max_misses, P.max_misses);
  pnh.param("init_cov", P.init_cov, P.init_cov);
  pnh.param("q_xy", P.q_xy, P.q_xy);
  pnh.param("r_scale", P.r_scale, P.r_scale);
  pnh.param("max_dt", P.max_dt, P.max_dt);
  pnh.param("use_hungarian", P.use_hungarian, P.use_hungarian);
  pnh.param("debug", P.debug, P.debug);

  tf_buffer.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
  tf_listener.reset(new tf2_ros::TransformListener(*tf_buffer));

  pub_tracks = nh.advertise<qcar_visnav::ConeArray>(P.tracks_topic, 1, false);
  sub_dets   = nh.subscribe<qcar_visnav::ConeArray>(P.detections_topic, 1, &conesCb);

  ROS_INFO("[cone_tracker] up. subs='%s' -> pubs='%s', frames: odom='%s' base='%s'",
           P.detections_topic.c_str(), P.tracks_topic.c_str(),
           P.odom_frame.c_str(), P.base_frame.c_str());

  ros::spin();
  return 0;
}
