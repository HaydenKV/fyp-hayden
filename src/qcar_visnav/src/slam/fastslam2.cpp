#include "qcar_visnav/slam/fastslam2.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <Eigen/Dense>

// TF & msgs
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/measurement_model.h"

using namespace qcar_visnav::slam;

// ============================================================================
// Helpers for pose proposal prior, likelihoods, and sampling
// ============================================================================

namespace {
// Q for pose prior over dt using diffusive motion noise
inline Eigen::Matrix3d Qx_from_noise_dt(const MotionNoise& n, double dt) {
  Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
  const double s = std::max(0.0, dt);
  Q(0,0) = n.sigma_x   * n.sigma_x   * s;
  Q(1,1) = n.sigma_y   * n.sigma_y   * s;
  Q(2,2) = n.sigma_yaw * n.sigma_yaw * s;
  return Q;
}

inline double logGaussian(const Eigen::VectorXd& x,
                          const Eigen::VectorXd& mu,
                          const Eigen::MatrixXd& Sigma)
{
  const int d = static_cast<int>(x.size());
  Eigen::LLT<Eigen::MatrixXd> llt(Sigma);
  if (llt.info() != Eigen::Success) {
    Eigen::MatrixXd Sj = Sigma;
    Sj += 1e-9 * Eigen::MatrixXd::Identity(d, d);
    llt.compute(Sj);
  }
  const Eigen::MatrixXd L = llt.matrixL();
  const double logdet = 2.0 * L.diagonal().array().log().sum();
  const Eigen::VectorXd y = llt.solve(x - mu);
  const double quad = (x - mu).dot(y);
  return -0.5 * (quad + logdet + d * std::log(2.0 * M_PI));
}

inline Eigen::VectorXd sampleGaussian(const Eigen::VectorXd& mu,
                                      const Eigen::MatrixXd& Sigma,
                                      std::mt19937& rng)
{
  const int d = static_cast<int>(mu.size());
  Eigen::LLT<Eigen::MatrixXd> llt(Sigma);
  if (llt.info() != Eigen::Success) {
    // Not PD -> deterministic fallback
    return mu;
  }
  const Eigen::MatrixXd L = llt.matrixL();
  std::normal_distribution<double> N01(0.0, 1.0);
  Eigen::VectorXd z(d);
  for (int i = 0; i < d; ++i) z(i) = N01(rng);
  return mu + L * z;
}
} // anon namespace

// ============================================================================
// Small geometry helpers
// ============================================================================

bool FastSLAM2::inFoV(const Particle& p,
                      const Eigen::Vector2d& mu_world,
                      const LidarExtrinsics& ex) const
{
  // Lidar position in world
  const double cy = std::cos(p.yaw), sy = std::sin(p.yaw);
  const double lx_w = p.x + cy*ex.x - sy*ex.y;
  const double ly_w = p.y + sy*ex.x + cy*ex.y;

  // Vector world->world
  const double dx = mu_world.x() - lx_w;
  const double dy = mu_world.y() - ly_w;

  // Rotate world->lidar: R(yaw+ex.yaw)^T
  const double yaw_lw = p.yaw + ex.yaw;
  const double c = std::cos(yaw_lw), s = std::sin(yaw_lw);
  const double lx =  c*dx + s*dy;
  const double ly = -s*dx + c*dy;

  const double r = std::hypot(lx, ly);
  const double b = std::atan2(ly, lx);
  return (r <= fov_range_max_ && std::fabs(b) <= fov_bearing_rad_);
}

double FastSLAM2::worldMaha2(const Eigen::Vector2d& z_world,
                             const Eigen::Matrix2d& Rw,
                             const Landmark& lm) const
{
  const Eigen::Matrix2d Sw   = lm.Sigma + Rw;
  const Eigen::Matrix2d Sinv = Sw.inverse();
  const Eigen::Vector2d d    = z_world - lm.mu;
  return (d.transpose() * Sinv * d)(0,0);
}

// Build greedy one-to-one matches (RB gating & cost = Mahalanobis^2)
void FastSLAM2::buildGreedyMatches(const Particle& p,
                                   const std::vector<MeasRB>& meas_vec,
                                   const LidarExtrinsics& ex,
                                   std::vector<MatchPair>& out_matches,
                                   std::vector<int>& out_unmatched_meas) const
{
  struct Cand {
    int k, j; double d2;
    Eigen::Matrix2d S, H;
    Eigen::Matrix<double,2,3> Gx;
    Eigen::Vector2d nu;
    Eigen::Matrix2d R;
  };
  std::vector<Cand> cands;
  cands.reserve(meas_vec.size() * std::max<size_t>(1, p.map.size()));

  // Build all gated candidates
  for (int k = 0; k < (int)meas_vec.size(); ++k) {
    const auto& m = meas_vec[k];
    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = m.r_var; R(1,1) = m.b_var;

    for (int j = 0; j < (int)p.map.size(); ++j) {
      const auto& lm = p.map[j];

      double r_h, b_h; Eigen::Matrix2d H;
      Eigen::Matrix<double,2,3> Gx;
      predictRBWithJacobians(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, Gx, H);

      Eigen::Vector2d nu; nu << (m.r - r_h), wrapToPi(m.b - b_h);
      const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + R;
      const Eigen::Matrix2d Sinv = S.inverse();
      const double d2 = (nu.transpose() * Sinv * nu)(0,0);

      if (d2 <= chi2_gate_) {
        cands.push_back({k, j, d2, S, H, Gx, nu, R});
      }
    }
  }

  // Greedy: sort by cost asc and pick non-conflicting pairs
  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b){ return a.d2 < b.d2; });
  std::vector<char> k_used(meas_vec.size(), 0), j_used(p.map.size(), 0);
  out_matches.clear();

  for (const auto& c : cands) {
    if (k_used[c.k]) continue;
    if (c.j < (int)j_used.size() && j_used[c.j]) continue;
    MatchPair mp;
    mp.k = c.k; mp.j = c.j; mp.d2 = c.d2;
    mp.S = c.S; mp.Hlm = c.H; mp.Gx = c.Gx;
    mp.nu = c.nu; mp.R = c.R;
    out_matches.push_back(mp);
    k_used[c.k] = 1; if (c.j < (int)j_used.size()) j_used[c.j] = 1;
  }

  // Collect unmatched measurements
  out_unmatched_meas.clear();
  for (int k = 0; k < (int)meas_vec.size(); ++k)
    if (!k_used[k]) out_unmatched_meas.push_back(k);
}

// ============================================================================
// Constructor / Configuration
// ============================================================================

FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise())
{
  ROS_INFO("[FastSLAM2] Initializing FastSLAM 2.0...");

  // ---- Frames ----
  pnh.param("frames/map_frame",  map_frame_,  std::string("odom"));
  pnh.param("frames/odom_frame", odom_frame_, std::string("odom"));
  pnh.param("frames/base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("frames/lidar_frame", lidar_frame_, std::string("lidar"));

  // ---- Odometry source ----
  bool use_ekf_odom = false;
  std::string truth_topic = "/odom";
  std::string ekf_topic   = "/qcar/ekf/odom";
  pnh.param("odometry/use_ekf", use_ekf_odom, false);
  pnh.param("odometry/truth_topic", truth_topic, truth_topic);
  pnh.param("odometry/ekf_topic",   ekf_topic,   ekf_topic);
  odom_topic_ = use_ekf_odom ? ekf_topic : truth_topic;
  ROS_INFO("[FastSLAM2] Using odometry from: %s", odom_topic_.c_str());

  // ---- Core PF params ----
  pnh.param("particles", N_, 120);
  pnh.param("resample_neff_ratio", neff_ratio_, 0.4);
  pnh.param("association/chi2_gate", chi2_gate_, 9.21);
  pnh.param("association/chi2_gate_world", chi2_gate_world_, 9.21);
  pnh.param("association/euclid_gate", euclid_gate_, 0.5); // reserved

  // ---- Landmark management ----
  pnh.param("confirm_hits",    confirm_hits_,    2);
  pnh.param("min_new_lm_dist", min_new_lm_dist_, 0.35);
  pnh.param("merge_R_scale",   merge_R_scale_,   4.0);
  pnh.param("landmark_prior_var", landmark_prior_var_, 0.25);
  pnh.param("landmarks/unconfirmed_R_scale",      unconfirmed_R_scale_,      2.0);
  pnh.param("landmarks/prune_unconfirmed_misses", prune_unconfirmed_misses_, 8);  // reserved
  pnh.param("landmarks/prune_stale_misses",       prune_stale_misses_,       25); // reserved

  // ---- Proposal, births, penalties, FoV ----
  pnh.param("proposal/max_sigma_trace", proposal_max_sigma_trace_, 0.20);
  pnh.param("proposal/sample_every_k",  proposal_sample_every_k_,  0);
  pnh.param("birth/required_hits", birth_required_hits_, 3);
  pnh.param("birth/max_age",       birth_max_age_,       10);
  pnh.param("birth/promote_radius",birth_promote_radius_,0.30);
  pnh.param("weight/new_landmark_penalty", new_landmark_penalty_, 1.0);
  pnh.param("weight/fov_miss_penalty",     fov_miss_penalty_,     0.4);
  pnh.param("sensor/fov_range_max",        fov_range_max_,        20.0);
  double fov_bearing_deg = 90.0;
  pnh.param("sensor/fov_bearing_deg",      fov_bearing_deg,       90.0);
  fov_bearing_rad_ = fov_bearing_deg * M_PI / 180.0;
  pnh.param("lap/freeze_after_first_loop", freeze_after_first_loop_, false);

  // ---- Initialization behavior ----
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, true);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.02);
  pnh.param("initial_spread/y",   spread_y_,   0.02);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.01);

  // ---- Odom buffer window ----
  pnh.param("odom_buffer_window_sec", odom_buffer_window_sec_, 2.0);

  // ---- Motion model ----
  MotionNoise q;
  pnh.param("motion_noise/sigma_x",   q.sigma_x,   0.02);
  pnh.param("motion_noise/sigma_y",   q.sigma_y,   0.02);
  pnh.param("motion_noise/sigma_yaw", q.sigma_yaw, 0.005);
  motion_ = MotionModel(q);
  motion_noise_ = q;

  // ---- Particles ----
  P_.resize(N_);
  const double w0 = 1.0 / std::max(1, N_);
  for (int i = 0; i < N_; ++i) {
    P_[i].x = 0.0; P_[i].y = 0.0; P_[i].yaw = 0.0;
    P_[i].weight = w0;
    P_[i].log_w  = std::log(w0);
    P_[i].id = i;
    P_[i].map.clear();
    P_[i].births.clear();
  }

  if (seed_from_params_) {
    ROS_INFO("[FastSLAM2] Seeding from params: init(%.3f, %.3f, %.3f deg), spread(%.3f, %.3f, %.3f deg)",
             init_x_, init_y_, init_yaw_*180/M_PI, spread_x_, spread_y_, spread_yaw_);
    initializeParticlesFrom(init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
  } else {
    ROS_INFO("[FastSLAM2] Will initialize from first odometry message.");
  }

  // ---- ROS pubs/subs ----
  pub_particles_            = nh.advertise<visualization_msgs::MarkerArray>("slam/particles", 1);
  pub_particles_posearray_  = nh.advertise<geometry_msgs::PoseArray>("slam/particles_pose", 1);
  pub_map_markers_          = nh.advertise<visualization_msgs::MarkerArray>("slam/map_markers", 1);
  pub_slam_odom_            = nh.advertise<nav_msgs::Odometry>("slam/odom", 1);
  pub_slam_odom_mean_       = nh.advertise<nav_msgs::Odometry>("slam/odom_mean", 1);

  sub_odom_  = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 50, &FastSLAM2::cbOdom, this);
  sub_cones_ = nh.subscribe<qcar_visnav::ConeArray>("/tracked_cones", 10, &FastSLAM2::cbCones, this);

  ROS_INFO("[FastSLAM2] Config:");
  ROS_INFO("  Assoc: chi2_rb=%.2f chi2_world=%.2f euclid_gate=%.2f",
           chi2_gate_, chi2_gate_world_, euclid_gate_);
  ROS_INFO("  Birth: K=%d age=%d promote_r=%.2f",
           birth_required_hits_, birth_max_age_, birth_promote_radius_);
  ROS_INFO("  Proposal: max_trace=%.3f sample_every_k=%d",
           proposal_max_sigma_trace_, proposal_sample_every_k_);
  ROS_INFO("  Penalties: new=%.2f fov_miss=%.2f",
           new_landmark_penalty_, fov_miss_penalty_);
  ROS_INFO("  FoV: range=%.1f bearing=%.1f deg",
           fov_range_max_, fov_bearing_rad_*180/M_PI);
}

// ============================================================================
// Particle initialization
// ============================================================================

void FastSLAM2::initializeParticlesFrom(double x, double y, double yaw,
                                        double sx, double sy, double syaw)
{
  ROS_INFO("[FastSLAM2] Initializing %d particles around (%.3f, %.3f, %.1f deg)",
           N_, x, y, yaw*180/M_PI);

  std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> nx(0.0, sx);
  std::normal_distribution<double> ny(0.0, sy);
  std::normal_distribution<double> nyw(0.0, syaw);

  for (auto& part : P_) {
    part.x = x + nx(rng);
    part.y = y + ny(rng);
    part.yaw = yaw + nyw(rng);
  }

  particles_initialized_ = true;
  last_prop_stamp_ = ros::Time(0);

  best_idx_ = 0;
  mean_x_ = x; mean_y_ = y; mean_yaw_ = yaw;

  ROS_INFO("[FastSLAM2] Particles initialized.");
}

// ============================================================================
// Event loop placeholder (kept for future periodic tasks)
// ============================================================================

void FastSLAM2::spinOnce() {
  // no-op
}

// ============================================================================
// Odom callback: push body-frame twist; optional snap-to-first pose
// ============================================================================

void FastSLAM2::cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!snapped_to_first_odom_ && overwrite_with_odom_on_first_msg_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
    snapped_to_first_odom_ = true;
    ROS_INFO("[FastSLAM2] Snapped to first /odom: (%.3f, %.3f, %.1f deg)",
             p.x, p.y, yaw*180/M_PI);
  }

  if (!particles_initialized_ && !seed_from_params_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
  }

  // Body-frame twist sample
  OdomStamped o;
  o.t  = msg->header.stamp;
  o.vx = msg->twist.twist.linear.x;
  o.vy = msg->twist.twist.linear.y;
  o.r  = msg->twist.twist.angular.z;
  odom_buf_.push_back(o);

  // Ring buffer
  while (!odom_buf_.empty() &&
         (o.t - odom_buf_.front().t).toSec() > odom_buffer_window_sec_) {
    odom_buf_.pop_front();
  }
}

// ============================================================================
// Twist interpolation and propagation
// ============================================================================

FastSLAM2::OdomStamped
FastSLAM2::interpTwist(const OdomStamped& a,
                       const OdomStamped& b,
                       const ros::Time& s) const
{
  const double seg = (b.t - a.t).toSec();
  if (seg <= 0.0) return a;
  const double frac = (s - a.t).toSec() / seg;
  OdomStamped result;
  result.t  = s;
  result.vx = (1.0 - frac)*a.vx + frac*b.vx;
  result.vy = (1.0 - frac)*a.vy + frac*b.vy;
  result.r  = (1.0 - frac)*a.r  + frac*b.r;
  return result;
}

void FastSLAM2::propagateParticlesTo(const ros::Time& t) {
  if (!particles_initialized_ || odom_buf_.size() < 2) return;

  if (last_prop_stamp_.isZero()) last_prop_stamp_ = odom_buf_.front().t;
  if (t <= last_prop_stamp_) return;

  for (size_t i = 0; i + 1 < odom_buf_.size(); ++i) {
    const auto& a = odom_buf_[i];
    const auto& b = odom_buf_[i+1];

    // Overlap of [a.t, b.t] with [last_prop_stamp_, t]
    const ros::Time seg_start = std::max(a.t, last_prop_stamp_);
    const ros::Time seg_end   = std::min(b.t, t);
    if (seg_end <= seg_start) continue;

    // Interpolate and integrate (trapezoidal)
    const auto ta = interpTwist(a, b, seg_start);
    const auto tb = interpTwist(a, b, seg_end);

    const double dt = (seg_end - seg_start).toSec();
    const double vx_avg = 0.5*(ta.vx + tb.vx);
    const double vy_avg = 0.5*(ta.vy + tb.vy);
    const double r_avg  = 0.5*(ta.r  + tb.r);

    for (auto& p : P_) {
      motion_.propagate(p, vx_avg, vy_avg, r_avg, dt);
    }
  }

  last_prop_stamp_ = t;
}

// ============================================================================
// Cone observations callback (main SLAM update from tracked cones)
// ============================================================================

bool FastSLAM2::lookupLidarExtrinsics(const ros::Time& t, LidarExtrinsics& ex) const {
  try {
    const auto T = tfbuf_.lookupTransform(base_frame_, lidar_frame_, t, ros::Duration(0.05));
    ex.x = T.transform.translation.x;
    ex.y = T.transform.translation.y;
    tf2::Quaternion q; tf2::fromMsg(T.transform.rotation, q);
    double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ex.yaw = yaw;
    ex.valid = true;
    return true;
  } catch (const tf2::TransformException& e) {
    ROS_WARN_THROTTLE(1.0, "[FastSLAM2] TF %s->%s at t=%.3f failed: %s",
                      base_frame_.c_str(), lidar_frame_.c_str(), t.toSec(), e.what());
    ex.valid = false;
    return false;
  }
}

void FastSLAM2::cbCones(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!particles_initialized_) {
    ROS_WARN_THROTTLE(2.0, "[FastSLAM2] Cones received before particles initialized; ignoring");
    return;
  }

  // 1) Propagate to observation time
  propagateParticlesTo(msg->header.stamp);

  // 2) LiDAR extrinsics at this time
  LidarExtrinsics ex;
  if (!lookupLidarExtrinsics(msg->header.stamp, ex) || !ex.valid) return;

  // 3) Build RB measurements from tracked cones (LiDAR frame)
  std::vector<MeasRB> meas; meas.reserve(msg->cones.size());
  for (const auto& c : msg->cones) {
    if (!std::isfinite(c.range) || !std::isfinite(c.bearing)) continue;
    MeasRB m;
    m.r     = c.range;
    m.b     = c.bearing;
    m.r_var = std::max(1e-10, c.r_var);
    m.b_var = std::max(1e-12, c.bearing_var);
    meas.push_back(m);
  }
  if (meas.empty()) {
    ROS_DEBUG("[FastSLAM2] No valid cone observations after RB filtering");
    return;
  }

  // 4) Core SLAM update
  processMeasurementAt(msg->header.stamp, meas, ex);

  // 5) Visualization/output
  publishViz(msg->header.stamp);
}

// ============================================================================
// Core measurement update (FastSLAM 2.0, RB model)
// ============================================================================

void FastSLAM2::processMeasurementAt(const ros::Time& t,
                                     const std::vector<MeasRB>& meas_vec,
                                     const LidarExtrinsics& ex)
{
  // Pose prior over dt
  double dt = 0.01;
  if (!last_prop_stamp_.isZero()) dt = std::max(0.0, (t - last_prop_stamp_).toSec());

  for (auto& p : P_) {
    // 1) Exclusive meas↔LM matches (RB gating, per particle)
    std::vector<MatchPair> matches;
    std::vector<int> unmatched_meas;
    buildGreedyMatches(p, meas_vec, ex, matches, unmatched_meas);

    // 2) FS2.0 pose proposal from *stable* matches only
    Eigen::Vector3d mu_prior(p.x, p.y, p.yaw);
    Eigen::Matrix3d Qprior = Qx_from_noise_dt(motion_noise_, dt);
    Qprior += 1e-9 * Eigen::Matrix3d::Identity(); // PD floor

    Eigen::Matrix3d  Lambda = Qprior.inverse();
    Eigen::Vector3d  eta    = Lambda * mu_prior;

    double log_det_S_norm_sum = 0.0;
    int num_pose_meas = 0;

    for (const auto& mp : matches) {
      const auto& lm = p.map[mp.j];
      const bool stable = lm.confirmed && (lm.Sigma.trace() <= proposal_max_sigma_trace_);
      if (!stable) continue;

      const Eigen::Matrix2d S_inv = mp.S.inverse();
      const Eigen::Matrix<double,3,2> Gt = mp.Gx.transpose();
      Lambda += Gt * S_inv * mp.Gx;
      eta    += Gt * S_inv * mp.nu;

      log_det_S_norm_sum += std::log(std::max(1e-18, mp.S.determinant()));
      ++num_pose_meas;
    }

    Eigen::Matrix3d  Sigma_q = Lambda.inverse();
    Eigen::Vector3d  mu_q    = Sigma_q * eta;

    // Mean proposal by default; optionally sample sometimes
    Eigen::Vector3d x_samp = mu_q;
    if (proposal_sample_every_k_ > 0) {
      static uint64_t frame_idx = 0;
      if ((frame_idx++ % proposal_sample_every_k_) == 0) {
        x_samp = sampleGaussian(mu_q, Sigma_q, rng_);
      }
    }

    p.x = x_samp(0);
    p.y = x_samp(1);
    p.yaw = wrapToPi(x_samp(2));

    // Importance weight correction: p_motion / q
    const double log_p_motion = logGaussian(x_samp, mu_prior, Qprior);
    const double log_q        = logGaussian(x_samp, mu_q,    Sigma_q);
    const double log_lik_norm =
      (num_pose_meas > 0) ? (-0.5 * log_det_S_norm_sum - 0.5 * num_pose_meas * std::log(2*M_PI)) : 0.0;
    p.log_w += (log_p_motion - log_q + log_lik_norm);

    // Track LMs seen this frame (for FoV-miss penalty)
    std::vector<char> lm_seen(p.map.size(), 0);

    // 3) EKF updates for matched landmarks (at proposed pose)
    for (const auto& mp : matches) {
      auto& lm = p.map[mp.j];

      Eigen::Matrix2d R_eff = (!lm.confirmed) ? (mp.R * unconfirmed_R_scale_) : mp.R;

      double r_h, b_h; Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
      Eigen::Vector2d nu; nu << (meas_vec[mp.k].r - r_h), wrapToPi(meas_vec[mp.k].b - b_h);

      const Eigen::Matrix2d S    = H * lm.Sigma * H.transpose() + R_eff;
      const Eigen::Matrix2d Sinv = S.inverse();
      const Eigen::Matrix2d K    = lm.Sigma * H.transpose() * Sinv;

      lm.mu    = lm.mu + K * nu;
      lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;

      lm.hits++;
      if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

      lm_seen[mp.j] = 1;
    }

    // 4) Unmatched measurements: world-merge BEFORE births
    int promotions_this_frame = 0;

    for (int k : unmatched_meas) {
      const auto& m = meas_vec[k];

      // world z and covariance Rw
      Eigen::Vector2d z_world; Eigen::Matrix2d Jrb;
      measRBToWorld(p.x, p.y, p.yaw, ex, m.r, m.b, z_world, Jrb);
      const Eigen::Matrix2d Rw = Jrb * (Eigen::Matrix2d() << m.r_var, 0, 0, m.b_var).finished()
                               * Jrb.transpose();

      // Try world-space merge into existing LM (χ² in world)
      int best_j = -1; double best_d2 = std::numeric_limits<double>::infinity();
      for (int j = 0; j < (int)p.map.size(); ++j) {
        const double d2 = worldMaha2(z_world, Rw, p.map[j]);
        if (d2 < best_d2) { best_d2 = d2; best_j = j; }
      }

      if (best_j >= 0 && best_d2 <= chi2_gate_world_) {
        auto& lm = p.map[best_j];

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R(0,0)=m.r_var; R(1,1)=m.b_var;
        Eigen::Matrix2d R_eff = (!lm.confirmed) ? (R * unconfirmed_R_scale_) : R;

        double r_h, b_h; Eigen::Matrix2d H;
        predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
        Eigen::Vector2d nu; nu << (m.r - r_h), wrapToPi(m.b - b_h);

        const Eigen::Matrix2d S    = H * lm.Sigma * H.transpose() + R_eff;
        const Eigen::Matrix2d Sinv = S.inverse();
        const Eigen::Matrix2d K    = lm.Sigma * H.transpose() * Sinv;

        lm.mu    = lm.mu + K * nu;
        lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;
        lm.hits++; if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

        lm_seen[best_j] = 1;
        continue;
      }

      // Otherwise: update/start a birth track in world (H = I)
      int best_bt = -1; double best_d2_euclid = std::numeric_limits<double>::infinity();
      for (int bti = 0; bti < (int)p.births.size(); ++bti) {
        const double d2 = (z_world - p.births[bti].mu).squaredNorm();
        if (d2 < best_d2_euclid) { best_d2_euclid = d2; best_bt = bti; }
      }

      if (best_bt >= 0 && std::sqrt(best_d2_euclid) <= birth_promote_radius_) {
        auto& bt = p.births[best_bt];
        const Eigen::Matrix2d S = bt.Sigma + Rw;
        const Eigen::Matrix2d K = bt.Sigma * S.inverse();
        bt.mu    = bt.mu + K * (z_world - bt.mu);
        bt.Sigma = (Eigen::Matrix2d::Identity() - K) * bt.Sigma;
        bt.hits++;
        bt.age = 0;
      } else {
        BirthTrack bt;
        bt.mu = z_world;
        bt.Sigma = Rw + 0.01 * Eigen::Matrix2d::Identity();
        bt.hits = 1;
        bt.age  = 0;
        p.births.push_back(bt);
      }
    }

    // Age & promote births; drop old ones
    for (auto& bt : p.births) bt.age++;
    for (int bti = (int)p.births.size()-1; bti >= 0; --bti) {
      auto& bt = p.births[bti];
      if (bt.hits >= birth_required_hits_) {
        bool near_lm = false;
        for (const auto& lm : p.map) {
          if ((bt.mu - lm.mu).norm() <= birth_promote_radius_) { near_lm = true; break; }
        }
        if (!near_lm && mapping_enabled_) {
          Landmark lm;
          lm.mu = bt.mu;
          lm.Sigma = bt.Sigma + landmark_prior_var_ * Eigen::Matrix2d::Identity();
          lm.hits = 1; lm.misses = 0;
          lm.confirmed = (confirm_hits_ <= 1);
          p.map.push_back(lm);
          promotions_this_frame++;
          p.births.erase(p.births.begin() + bti);
          continue;
        }
      }
      if (bt.age > birth_max_age_) {
        p.births.erase(p.births.begin() + bti);
      }
    }

    // 5) Weight penalties: discourage exploding maps or un-explained FoV
    if (promotions_this_frame > 0) {
      p.log_w -= new_landmark_penalty_ * promotions_this_frame;
    }
    int fov_miss_cnt = 0;
    for (int j = 0; j < (int)p.map.size(); ++j) {
      const auto& lm = p.map[j];
      if (!lm.confirmed) continue;
      if (!inFoV(p, lm.mu, ex)) continue;
      if (j < (int)lm_seen.size() && lm_seen[j]) continue;
      fov_miss_cnt++;
    }
    if (fov_miss_cnt > 0) {
      p.log_w -= fov_miss_penalty_ * fov_miss_cnt;
    }
  } // for each particle

  // 6) Normalize weights (log-sum-exp)
  double max_logw = -1e300;
  for (const auto& p : P_) max_logw = std::max(max_logw, p.log_w);

  double sum_w = 0.0;
  for (auto& p : P_) { p.weight = std::exp(p.log_w - max_logw); sum_w += p.weight; }
  if (sum_w <= 0.0) {
    const double w0 = 1.0 / std::max<int>(1, P_.size());
    for (auto& p : P_) p.weight = w0;
  } else {
    for (auto& p : P_) p.weight /= sum_w;
  }

  // 7) Adaptive resampling
  double inv_neff = 0.0; for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;

  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_, rng_);
    ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Resampled: Neff=%.1f < %.1f",
                      neff, neff_ratio_ * P_.size());
    for (auto& p : P_) p.log_w = std::log(std::max(1e-300, p.weight));
  }

  // 8) Cache best & mean
  best_idx_ = 0; double bestw = -1.0;
  for (int i = 0; i < (int)P_.size(); ++i)
    if (P_[i].weight > bestw) { bestw = P_[i].weight; best_idx_ = i; }

  double cx=0, cy=0, cyaw_c=0, cyaw_s=0;
  for (const auto& p : P_) {
    cx += p.weight * p.x; cy += p.weight * p.y;
    cyaw_c += p.weight * std::cos(p.yaw);
    cyaw_s += p.weight * std::sin(p.yaw);
  }
  mean_x_ = cx; mean_y_ = cy; mean_yaw_ = std::atan2(cyaw_s, cyaw_c);
}

// ============================================================================
// Visualization
// ============================================================================

void FastSLAM2::publishViz(const ros::Time& t) {
  // Particles (arrows)
  visualization_msgs::MarkerArray particle_markers;
  int id = 0;
  for (const auto& p : P_) {
    visualization_msgs::Marker m;
    m.header.stamp = t;
    m.header.frame_id = map_frame_;
    m.ns = "particles";
    m.id = id++;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x; m.pose.position.y = p.y; m.pose.position.z = 0.05;
    tf2::Quaternion q; q.setRPY(0, 0, p.yaw);
    m.pose.orientation = tf2::toMsg(q);
    m.scale.x = 0.3; m.scale.y = 0.05; m.scale.z = 0.05;
    m.color.a = 0.3 + 0.7 * (p.weight * P_.size());
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;
    particle_markers.markers.push_back(m);
  }
  pub_particles_.publish(particle_markers);

  // Particle pose array (for easy RViz display)
  geometry_msgs::PoseArray poses;
  poses.header.stamp = t;
  poses.header.frame_id = map_frame_;
  for (const auto& p : P_) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x; pose.position.y = p.y; pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,p.yaw); pose.orientation = tf2::toMsg(q);
    poses.poses.push_back(pose);
  }
  pub_particles_posearray_.publish(poses);

  // Landmarks from best particle (show all)
  visualization_msgs::MarkerArray landmark_markers;
  const auto& bestP = P_[std::max(0, std::min<int>(best_idx_, (int)P_.size()-1))];
  int lm_id = 0;
  for (const auto& lm : bestP.map) {
    visualization_msgs::Marker m;
    m.header.stamp = t;
    m.header.frame_id = map_frame_;
    m.ns = "landmarks";
    m.id = lm_id++;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = lm.mu.x();
    m.pose.position.y = lm.mu.y();
    m.pose.position.z = 0.05;
    m.scale.x = m.scale.y = m.scale.z = 0.18;
    m.color.a = 1.0; m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;
    landmark_markers.markers.push_back(m);
  }
  pub_map_markers_.publish(landmark_markers);

  // Odometry estimates (best and mean)
  const auto& bp = bestP;
  nav_msgs::Odometry odom_best;
  odom_best.header.stamp = t;
  odom_best.header.frame_id = map_frame_;
  odom_best.child_frame_id  = base_frame_;
  odom_best.pose.pose.position.x = bp.x;
  odom_best.pose.pose.position.y = bp.y;
  odom_best.pose.pose.position.z = 0.0;
  tf2::Quaternion q_best; q_best.setRPY(0,0,bp.yaw);
  odom_best.pose.pose.orientation = tf2::toMsg(q_best);
  pub_slam_odom_.publish(odom_best);

  nav_msgs::Odometry odom_mean = odom_best;
  odom_mean.pose.pose.position.x = mean_x_;
  odom_mean.pose.pose.position.y = mean_y_;
  tf2::Quaternion q_mean; q_mean.setRPY(0,0,mean_yaw_);
  odom_mean.pose.pose.orientation = tf2::toMsg(q_mean);
  pub_slam_odom_mean_.publish(odom_mean);

  ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Published viz: particles=%zu landmarks=%zu (best=%d)",
                    P_.size(), bestP.map.size(), best_idx_);
}
