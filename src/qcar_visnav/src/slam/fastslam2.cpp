/* =====================================================================================
   FastSLAM 2.0
   =====================================================================================

   INPUTS (ROS)
   ------------
   • Odometry: nav_msgs/Odometry from either /odom or /qcar/ekf/odom
   • Tracked cones: qcar_visnav::ConeArray on /tracked_cones (range, bearing, variances, id)
   • TF: transform from base_frame → lidar_frame at measurement time
   • YAML params: frames, topics, PF sizes, noises, association mode, proposal knobs, etc.

   OUTPUTS (ROS)
   -------------
   • /slam/particles_pose          : geometry_msgs::PoseArray of all particles
   • /slam/landmarks_pose          : PoseArray of all landmarks (best particle)
   • /slam/landmarks_pose_confirmed: PoseArray of confirmed landmarks
   • /slam/landmarks_pose_locked   : PoseArray of “locked” landmarks
   • /slam/unmatched_pose          : PoseArray of current frame’s unmatched detections (world)
   • /slam/odom                    : nav_msgs::Odometry (best particle pose)
   • /slam/odom_mean               : nav_msgs::Odometry (weight-averaged mean pose)

   PIPELINE (frame-by-frame)
   -------------------------
   1) Initialization
      - Load YAML (frames, topics, particle count, motion/measurement noise, DA mode, pose-proposal).
      - Seed particles near initial pose (or on first odom) with Gaussian spread.
   2) Odometry ingestion & propagation
      - Buffer body-frame twist samples (vx, vy, r, stamp) in a rolling window.
      - For a measurement at time t, interpolate twists and propagate every particle
        from last processed stamp → t with the motion model.
   3) Measurement callback (/tracked_cones)
      - Look up lidar extrinsics (x, y, yaw of lidar in base) from TF at the frame stamp.
      - Build a vector of range–bearing measurements {r, b, r_var, b_var, id}.
   4) Pose proposal (FastSLAM 2.0) — optional
      - For each particle:
        • Build provisional associations via either ID or nearest-neighbor in RB.
        • For each tentative match compute innovation ν and S = H Σ Hᵀ + R.
        • Keep top-K (lowest Mahalanobis d² = νᵀ S^{-1} ν) matches.
        • Linearize measurement wrt pose (finite differences) to get Hx.
        • Form information update:
              Q_prior = diag(σ_x², σ_y², σ_yaw²) * max(dt, min_dt) * prior_scale
              Λ = Q_prior^{-1} + Σ Hxᵀ S^{-1} Hx
              η = Σ Hxᵀ S^{-1} ν
          Solve δx = Λ^{-1} η (LDLT) and apply a clamped Gauss–Newton pose step.
   5) Landmark EKF + particle reweighting
      - For each measurement:
        • Associate (ID or NN in RB) with χ²(2) gate on d² = νᵀ S^{-1} ν.
        • If matched: accumulate exact log-likelihood
              log p(z|·) = −½(νᵀ S^{-1} ν + log|2π S|)
          and EKF-update landmark:
              K = Σ Hᵀ S^{-1}
              μ ← μ + K ν
              Σ ← (I − K H) Σ
          Track hits/confirmation/lock states.
        • If unmatched: birth a new landmark at world-projected (r, b) with covariance
              Σ_birth = J R Jᵀ + init_var·I
   6) Weight normalization & resampling
      - Convert log_w → w stably by subtracting max log_w, normalize.
      - Compute N_eff = 1 / Σ w_i²; if below threshold, systematic resample.
   7) Best/mean pose & publish
      - Best = argmax_i w_i; mean pose uses circular mean for yaw.
      - Publish particles, landmarks (all/confirmed/locked), unmatched (world), odom(best), odom(mean).

   YAML KEYS (mapping to code)
   ---------------------------
   frames.*                         : frame names for TF & publishing
   odometry.use_ekf / topics        : odom source selection
   particles, resample_neff_ratio   : PF size & resampling sensitivity
   motion_noise.{sigma_x,y,yaw}     : pose prior σ; used to build Q_prior
   association.{mode, chi2_gate_rb} : DA policy & χ² gate (e.g., 3.91=95%, 5.99=99%)
   landmarks.{init_var, confirm_hits, lock_hits, lock_cov_trace}
   pose_proposal.{enable, Kmax, eps_fd, clamp_dx, clamp_dy, clamp_dyaw, min_dt, prior_scale}
   seed_from_params, init.*, initial_spread.*, odom_buffer_window_sec

   ===================================================================================== */


#include "qcar_visnav/slam/fastslam2.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <limits>
#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/measurement_model.h"

using namespace qcar_visnav::slam;

// ================== file-local helpers ==================
namespace {

/* -----------------------------------------------------------------------------
wrapPi
WHAT: Normalize an angle to (-π, π].
WHY:  Keep bearings and yaws numerically stable and compatible with χ² gates and
      EKF linearizations.
HOW:  Repeatedly add/subtract 2π until inside the principal interval.
----------------------------------------------------------------------------- */
inline double wrapPi(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

/* -----------------------------------------------------------------------------
associateById
WHAT:  Associate a measurement to an existing landmark by persistent tracker ID.
WHY:   Upstream tracker gives high-quality IDs; direct association is cheap and
       robust if IDs are valid.
HOW:   Linear scan over particle's landmarks and return index with matching id;
       -1 if not found. (A χ² gate is still applied later before using it.)
----------------------------------------------------------------------------- */
inline int associateById(const Particle& p, const MeasRB& m) {
  if (m.id <= 0) return -1;
  for (int i = 0; i < (int)p.map.size(); ++i) {
    if (p.map[i].id == m.id) return i;
  }
  return -1;
}

/* -----------------------------------------------------------------------------
associateByNNRB
WHAT:  Nearest-neighbor data association in Range-Bearing space with a χ²(2) gate.
WHY:   Fallback when IDs are missing/incorrect; uses geometry + uncertainty.
HOW:   For each landmark:
        - Predict measurement h(x) = [r̂, b̂], compute innovation ν and S = H Σ Hᵀ + R.
        - Compute Mahalanobis d² = νᵀ S^{-1} ν.
       Keep the landmark with the smallest d² and accept if d² <= χ² gate; else -1.
NOTE:  Uses S^{-1} (not R^{-1}) so landmark uncertainty is respected.
----------------------------------------------------------------------------- */
inline int associateByNNRB(const Particle& p,
                           const MeasRB& m,
                           const LidarExtrinsics& ex,
                           double chi2_gate_rb)
{
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0,0) = std::max(1e-10, m.r_var);
  R(1,1) = std::max(1e-12, m.b_var);

  for (int i = 0; i < (int)p.map.size(); ++i) {
    const auto& lm = p.map[i];

    double r_hat=0, b_hat=0; Eigen::Matrix2d H;
    predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_hat, b_hat, H);

    Eigen::Vector2d nu; nu << (m.r - r_hat), wrapPi(m.b - b_hat);
    Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + R;

    Eigen::LLT<Eigen::Matrix2d> llt(S);
    if (llt.info() != Eigen::Success) continue;

    const Eigen::Matrix2d S_inv = llt.solve(Eigen::Matrix2d::Identity());
    const double d2 = (nu.transpose() * S_inv * nu)(0,0);

    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }

  if (best >= 0 && best_d2 <= chi2_gate_rb) return best;
  return -1;
}

/* -----------------------------------------------------------------------------
logGaussian
WHAT:  Log pdf of a multivariate Gaussian N(x | μ, Σ). Kept for completeness /
       debugging / potential scoring.
WHY:   Sometimes useful to inspect likelihoods explicitly.
HOW:   Cholesky on Σ for log|Σ| and solve for the quadratic form.
----------------------------------------------------------------------------- */
inline double logGaussian(const Eigen::VectorXd& x,
                          const Eigen::VectorXd& mu,
                          const Eigen::MatrixXd& Sigma)
{
  const int d = (int)x.size();
  Eigen::LLT<Eigen::MatrixXd> llt(Sigma);
  if (llt.info() != Eigen::Success) {
    Eigen::MatrixXd Sj = Sigma;
    Sj += 1e-9 * Eigen::MatrixXd::Identity(d, d);
    llt.compute(Sj);
  }
  const Eigen::MatrixXd L = llt.matrixL();
  double logdet = 0.0;
  for (int i = 0; i < d; ++i) logdet += std::log(std::max(1e-18, L(i,i)));
  logdet *= 2.0;

  const Eigen::VectorXd y = llt.solve(x - mu);
  const double quad = (x - mu).dot(y);
  return -0.5 * (quad + logdet + d * std::log(2.0 * M_PI));
}

/* -----------------------------------------------------------------------------
PosePropCfg (YAML)
WHAT:  Tunables for the FastSLAM 2.0 pose-proposal step.
WHY:   Control how aggressively the proposal uses measurement information and
       how the prior Q is built from motion noise and dt.
HOW:   Parameters are loaded via pnh.param(...) in the constructor.
----------------------------------------------------------------------------- */
struct PosePropCfg {
  bool   enable      = true;   // ~pose_proposal/enable
  int    Kmax        = 6;      // ~pose_proposal/Kmax
  double eps_fd      = 1e-4;   // ~pose_proposal/eps_fd
  double clamp_dx    = 0.25;   // ~pose_proposal/clamp_dx
  double clamp_dy    = 0.25;   // ~pose_proposal/clamp_dy
  double clamp_dyaw  = 0.05;   // ~pose_proposal/clamp_dyaw (rad)
  double min_dt      = 0.02;   // ~pose_proposal/min_dt (s) lower bound for dt
  double prior_scale = 1.0;    // ~pose_proposal/prior_scale multiplier on Q_prior
} PPC;

} // anon

// ================== ctor ==================
/* -----------------------------------------------------------------------------
FastSLAM2::FastSLAM2
WHAT:  Node setup & parameterization.
WHY:   Centralize configuration (frames, topics, particle count/noise, DA mode,
       landmark policy, proposal knobs), allocate particle set, wire ROS I/O.
HOW:
  - Load YAML params (frames, odom source, PF params, noises, association, etc.).
  - Initialize particles (either from params or first odom).
  - Advertise PoseArrays/Odometry; subscribe to odom & tracked cones.
----------------------------------------------------------------------------- */
FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise())
{
  ROS_INFO("[FastSLAM2] Initializing (lean) ...");

  // Frames
  pnh.param("frames/map_frame",  map_frame_,  std::string("odom"));
  pnh.param("frames/odom_frame", odom_frame_, std::string("odom"));
  pnh.param("frames/base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("frames/lidar_frame", lidar_frame_, std::string("lidar"));

  // Odom
  bool use_ekf_odom = false;
  std::string truth_topic = "/odom";
  std::string ekf_topic   = "/qcar/ekf/odom";
  pnh.param("odometry/use_ekf", use_ekf_odom, false);
  pnh.param("odometry/truth_topic", truth_topic, truth_topic);
  pnh.param("odometry/ekf_topic",   ekf_topic,   ekf_topic);
  odom_topic_ = use_ekf_odom ? ekf_topic : truth_topic;

  // PF
  pnh.param("particles", N_, 80);
  pnh.param("resample_neff_ratio", neff_ratio_, 0.5);

  // Association
  pnh.param("association/mode", assoc_mode_, std::string("id"));   // "id" | "nn_rb"
  pnh.param("association/chi2_gate_rb", chi2_gate_rb_, 5.99);

  // Landmark behavior
  pnh.param("landmarks/init_var",        lm_init_var_,        0.15);
  pnh.param("landmarks/confirm_hits",    lm_confirm_hits_,    2);
  pnh.param("landmarks/lock_hits",       lm_lock_hits_,       6);
  pnh.param("landmarks/lock_cov_trace",  lm_lock_cov_trace_,  0.02);

  // Motion model params (constant white noise)
  MotionNoise mn;
  pnh.param("motion_noise/sigma_x",   mn.sigma_x,   0.04);
  pnh.param("motion_noise/sigma_y",   mn.sigma_y,   0.04);
  pnh.param("motion_noise/sigma_yaw", mn.sigma_yaw, 0.01);
  motion_ = MotionModel(mn);
  motion_noise_ = mn;

  // --- pose proposal (YAML: ~pose_proposal/...) ---
  pnh.param("pose_proposal/enable",      PPC.enable,      PPC.enable);
  pnh.param("pose_proposal/Kmax",        PPC.Kmax,        PPC.Kmax);
  pnh.param("pose_proposal/eps_fd",      PPC.eps_fd,      PPC.eps_fd);
  pnh.param("pose_proposal/clamp_dx",    PPC.clamp_dx,    PPC.clamp_dx);
  pnh.param("pose_proposal/clamp_dy",    PPC.clamp_dy,    PPC.clamp_dy);
  pnh.param("pose_proposal/clamp_dyaw",  PPC.clamp_dyaw,  PPC.clamp_dyaw);
  pnh.param("pose_proposal/min_dt",      PPC.min_dt,      PPC.min_dt);
  pnh.param("pose_proposal/prior_scale", PPC.prior_scale, PPC.prior_scale);

  // Init
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, false);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.03);
  pnh.param("initial_spread/y",   spread_y_,   0.03);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.05);
  pnh.param("odom_buffer_window_sec", odom_buffer_window_sec_, 2.0);

  // Particles
  P_.resize(N_);
  const double w0 = 1.0 / std::max(1, N_);
  for (int i = 0; i < N_; ++i) {
    P_[i].x = 0.0; P_[i].y = 0.0; P_[i].yaw = 0.0;
    P_[i].weight = w0;
    P_[i].log_w  = std::log(w0);
    P_[i].id = i;
    P_[i].map.clear();
  }
  if (seed_from_params_) {
    initializeParticlesFrom(init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
  }

  // Pubs/subs
  pub_particles_posearray_            = nh.advertise<geometry_msgs::PoseArray>("/slam/particles_pose", 1);
  pub_landmarks_posearray_            = nh.advertise<geometry_msgs::PoseArray>("/slam/landmarks_pose", 1);
  pub_landmarks_posearray_confirmed_  = nh.advertise<geometry_msgs::PoseArray>("/slam/landmarks_pose_confirmed", 1);
  pub_landmarks_posearray_locked_     = nh.advertise<geometry_msgs::PoseArray>("/slam/landmarks_pose_locked", 1);
  pub_unmatched_posearray_            = nh.advertise<geometry_msgs::PoseArray>("/slam/unmatched_pose", 1);
  pub_slam_odom_                      = nh.advertise<nav_msgs::Odometry>("/slam/odom", 1);
  pub_slam_odom_mean_                 = nh.advertise<nav_msgs::Odometry>("/slam/odom_mean", 1);

  sub_odom_  = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 50, &FastSLAM2::cbOdom, this);
  sub_cones_ = nh.subscribe<qcar_visnav::ConeArray>("/tracked_cones", 10, &FastSLAM2::cbCones, this);

  ROS_INFO("[FastSLAM2] Ready. assoc_mode=%s", assoc_mode_.c_str());
}

// ================== init ==================
/* -----------------------------------------------------------------------------
initializeParticlesFrom
WHAT:  Seed particle states around a given mean pose with Gaussian spread.
WHY:   Provide initial diversity so the PF can converge even if the prior mean
       is imperfect.
HOW:   Sample x,y,yaw with provided stddevs (sx,sy,syaw). Reset book-keeping.
----------------------------------------------------------------------------- */
void FastSLAM2::initializeParticlesFrom(double x, double y, double yaw,
                                        double sx, double sy, double syaw)
{
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
}

/* -----------------------------------------------------------------------------
spinOnce
WHAT/WHY/HOW: Placeholder; no periodic work needed outside callbacks in this node.
----------------------------------------------------------------------------- */
void FastSLAM2::spinOnce() {}

// ================== odom ==================
/* -----------------------------------------------------------------------------
cbOdom
WHAT:  Consume odometry/EKF odom, optionally snap-initialize particles, and append
       twist samples to a bounded buffer.
WHY:   We later time-align propagation to measurement stamps using this buffer.
HOW:   Store (vx,vy,r, t). Keep only a recent window per odom_buffer_window_sec.
----------------------------------------------------------------------------- */
void FastSLAM2::cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!snapped_to_first_odom_ && overwrite_with_odom_on_first_msg_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
    snapped_to_first_odom_ = true;
  }
  if (!particles_initialized_ && !seed_from_params_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
  }

  OdomStamped o;
  o.t  = msg->header.stamp;
  o.vx = msg->twist.twist.linear.x;
  o.vy = msg->twist.twist.linear.y;
  o.r  = msg->twist.twist.angular.z;
  odom_buf_.push_back(o);
  while (!odom_buf_.empty() &&
         (o.t - odom_buf_.front().t).toSec() > odom_buffer_window_sec_) {
    odom_buf_.pop_front();
  }
}

/* -----------------------------------------------------------------------------
interpTwist
WHAT:  Linearly interpolate body-frame twist between two odom samples at time s.
WHY:   Gives smooth per-segment propagation matching measurement time stamps.
HOW:   Linear blend of vx and r; vy is taken from 'a' (as per source code intent).
----------------------------------------------------------------------------- */
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
  result.vy = (1.0 - frac)*a.vy;
  result.r  = (1.0 - frac)*a.r  + frac*b.r;
  return result;
}

/* -----------------------------------------------------------------------------
propagateParticlesTo
WHAT:  Advance each particle from the last processed time to t using the buffered
       odometry, with piecewise-constant (interpolated endpoints) twists.
WHY:   Ensures motion model propagation is time-aligned to sensor frames.
HOW:   For each buffered segment overlapping (last_prop_stamp_, t]:
        - Interpolate twists at segment endpoints
        - Average them and integrate dt via MotionModel::propagate
----------------------------------------------------------------------------- */
void FastSLAM2::propagateParticlesTo(const ros::Time& t) {
  if (!particles_initialized_ || odom_buf_.size() < 2) return;
  if (last_prop_stamp_.isZero()) last_prop_stamp_ = odom_buf_.front().t;
  if (t <= last_prop_stamp_) return;

  for (size_t i = 0; i + 1 < odom_buf_.size(); ++i) {
    const auto& a = odom_buf_[i];
    const auto& b = odom_buf_[i+1];
    const ros::Time seg_start = std::max(a.t, last_prop_stamp_);
    const ros::Time seg_end   = std::min(b.t, t);
    if (seg_end <= seg_start) continue;

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

// ================== cones ==================
/* -----------------------------------------------------------------------------
lookupLidarExtrinsics
WHAT:  Query TF at time t for lidar pose relative to base: (x,y,yaw).
WHY:   Corrects measurement geometry/Jacobians for sensor offsets to avoid bias.
HOW:   tf2 buffer lookupTransform(base_frame, lidar_frame, t), convert to yaw.
----------------------------------------------------------------------------- */
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

/* -----------------------------------------------------------------------------
cbCones
WHAT:  Measurement callback for tracked cones. Propagate to stamp, fetch
       extrinsics, convert to RB measurements, run the core update, compute
       unmatched positions for viz, and publish outputs.
WHY:   Central per-frame entry point that aligns motion & measurement processing.
HOW:
  - propagateParticlesTo(msg->header.stamp)
  - lookupLidarExtrinsics
  - build MeasRB array from ConeArray (range/bearing/variances/id)
  - processMeasurementsAt(...) -> FS2.0 + EKF + weights + resample + best/mean
  - project unmatched detections to world (best particle) for visualization
  - publishCoreOutputs
----------------------------------------------------------------------------- */
void FastSLAM2::cbCones(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!particles_initialized_) {
    ROS_WARN_THROTTLE(2.0, "[FastSLAM2] Cones before particles init; ignoring");
    return;
  }
  propagateParticlesTo(msg->header.stamp);

  LidarExtrinsics ex;
  if (!lookupLidarExtrinsics(msg->header.stamp, ex) || !ex.valid) return;

  std::vector<MeasRB> meas; meas.reserve(msg->cones.size());
  for (const auto& c : msg->cones) {
    if (!std::isfinite(c.range) || !std::isfinite(c.bearing)) continue;
    MeasRB m;
    m.r     = c.range;
    m.b     = c.bearing;
    m.r_var = std::max(1e-10, c.r_var);
    m.b_var = std::max(1e-12, c.bearing_var);
    m.id    = c.id; // requires MeasRB::id
    meas.push_back(m);
  }
  if (meas.empty()) return;

  // core update (per-particle landmark updates, resampling, best index)
  processMeasurementsAt(msg->header.stamp, meas, ex);

  // Compute unmatched (w.r.t. best particle) in WORLD to feed a PoseArray for viz
  std::vector<Eigen::Vector2d> unmatched_world;
  const auto& bestP = P_[std::max(0, std::min<int>(best_idx_, (int)P_.size()-1))];
  unmatched_world.reserve(meas.size());
  for (const auto& m : meas) {
    int lidx = (assoc_mode_ == "id") ? associateById(bestP, m)
                                     : associateByNNRB(bestP, m, ex, chi2_gate_rb_);
    if (lidx < 0) {
      Eigen::Vector2d mu_w; Eigen::Matrix2d J;
      measRBToWorld(bestP.x, bestP.y, bestP.yaw, ex, m.r, m.b, mu_w, J);
      unmatched_world.push_back(mu_w);
    }
  }

  // publish (PoseArrays + odom + unmatched PoseArray)
  publishCoreOutputs(msg->header.stamp, unmatched_world);
}

// ================== core update ==================
/* -----------------------------------------------------------------------------
processMeasurementsAt
WHAT:  Core per-frame SLAM step: (A) optional FastSLAM 2.0 pose-proposal,
       (B) landmark EKF updates + particle reweighting, (C) normalization,
       resampling, and best/mean pose extraction.
WHY:   Implements Rao–Blackwellized FastSLAM with a measurement-aware proposal.
HOW (A) Pose proposal (if enabled):
       - Build provisional matches for each measurement (ID or NN in RB with χ² gate).
       - Keep top-K by Mahalanobis distance (d²).
       - Build pose posterior information:
           Q = diag(σ²) * max(dt, min_dt) * prior_scale
           Λ = Q^{-1} + Σ Hxᵀ S^{-1} Hx
           η = Σ Hxᵀ S^{-1} ν
         Solve δx = Λ^{-1} η and apply (with small safety clamps).
     (B) Landmark EKF & log-likelihood:
       - For each meas: (associate, gate with S), compute ν and S, accumulate
         log p(z|.) = −½(νᵀ S^{-1} ν + log|2πS|), and EKF-update matched landmark.
       - If unmatched: birth new landmark with Σ_new = J R Jᵀ + init_var I.
     (C) Weights & resampling:
       - Softmax log_w, normalize; resample if N_eff below threshold.
       - Track best (argmax w) and compute mean pose (circular for yaw).
NOTE:  All weighting uses S^{-1} (not R^{-1}) to honor landmark uncertainty.
----------------------------------------------------------------------------- */
void FastSLAM2::processMeasurementsAt(const ros::Time& t,
                                      const std::vector<MeasRB>& meas_vec,
                                      const LidarExtrinsics& ex)
{
  // === Per-particle: measurement-aware pose proposal (FastSLAM 2.0) ===
  if (PPC.enable) {
    const int    Kmax   = PPC.Kmax;
    const double epsFD  = PPC.eps_fd;

    // Approximate dt for pose prior Q from odom; enforce a lower bound
    double dt_Q = PPC.min_dt;
    if (odom_buf_.size() >= 2) {
      const auto& a = odom_buf_[odom_buf_.size()-2];
      const auto& b = odom_buf_.back();
      dt_Q = std::max(PPC.min_dt, (b.t - a.t).toSec());
    }

    Eigen::Vector3d qv(
      motion_noise_.sigma_x   * motion_noise_.sigma_x,
      motion_noise_.sigma_y   * motion_noise_.sigma_y,
      motion_noise_.sigma_yaw * motion_noise_.sigma_yaw
    );
    Eigen::Matrix3d Q_prior = qv.asDiagonal() * dt_Q * PPC.prior_scale;

    for (auto& p : P_) {
      // --- 1) Build provisional associations and scores (d2) ---
      struct Match {
        int lidx;               // landmark index
        int midx;               // measurement index
        double d2;              // Mahalanobis distance
        Eigen::Vector2d nu;     // innovation (wrapped)
        Eigen::Matrix2d Sinv;   // (H Σ H^T + R)^{-1}
      };
      std::vector<Match> matches; matches.reserve(meas_vec.size());

      for (int mi = 0; mi < (int)meas_vec.size(); ++mi) {
        const auto& m = meas_vec[mi];
        int lidx = -1;

        if (assoc_mode_ == "id") {
          lidx = associateById(p, m);
          if (lidx >= 0) {
            const auto& lm = p.map[lidx];
            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = std::max(1e-10, m.r_var);
            R(1,1) = std::max(1e-12, m.b_var);

            double r_h=0, b_h=0; Eigen::Matrix2d Hlm;
            predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, Hlm);
            Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);
            const Eigen::Matrix2d S = Hlm * lm.Sigma * Hlm.transpose() + R;
            Eigen::LLT<Eigen::Matrix2d> llt(S);
            if (llt.info() == Eigen::Success) {
              const Eigen::Matrix2d Sinv = llt.solve(Eigen::Matrix2d::Identity());
              const double d2 = (nu.transpose() * Sinv * nu)(0,0);
              if (d2 <= chi2_gate_rb_) {
                matches.push_back({lidx, mi, d2, nu, Sinv});
              }
            }
          }
        } else {
          lidx = associateByNNRB(p, m, ex, chi2_gate_rb_);
          if (lidx >= 0) {
            const auto& lm = p.map[lidx];
            Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
            R(0,0) = std::max(1e-10, m.r_var);
            R(1,1) = std::max(1e-12, m.b_var);
            double r_h=0, b_h=0; Eigen::Matrix2d Hlm;
            predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, Hlm);
            Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);
            const Eigen::Matrix2d S = Hlm * lm.Sigma * Hlm.transpose() + R;
            Eigen::LLT<Eigen::Matrix2d> llt(S);
            if (llt.info() == Eigen::Success) {
              const Eigen::Matrix2d Sinv = llt.solve(Eigen::Matrix2d::Identity());
              const double d2 = (nu.transpose() * Sinv * nu)(0,0);
              matches.push_back({lidx, mi, d2, nu, Sinv});
            }
          }
        }
      }

      if (!matches.empty()) {
        // --- 2) pick up to K best by Mahalanobis distance ---
        std::sort(matches.begin(), matches.end(),
                  [](const Match& a, const Match& b){ return a.d2 < b.d2; });
        if ((int)matches.size() > Kmax) matches.resize(Kmax);

        // --- 3) Build pose posterior info and take one Gauss–Newton step ---
        Eigen::Matrix3d Lambda = Q_prior.inverse();
        Eigen::Vector3d eta    = Eigen::Vector3d::Zero();

        for (const auto& mrec : matches) {
          const auto& lm = p.map[mrec.lidx];

          auto h = [&](double X, double Y, double Yaw){
            double r=0, b=0; Eigen::Matrix2d H_unused;
            predictMeasurementRB(X, Y, Yaw, lm.mu, ex, r, b, H_unused);
            return Eigen::Vector2d(r,b);
          };

          const Eigen::Vector2d h0 = h(p.x, p.y, p.yaw);
          Eigen::Matrix<double,2,3> Hx;
          Hx.col(0) = (h(p.x+epsFD, p.y,       p.yaw     ) - h0) / epsFD;
          Hx.col(1) = (h(p.x,       p.y+epsFD, p.yaw     ) - h0) / epsFD;
          Hx.col(2) = (h(p.x,       p.y,       p.yaw+epsFD) - h0) / epsFD;

          // Weight by S^{-1} (landmark + measurement uncertainty)
          const Eigen::Matrix2d& W = mrec.Sinv;
          Lambda += Hx.transpose() * W * Hx;
          eta    += Hx.transpose() * W * mrec.nu;
        }

        Eigen::LDLT<Eigen::Matrix3d> ldlt(Lambda);
        if (ldlt.info() == Eigen::Success) {
          const Eigen::Vector3d dxi = ldlt.solve(eta);
          // Safety clamps on the GN pose step (tunable in YAML).
          const double dx   = std::max(-PPC.clamp_dx,   std::min(PPC.clamp_dx,   dxi(0)));
          const double dy   = std::max(-PPC.clamp_dy,   std::min(PPC.clamp_dy,   dxi(1)));
          const double dyaw = std::max(-PPC.clamp_dyaw, std::min(PPC.clamp_dyaw, dxi(2)));

          p.x   += dx;
          p.y   += dy;
          p.yaw  = wrapPi(p.yaw + dyaw);
        }
      }
    } // end pose proposal loop
  }

  // === Landmark EKF updates + log-likelihood accumulation (original) ===
  for (auto& p : P_) {
    double log_w_inc = 0.0;
    int matched_cnt = 0;

    for (const auto& m : meas_vec) {
      int lidx = -1;
      if (assoc_mode_ == "id") {
        lidx = associateById(p, m);
        // gate the ID match once more here to avoid bad updates
        if (lidx >= 0) {
          const auto& lm = p.map[lidx];
          Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
          Rz(0,0) = std::max(1e-10, m.r_var);
          Rz(1,1) = std::max(1e-12, m.b_var);
          double r_h=0, b_h=0; Eigen::Matrix2d H;
          predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
          Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);
          const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Rz;
          Eigen::LLT<Eigen::Matrix2d> llt(S);
          if (llt.info() == Eigen::Success) {
            const Eigen::Matrix2d Sinv = llt.solve(Eigen::Matrix2d::Identity());
            const double d2 = (nu.transpose() * Sinv * nu)(0,0);
            if (d2 > chi2_gate_rb_) lidx = -1; // reject bad ID
          } else {
            lidx = -1;
          }
        }
      } else {
        lidx = associateByNNRB(p, m, ex, chi2_gate_rb_);
      }

      if (lidx >= 0) {
        auto& lm = p.map[lidx];

        Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
        Rz(0,0) = std::max(1e-10, m.r_var);
        Rz(1,1) = std::max(1e-12, m.b_var);

        double r_h=0, b_h=0; Eigen::Matrix2d H;
        predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);

        Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);
        const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Rz;

        Eigen::LLT<Eigen::Matrix2d> llt(S);
        if (llt.info() != Eigen::Success) {
          Eigen::Matrix2d Sj = S + 1e-9 * Eigen::Matrix2d::Identity();
          llt.compute(Sj);
        }
        const Eigen::Matrix2d Sinv = llt.solve(Eigen::Matrix2d::Identity());
        const double quad = (nu.transpose() * Sinv * nu)(0,0);
        Eigen::Matrix2d L = llt.matrixL().toDenseMatrix();
        const double logdet = 2.0 * (std::log(std::max(1e-18, L(0,0))) +
                                     std::log(std::max(1e-18, L(1,1))));
        log_w_inc += -0.5 * (quad + logdet + 2.0 * std::log(2.0 * M_PI));
        matched_cnt++;

        // EKF update (skip if locked)
        if (!lm.locked) {
          const Eigen::Matrix2d K = lm.Sigma * H.transpose() * Sinv;
          lm.mu    = lm.mu + K * nu;
          lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;
        }

        lm.hits++;
        if (!lm.confirmed && lm.hits >= lm_confirm_hits_) lm.confirmed = true;
        if (!lm.locked) {
          if (lm.hits >= lm_lock_hits_ || lm.Sigma.trace() <= lm_lock_cov_trace_) {
            lm.locked = true;
          }
        }
      } else {
        // Unmatched -> birth
        Eigen::Vector2d mu_w; Eigen::Matrix2d J;
        measRBToWorld(p.x, p.y, p.yaw, ex, m.r, m.b, mu_w, J);

        Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
        Rz(0,0) = std::max(1e-10, m.r_var);
        Rz(1,1) = std::max(1e-12, m.b_var);

        Landmark lm;
        lm.mu = mu_w;
        lm.Sigma = J * Rz * J.transpose() + lm_init_var_ * Eigen::Matrix2d::Identity();
        lm.hits = 1; lm.misses = 0; lm.confirmed = (lm_confirm_hits_ <= 1);
        lm.locked = (lm.hits >= lm_lock_hits_) || (lm.Sigma.trace() <= lm_lock_cov_trace_);
        lm.id = m.id;
        p.map.push_back(lm);
      }
    } // measurements

    // Optional penalty when nothing matched (kept disabled)
    // if (matched_cnt == 0) log_w_inc += -1.0;

    p.log_w += log_w_inc;
  } // particles

  // === Convert log-weights -> weights (stable), then normalize ===
  double max_logw = -std::numeric_limits<double>::infinity();
  for (const auto& p : P_) max_logw = std::max(max_logw, p.log_w);

  double sum_w = 0.0;
  for (auto& p : P_) {
    p.weight = std::exp(p.log_w - max_logw);
    sum_w += p.weight;
  }
  if (sum_w <= 0.0) {
    const double w0 = 1.0 / std::max<int>(1, P_.size());
    for (auto& p : P_) { p.weight = w0; p.log_w = std::log(w0); }
  } else {
    for (auto& p : P_) p.weight /= sum_w;
  }

  // === Resample if needed ===
  double inv_neff = 0.0; for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;
  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_, rng_);
    for (auto& p : P_) p.log_w = std::log(std::max(1e-300, p.weight));
  }

  // === Best & mean pose ===
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


// (optional) measurement log-likelihood
/* -----------------------------------------------------------------------------
measLogLikelihood
WHAT:  Compute log p(z | particle pose, landmark) using innovation ν and S.
WHY:   Useful for diagnostics or alternative weighting strategies.
HOW:   Cholesky on S for log|S| and solve for νᵀ S^{-1} ν; return Gaussian logpdf.
NOTE:  This uses S = H Σ Hᵀ + R consistent with the rest of the pipeline.
----------------------------------------------------------------------------- */
double FastSLAM2::measLogLikelihood(const Landmark& lm,
                                    const Particle& p,
                                    const MeasRB& m,
                                    const LidarExtrinsics& ex) const
{
  Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
  Rz(0,0) = std::max(1e-10, m.r_var);
  Rz(1,1) = std::max(1e-12, m.b_var);

  double r_h=0, b_h=0; Eigen::Matrix2d H;
  predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
  Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);

  const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Rz;
  Eigen::LLT<Eigen::Matrix2d> llt(S);
  if (llt.info() != Eigen::Success) {
    Eigen::Matrix2d Sj = S + 1e-9 * Eigen::Matrix2d::Identity();
    llt.compute(Sj);
  }
  Eigen::Matrix2d L = llt.matrixL().toDenseMatrix();
  const double logdet = 2.0 * (std::log(std::max(1e-18, L(0,0))) +
                               std::log(std::max(1e-18, L(1,1))));
  const Eigen::Vector2d y = llt.solve(nu);
  const double quad = nu.dot(y);
  return -0.5 * (quad + logdet + 2.0 * std::log(2.0 * M_PI));
}

// ================== publish outputs ==================
/* -----------------------------------------------------------------------------
publishCoreOutputs
WHAT:  Publish particles, landmarks (all / confirmed / locked), unmatched current
       detections (world-projected), and odom (best & mean).
WHY:   Lightweight visualization and downstream consumption.
HOW:
  - PoseArray of particle states in map_frame
  - PoseArray of landmarks for the best particle (all/confirmed/locked subsets)
  - PoseArray of unmatched projected positions (this frame)
  - Odometry messages for best and mean particle poses
----------------------------------------------------------------------------- */
void FastSLAM2::publishCoreOutputs(const ros::Time& t,
                                   const std::vector<Eigen::Vector2d>& unmatched_world)
{
  // Particles (PoseArray)
  geometry_msgs::PoseArray particles;
  particles.header.stamp = t;
  particles.header.frame_id = map_frame_;
  for (const auto& p : P_) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x; pose.position.y = p.y; pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,p.yaw);
    pose.orientation = tf2::toMsg(q);
    particles.poses.push_back(pose);
  }
  pub_particles_posearray_.publish(particles);

  // Landmarks PoseArrays (best particle)
  const auto& bestP = P_[std::max(0, std::min<int>(best_idx_, (int)P_.size()-1))];

  geometry_msgs::PoseArray lms_all;
  lms_all.header.stamp = t;
  lms_all.header.frame_id = map_frame_;
  for (const auto& lm : bestP.map) {
    geometry_msgs::Pose pose;
    pose.position.x = lm.mu.x();
    pose.position.y = lm.mu.y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    lms_all.poses.push_back(pose);
  }
  pub_landmarks_posearray_.publish(lms_all);

  geometry_msgs::PoseArray lms_confirmed;
  lms_confirmed.header.stamp = t;
  lms_confirmed.header.frame_id = map_frame_;
  for (const auto& lm : bestP.map) {
    if (!lm.confirmed) continue;
    geometry_msgs::Pose pose;
    pose.position.x = lm.mu.x();
    pose.position.y = lm.mu.y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    lms_confirmed.poses.push_back(pose);
  }
  pub_landmarks_posearray_confirmed_.publish(lms_confirmed);

  // Optional: locked set
  geometry_msgs::PoseArray lms_locked;
  lms_locked.header.stamp = t;
  lms_locked.header.frame_id = map_frame_;
  for (const auto& lm : bestP.map) {
    if (!lm.locked) continue;
    geometry_msgs::Pose pose;
    pose.position.x = lm.mu.x();
    pose.position.y = lm.mu.y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    lms_locked.poses.push_back(pose);
  }
  pub_landmarks_posearray_locked_.publish(lms_locked);

  // Unmatched (this frame) as PoseArray (for red spheres in viz)
  geometry_msgs::PoseArray unmatched;
  unmatched.header.stamp = t;
  unmatched.header.frame_id = map_frame_;
  for (const auto& v : unmatched_world) {
    geometry_msgs::Pose pose;
    pose.position.x = v.x();
    pose.position.y = v.y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    unmatched.poses.push_back(pose);
  }
  pub_unmatched_posearray_.publish(unmatched);

  // Odom (best + mean)
  const auto& bp = bestP;
  nav_msgs::Odometry odom_best;
  odom_best.header.stamp = t;
  odom_best.header.frame_id = map_frame_;
  odom_best.child_frame_id  = base_frame_;
  odom_best.pose.pose.position.x = bp.x;
  odom_best.pose.pose.position.y = bp.y;
  tf2::Quaternion q_best; q_best.setRPY(0,0,bp.yaw);
  odom_best.pose.pose.orientation = tf2::toMsg(q_best);
  pub_slam_odom_.publish(odom_best);

  nav_msgs::Odometry odom_mean = odom_best;
  odom_mean.pose.pose.position.x = mean_x_;
  odom_mean.pose.pose.position.y = mean_y_;
  tf2::Quaternion q_mean; q_mean.setRPY(0,0,mean_yaw_);
  odom_mean.pose.pose.orientation = tf2::toMsg(q_mean);
  pub_slam_odom_mean_.publish(odom_mean);

  // Debug: print persistent landmarks from best particle
  {
    const auto& bp = bestP; // bestP is defined above in this function
    ROS_INFO("[FastSLAM2] best_particle:%d landmarks=%zu", best_idx_, bp.map.size());
    for (const auto& lm : bp.map) {
      const double trace = lm.Sigma.trace();
      ROS_INFO("[FastSLAM2] LM[id=%d] x=%.3f y=%.3f trace=%.6f hits=%d conf=%d locked=%d",
               lm.id, lm.mu.x(), lm.mu.y(), trace,
               lm.hits, lm.confirmed ? 1 : 0, lm.locked ? 1 : 0);
    }
  }
}
