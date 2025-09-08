#include "qcar_visnav/slam/fastslam2.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/measurement_model.h"

using namespace qcar_visnav::slam;

// ================== file-local helpers ==================
namespace {

inline double wrapPi(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

// Associate by tracker ID; returns -1 if no match
inline int associateById(const Particle& p, const MeasRB& m) {
  if (m.id <= 0) return -1;
  for (int i = 0; i < (int)p.map.size(); ++i) {
    if (p.map[i].id == m.id) return i;
  }
  return -1;
}

// Nearest-neighbor in RB space with chi^2(2) gate; returns -1 if no valid match
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

// log N(x|mu,Sigma) helper (kept for completeness)
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

} // anon

// ================== ctor ==================
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

void FastSLAM2::spinOnce() {}

// ================== odom ==================
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
void FastSLAM2::processMeasurementsAt(const ros::Time& t,
                                      const std::vector<MeasRB>& meas_vec,
                                      const LidarExtrinsics& ex)
{
  // === Per-particle: DA + LM EKF updates + accumulate measurement log-likelihood ===
  for (auto& p : P_) {
    double log_w_inc = 0.0;   // accumulate log-likelihood over matched measurements
    int matched_cnt = 0;

    for (const auto& m : meas_vec) {
      int lidx = -1;
      if (assoc_mode_ == "id") {
        lidx = associateById(p, m);
      } else { // "nn_rb"
        lidx = associateByNNRB(p, m, ex, chi2_gate_rb_);
      }

      if (lidx >= 0) {
        // --- Matched landmark: EKF update (unless locked) + likelihood for weights ---
        auto& lm = p.map[lidx];

        Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
        Rz(0,0) = std::max(1e-10, m.r_var);
        Rz(1,1) = std::max(1e-12, m.b_var);

        double r_h=0, b_h=0; Eigen::Matrix2d H;
        predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);

        Eigen::Vector2d nu; nu << (m.r - r_h), wrapPi(m.b - b_h);
        const Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + Rz;

        // log-likelihood: -0.5*(nu^T S^-1 nu + logdet(2πS))
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

        // EKF state update (skip state change if locked, but keep hit/locking logic)
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
        // --- Unmatched: birth a new LM in world frame (no weight bonus) ---
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

    // If nothing matched, you can optionally add a tiny penalty to discourage “blind” particles:
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
}
