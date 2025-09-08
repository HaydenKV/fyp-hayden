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

// ========== proposal helpers ==========
namespace {
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
  if (llt.info() != Eigen::Success) return mu;
  const Eigen::MatrixXd L = llt.matrixL();
  std::normal_distribution<double> N01(0.0, 1.0);
  Eigen::VectorXd z(d);
  for (int i = 0; i < d; ++i) z(i) = N01(rng);
  return mu + L * z;
}
} // anon

// ========== ctor ==========
FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise())
{
  ROS_INFO("[FastSLAM2] Initializing (core, no viz)...");

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
  pnh.param("association/method", association_method_, std::string("greedy"));
  pnh.param("association/chi2_gate", chi2_gate_, 7.38);
  pnh.param("association/chi2_gate_world", chi2_gate_world_, 9.21);
  pnh.param("association/ambiguity_threshold", ambiguity_threshold_, 0.7);
  pnh.param("association/use_jcbb", use_jcbb_, false);
  if (association_method_ == "jcbb") use_jcbb_ = true;
  data_assoc_.reset(new DataAssociation(chi2_gate_, ambiguity_threshold_, /*use_hungarian=*/false));

  // Landmark mgmt
  pnh.param("confirm_hits",    confirm_hits_,    3);
  pnh.param("min_new_lm_dist", min_new_lm_dist_, 0.25);
  pnh.param("merge_R_scale",   merge_R_scale_,   4.0);
  pnh.param("landmark_prior_var", landmark_prior_var_, 0.15);
  pnh.param("landmarks/unconfirmed_R_scale",      unconfirmed_R_scale_,      1.8);
  pnh.param("landmarks/prune_unconfirmed_misses", prune_unconfirmed_misses_, 10);
  pnh.param("landmarks/prune_stale_misses",       prune_stale_misses_,       35);

  // Motion model params
  MotionNoise enhanced_noise;
  pnh.param("motion_noise/sigma_x",   enhanced_noise.sigma_x,   0.03);
  pnh.param("motion_noise/sigma_y",   enhanced_noise.sigma_y,   0.03);
  pnh.param("motion_noise/sigma_yaw", enhanced_noise.sigma_yaw, 0.008);
  pnh.param("motion_noise/velocity_noise_scale", enhanced_noise.velocity_noise_scale, 0.4);
  pnh.param("motion_noise/min_velocity_for_scaling", enhanced_noise.min_velocity_for_scaling, 0.12);
  pnh.param("motion_noise/turning_noise_scale", enhanced_noise.turning_noise_scale, 0.6);
  pnh.param("motion_noise/min_yawrate_for_turning", enhanced_noise.min_yawrate_for_turning, 0.08);
  pnh.param("motion_noise/max_position_std", enhanced_noise.max_position_std, 0.08);
  pnh.param("motion_noise/max_yaw_std", enhanced_noise.max_yaw_std, 0.12);
  motion_ = MotionModel(enhanced_noise);
  motion_noise_ = enhanced_noise;

  // Births
  pnh.param("birth/required_hits", birth_required_hits_, 3);
  pnh.param("birth/max_age",       birth_max_age_,       10);
  pnh.param("birth/promote_radius",birth_promote_radius_, 0.25);

  // Proposal & bonuses
  pnh.param("proposal/max_sigma_trace", proposal_max_sigma_trace_, 0.25);
  pnh.param("proposal/sample_every_k",  proposal_sample_every_k_,  0);
  pnh.param("proposal/min_information_for_proposal", min_information_for_proposal_, 2.0);
  pnh.param("proposal/high_quality_threshold", high_quality_threshold_, 0.8);
  pnh.param("weight/new_landmark_penalty", new_landmark_penalty_, 0.8);
  pnh.param("weight/high_quality_bonus",   high_quality_bonus_,   0.1);
  pnh.param("weight/information_bonus_scale", information_bonus_scale_, 0.05);

  // Sensor
  pnh.param("sensor/fov_range_max",        fov_range_max_,        18.0);
  double fov_bearing_deg = 85.0;
  pnh.param("sensor/fov_bearing_deg",      fov_bearing_deg,       85.0);
  fov_bearing_rad_ = fov_bearing_deg * M_PI / 180.0;

  // Init
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, true);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.03);
  pnh.param("initial_spread/y",   spread_y_,   0.03);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.05);
  pnh.param("odom_buffer_window_sec", odom_buffer_window_sec_, 2.0);
  pnh.param("lap/freeze_after_first_loop", freeze_after_first_loop_, false);

  // Particles
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
    initializeParticlesFrom(init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
  }

  // Pubs/subs
  pub_particles_posearray_            = nh.advertise<geometry_msgs::PoseArray>("/slam/particles_pose", 1);
  pub_landmarks_posearray_            = nh.advertise<geometry_msgs::PoseArray>("/slam/landmarks_pose", 1);
  pub_landmarks_posearray_confirmed_  = nh.advertise<geometry_msgs::PoseArray>("/slam/landmarks_pose_confirmed", 1);
  pub_births_posearray_               = nh.advertise<geometry_msgs::PoseArray>("/slam/births_pose", 1);
  pub_slam_odom_                      = nh.advertise<nav_msgs::Odometry>("/slam/odom", 1);
  pub_slam_odom_mean_                 = nh.advertise<nav_msgs::Odometry>("/slam/odom_mean", 1);

  sub_odom_  = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 50, &FastSLAM2::cbOdom, this);
  sub_cones_ = nh.subscribe<qcar_visnav::ConeArray>("/tracked_cones", 10, &FastSLAM2::cbCones, this);

  ROS_INFO("[FastSLAM2] Core initialized. Publishing PoseArrays (particles, landmarks, births).");
}

// ========== init ==========
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

// ========== odom ==========
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

// ========== cones ==========
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
    meas.push_back(m);
  }
  if (meas.empty()) return;

  processMeasurementAt(msg->header.stamp, meas, ex);
  publishCoreOutputs(msg->header.stamp);
}

// ========== core update ==========
double FastSLAM2::worldMaha2(const Eigen::Vector2d& z_world,
                             const Eigen::Matrix2d& Rw,
                             const Landmark& lm) const
{
  const Eigen::Matrix2d Sw   = lm.Sigma + Rw;
  const Eigen::Matrix2d Sinv = Sw.inverse();
  const Eigen::Vector2d d    = z_world - lm.mu;
  return (d.transpose() * Sinv * d)(0,0);
}

void FastSLAM2::processMeasurementAt(const ros::Time& t,
                                     const std::vector<MeasRB>& meas_vec,
                                     const LidarExtrinsics& ex)
{
  double dt = 0.01;
  if (!last_prop_stamp_.isZero()) {
    dt = std::max(0.0, (t - last_prop_stamp_).toSec());
  }

  Eigen::Matrix2d base_R = Eigen::Matrix2d::Identity();
  base_R(0,0) = 0.01;
  base_R(1,1) = 0.01;

  for (auto& p : P_) {
    GlobalAssignment assignment;
    if (association_method_ == "jcbb") {
      assignment = data_assoc_->associateJCBB(p, meas_vec, ex, base_R);
    } else {
      assignment = data_assoc_->associateGreedy(p, meas_vec, ex, base_R);
    }

    double log_proposal_correction = 0.0;
    computeEnhancedPoseProposal(p, assignment, meas_vec, ex, dt, log_proposal_correction);
    p.log_w += log_proposal_correction;

    std::vector<bool> lm_seen(p.map.size(), false);
    updateMatchedLandmarks(p, assignment, meas_vec, ex, lm_seen);

    int promotions_this_frame = 0;
    processUnmatchedMeasurements(p, assignment.unmatched_measurements, meas_vec, ex,
                                 promotions_this_frame);

    applyWeightAdjustments(p, assignment, lm_seen, ex, promotions_this_frame);
  }

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

  double inv_neff = 0.0; for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;
  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_, rng_);
    ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Resampled: Neff=%.1f < %.1f", neff, neff_ratio_ * P_.size());
    for (auto& p : P_) p.log_w = std::log(std::max(1e-300, p.weight));
  }

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

// ========== proposal / updates / weights (same as before) ==========
void FastSLAM2::computeEnhancedPoseProposal(Particle& p,
                                            const GlobalAssignment& assignment,
                                            const std::vector<MeasRB>& meas_vec,
                                            const LidarExtrinsics& ex,
                                            double dt,
                                            double& log_proposal_correction)
{
  double current_vx = 0.0, current_vy = 0.0, current_r = 0.0;
  if (!odom_buf_.empty()) {
    const auto& latest_odom = odom_buf_.back();
    current_vx = latest_odom.vx;
    current_vy = latest_odom.vy;
    current_r = latest_odom.r;
  }

  Eigen::Vector3d mu_prior(p.x, p.y, p.yaw);
  Eigen::Matrix3d Qprior = motion_.getMotionCovariance(current_vx, current_vy, current_r, dt);
  Qprior += 1e-9 * Eigen::Matrix3d::Identity();

  Eigen::Matrix3d Lambda = Qprior.inverse();
  Eigen::Vector3d eta = Lambda * mu_prior;

  double log_det_S_norm_sum = 0.0;
  int num_pose_meas = 0;
  double total_information = 0.0;

  for (const auto& match : assignment.matches) {
    const int m_idx = match.first;
    const int l_idx = match.second;
    const auto& lm = p.map[l_idx];
    const auto& meas = meas_vec[m_idx];

    bool is_informative = lm.confirmed && (lm.Sigma.trace() <= proposal_max_sigma_trace_);
    if (!lm.confirmed) {
      is_informative = (lm.Sigma.trace() <= proposal_max_sigma_trace_ * 0.6) && (lm.hits >= 2);
    }
    if (!is_informative) continue;

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = meas.r_var; R(1,1) = meas.b_var;

    double r_hat, b_hat;
    Eigen::Matrix2d H;
    Eigen::Matrix<double,2,3> Gx;
    predictRBWithJacobians(p.x, p.y, p.yaw, lm.mu, ex, r_hat, b_hat, Gx, H);

    Eigen::Vector2d nu; nu << (meas.r - r_hat), wrapToPi(meas.b - b_hat);
    Eigen::Matrix2d S = H * lm.Sigma * H.transpose() + R;
    if (!lm.confirmed) S += R * (unconfirmed_R_scale_ - 1.0);

    Eigen::LLT<Eigen::Matrix2d> llt(S);
    if (llt.info() != Eigen::Success) continue;

    const Eigen::Matrix2d S_inv = llt.solve(Eigen::Matrix2d::Identity());
    const Eigen::Matrix<double,3,2> Gt = Gx.transpose();

    double information_weight = 1.0;
    if (assignment.average_compatibility > high_quality_threshold_) information_weight = 1.3;
    else if (assignment.average_compatibility < 0.4) information_weight = 0.6;

    const Eigen::Matrix3d info_contrib = Gt * S_inv * Gx * information_weight;
    const Eigen::Vector3d eta_contrib  = Gt * S_inv * nu * information_weight;
    Lambda += info_contrib;
    eta    += eta_contrib;

    log_det_S_norm_sum += std::log(std::max(1e-18, S.determinant()));
    total_information  += S_inv.trace() * information_weight;
    ++num_pose_meas;
  }

  Eigen::Matrix3d Sigma_q = Lambda.inverse();
  Eigen::Vector3d mu_q = Sigma_q * eta;

  Eigen::Vector3d x_samp = mu_q;
  const bool high_info = (total_information >= min_information_for_proposal_) && (num_pose_meas >= 1);
  if (high_info && proposal_sample_every_k_ > 0) {
    static uint64_t frame_idx = 0;
    int sample_freq = std::max(1, proposal_sample_every_k_);
    if (assignment.average_compatibility > high_quality_threshold_) {
      sample_freq = std::max(1, sample_freq / 2);
    }
    if ((frame_idx++ % sample_freq) == 0) {
      x_samp = sampleGaussian(mu_q, Sigma_q, rng_);
    }
  }

  p.x = x_samp(0);
  p.y = x_samp(1);
  p.yaw = std::atan2(std::sin(x_samp(2)), std::cos(x_samp(2)));

  const double log_p_motion = logGaussian(x_samp, mu_prior, Qprior);
  const double log_q        = logGaussian(x_samp, mu_q,    Sigma_q);
  const double log_lik_norm =
      (num_pose_meas > 0) ? (-0.5 * log_det_S_norm_sum - 0.5 * num_pose_meas * std::log(2*M_PI)) : 0.0;

  double info_bonus = 0.0;
  if (high_info && assignment.average_compatibility > high_quality_threshold_) {
    info_bonus = high_quality_bonus_ + information_bonus_scale_ * std::log1p(total_information);
  }
  log_proposal_correction = log_p_motion - log_q + log_lik_norm + info_bonus;
}

void FastSLAM2::updateMatchedLandmarks(Particle& p,
                                       const GlobalAssignment& assignment,
                                       const std::vector<MeasRB>& meas_vec,
                                       const LidarExtrinsics& ex,
                                       std::vector<bool>& lm_seen)
{
  for (const auto& match : assignment.matches) {
    const int m_idx = match.first;
    const int l_idx = match.second;
    auto& lm = p.map[l_idx];
    const auto& meas = meas_vec[m_idx];

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0,0) = meas.r_var; R(1,1) = meas.b_var;
    Eigen::Matrix2d R_eff = (!lm.confirmed) ? (R * unconfirmed_R_scale_) : R;

    double r_h, b_h; Eigen::Matrix2d H;
    predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
    Eigen::Vector2d nu; nu << (meas.r - r_h), wrapToPi(meas.b - b_h);

    const Eigen::Matrix2d S    = H * lm.Sigma * H.transpose() + R_eff;
    const Eigen::Matrix2d Sinv = S.inverse();
    const Eigen::Matrix2d K    = lm.Sigma * H.transpose() * Sinv;

    lm.mu    = lm.mu + K * nu;
    lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;

    lm.hits++;
    if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

    if (l_idx < (int)lm_seen.size()) lm_seen[l_idx] = true;
  }
}

void FastSLAM2::processUnmatchedMeasurements(Particle& p,
                                             const std::vector<int>& unmatched_indices,
                                             const std::vector<MeasRB>& meas_vec,
                                             const LidarExtrinsics& ex,
                                             int& promotions_this_frame)
{
  for (int k : unmatched_indices) {
    const auto& m = meas_vec[k];

    Eigen::Vector2d z_world; Eigen::Matrix2d Jrb;
    measRBToWorld(p.x, p.y, p.yaw, ex, m.r, m.b, z_world, Jrb);
    const Eigen::Matrix2d Rw = Jrb * (Eigen::Matrix2d() << m.r_var, 0, 0, m.b_var).finished()
                             * Jrb.transpose();

    int best_j = -1; double best_d2 = std::numeric_limits<double>::infinity();
    for (int j = 0; j < (int)p.map.size(); ++j) {
      const double d2 = worldMaha2(z_world, Rw, p.map[j]);
      if (d2 < best_d2) { best_d2 = d2; best_j = j; }
    }

    if (best_j >= 0 && best_d2 <= chi2_gate_world_) {
      auto& lm = p.map[best_j];

      Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
      R(0,0) = m.r_var; R(1,1) = m.b_var;
      Eigen::Matrix2d R_eff = (!lm.confirmed) ? (R * unconfirmed_R_scale_) : R;

      double r_h, b_h; Eigen::Matrix2d H;
      predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);
      Eigen::Vector2d nu; nu << (m.r - r_h), wrapToPi(m.b - b_h);

      const Eigen::Matrix2d S    = H * lm.Sigma * H.transpose() + R_eff;
      const Eigen::Matrix2d Sinv = S.inverse();
      const Eigen::Matrix2d K    = lm.Sigma * H.transpose() * Sinv;

      lm.mu    = lm.mu + K * nu;
      lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;
      lm.hits++; 
      if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

      continue;
    }

    // birth tracks
    int best_bt = -1; double best_d2_euclid = std::numeric_limits<double>::infinity();
    for (int bti = 0; bti < (int)p.births.size(); ++bti) {
      const double d2 = (z_world - p.births[bti].mu).squaredNorm();
      if (d2 < best_d2_euclid) { best_d2_euclid = d2; best_bt = bti; }
    }

    if (best_bt >= 0 && std::sqrt(best_d2_euclid) <= birth_promote_radius_) {
      auto& bt = p.births[best_bt];
      const Eigen::Matrix2d S = bt.Sigma + Rw;
      const Eigen::Matrix2d K = bt.Sigma * S.inverse();
      bt.mu = bt.mu + K * (z_world - bt.mu);
      bt.Sigma = (Eigen::Matrix2d::Identity() - K) * bt.Sigma;
      bt.hits++;
      bt.age = 0;
    } else {
      BirthTrack bt;
      bt.mu = z_world;
      bt.Sigma = Rw + 0.01 * Eigen::Matrix2d::Identity();
      bt.hits = 1;
      bt.age = 0;
      p.births.push_back(bt);
    }
  }

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
      }
      p.births.erase(p.births.begin() + bti);
    } else if (bt.age > birth_max_age_) {
      p.births.erase(p.births.begin() + bti);
    }
  }
}

void FastSLAM2::applyWeightAdjustments(Particle& p,
                                       const GlobalAssignment& assignment,
                                       const std::vector<bool>& /*lm_seen*/,
                                       const LidarExtrinsics& /*ex*/,
                                       int promotions_this_frame)
{
  if (promotions_this_frame > 0) {
    p.log_w -= new_landmark_penalty_ * promotions_this_frame;
  }
  if (assignment.average_compatibility > high_quality_threshold_ &&
      assignment.matches.size() >= 2) {
    double quality_bonus = high_quality_bonus_ * assignment.average_compatibility;
    p.log_w += quality_bonus;
  }
}

// ========== publish pose arrays ==========
void FastSLAM2::publishCoreOutputs(const ros::Time& t) {
  // Particles
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

  // Landmarks (best)
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

  // Births (best)
  geometry_msgs::PoseArray births;
  births.header.stamp = t;
  births.header.frame_id = map_frame_;
  for (const auto& bt : bestP.births) {
    geometry_msgs::Pose pose;
    pose.position.x = bt.mu.x();
    pose.position.y = bt.mu.y();
    pose.position.z = 0.0;
    pose.orientation.w = 1.0;
    births.poses.push_back(pose);
  }
  pub_births_posearray_.publish(births);

  // Odom
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
