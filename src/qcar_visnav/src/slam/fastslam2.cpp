#include "qcar_visnav/slam/fastslam2.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <Eigen/Dense>

// TF & msgs
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "qcar_visnav/slam/tf_utils.h"
#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/measurement_model.h"

using namespace qcar_visnav::slam;

// =============================================================================
// CONSTRUCTOR - SYSTEM INITIALIZATION
// =============================================================================

FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise()), assoc_(chi2_gate_)
{
  ROS_INFO("[FastSLAM2] Initializing FastSLAM 2.0 system...");

  // ---- COORDINATE FRAMES SETUP ----
  pnh.param("frames/map_frame",  map_frame_,  std::string("odom"));
  pnh.param("frames/odom_frame", odom_frame_, std::string("odom"));
  pnh.param("frames/base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("frames/lidar_frame", lidar_frame_, std::string("lidar"));

  // ---- ODOMETRY SOURCE CONFIGURATION ----
  bool use_ekf_odom = false;
  std::string truth_topic = "/odom";
  std::string ekf_topic   = "/qcar/ekf/odom";

  pnh.param("odometry/use_ekf", use_ekf_odom, false);
  pnh.param("odometry/truth_topic", truth_topic, truth_topic);
  pnh.param("odometry/ekf_topic",   ekf_topic,   ekf_topic);

  odom_topic_ = use_ekf_odom ? ekf_topic : truth_topic;
  ROS_INFO("[FastSLAM2] Using odometry from: %s", odom_topic_.c_str());

  // ---- PARTICLE FILTER CORE PARAMETERS ----
  pnh.param("particles", N_, 120);
  pnh.param("resample_neff_ratio", neff_ratio_, 0.4);
  pnh.param("association/chi2_gate", chi2_gate_, 9.21);

  // ---- LANDMARK MANAGEMENT PARAMETERS ----
  pnh.param("confirm_hits",    confirm_hits_,    2);
  pnh.param("min_new_lm_dist", min_new_lm_dist_, 0.35);
  pnh.param("merge_R_scale",   merge_R_scale_,   4.0);

  // Landmark prior variance (added when creating a brand-new landmark)
  pnh.param("landmark_prior_var", landmark_prior_var_, 0.25);

  // ---- INITIALIZATION BEHAVIOR ----
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, true);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.02);
  pnh.param("initial_spread/y",   spread_y_,   0.02);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.01);

  // ---- ODOMETRY BUFFER WINDOW ----
  pnh.param("odom_buffer_window_sec", odom_buffer_window_sec_, 2.0);

  // ---- MOTION MODEL SETUP ----
  MotionNoise q;
  pnh.param("motion_noise/sigma_x",   q.sigma_x,   0.02);
  pnh.param("motion_noise/sigma_y",   q.sigma_y,   0.02);
  pnh.param("motion_noise/sigma_yaw", q.sigma_yaw, 0.005);
  motion_ = MotionModel(q);

  // ---- DATA ASSOCIATION SETUP ----
  assoc_ = DataAssociation(chi2_gate_);

  // ---- PARTICLE INITIALIZATION ----
  P_.resize(N_);
  const double w0 = 1.0 / std::max(1, N_);
  for (int i = 0; i < N_; ++i) {
    P_[i].x = 0.0; P_[i].y = 0.0; P_[i].yaw = 0.0;  // default origin; may be reset below
    P_[i].weight = w0;
    P_[i].id = i;
    P_[i].map.clear();
  }

  if (seed_from_params_) {
    ROS_INFO("[FastSLAM2] Seeding particles from params: init(%.3f, %.3f, %.3f deg), spread(%.3f, %.3f, %.3f deg)",
             init_x_, init_y_, init_yaw_*180/M_PI, spread_x_, spread_y_, spread_yaw_*180/M_PI);
    initializeParticlesFrom(init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
  } else {
    ROS_INFO("[FastSLAM2] Will initialize from first odometry message.");
  }

  // ---- ROS PUBLISHERS SETUP ----
  pub_particles_            = nh.advertise<visualization_msgs::MarkerArray>("slam/particles", 1);
  pub_particles_posearray_  = nh.advertise<geometry_msgs::PoseArray>("slam/particles_pose", 1);
  pub_map_markers_          = nh.advertise<visualization_msgs::MarkerArray>("slam/map_markers", 1);
  pub_slam_odom_            = nh.advertise<nav_msgs::Odometry>("slam/odom", 1);
  pub_slam_odom_mean_       = nh.advertise<nav_msgs::Odometry>("slam/odom_mean", 1);

  // ---- ROS SUBSCRIBERS SETUP ----
  sub_odom_  = nh.subscribe<nav_msgs::Odometry>(odom_topic_, 50, &FastSLAM2::cbOdom, this);
  sub_cones_ = nh.subscribe<qcar_visnav::ConeArray>("/tracked_cones", 10, &FastSLAM2::cbCones, this);

  ROS_INFO("[FastSLAM2] Configuration complete:");
  ROS_INFO("  Frames: map=%s odom=%s base=%s lidar=%s",
           map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), lidar_frame_.c_str());
  ROS_INFO("  Particles: N=%d, neff_ratio=%.2f, chi2_gate=%.2f", N_, neff_ratio_, chi2_gate_);
  ROS_INFO("  Motion noise: sx=%.3f sy=%.3f syaw=%.4f", q.sigma_x, q.sigma_y, q.sigma_yaw);
  ROS_INFO("  Landmarks: confirm_hits=%d min_dist=%.2f m merge_scale=%.1f prior_var=%.3f",
           confirm_hits_, min_new_lm_dist_, merge_R_scale_, landmark_prior_var_);
  ROS_INFO("  Odom: topic=%s (twist assumed body-frame), buffer_window=%.2f s",
           odom_topic_.c_str(), odom_buffer_window_sec_);
}

// =============================================================================
// PARTICLE INITIALIZATION
// =============================================================================

void FastSLAM2::initializeParticlesFrom(double x, double y, double yaw,
                                        double sx, double sy, double syaw)
{
  ROS_INFO("[FastSLAM2] Initializing %d particles around pose (%.3f, %.3f, %.1f deg)",
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

  ROS_INFO("[FastSLAM2] Particles initialized successfully");
}

// =============================================================================
// MAIN PROCESSING LOOP (event-driven via callbacks)
// =============================================================================

void FastSLAM2::spinOnce() {
  // Currently unused; kept for future periodic tasks (cleanup, diagnostics).
}

// =============================================================================
/* ODOMETRY CALLBACK - stores velocity samples and optional first-message snap */
// =============================================================================

void FastSLAM2::cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  // Optional first-message snap to odom pose
  if (!snapped_to_first_odom_ && overwrite_with_odom_on_first_msg_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
    snapped_to_first_odom_ = true;
    ROS_INFO("[FastSLAM2] Snapped particles to first /odom pose: (%.3f, %.3f, %.1f deg)",
             p.x, p.y, yaw*180/M_PI);
  }

  // Fallback initialization if not seeding from params
  if (!particles_initialized_ && !seed_from_params_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
  }

  // Store twist sample (body-frame from gazebo_ros_diff_drive)
  OdomStamped o;
  o.t  = msg->header.stamp;
  o.vx = msg->twist.twist.linear.x;
  o.vy = msg->twist.twist.linear.y;
  o.r  = msg->twist.twist.angular.z;
  odom_buf_.push_back(o);

  // Maintain ring buffer using YAML-configured window
  while (!odom_buf_.empty() && (o.t - odom_buf_.front().t).toSec() > odom_buffer_window_sec_) {
    odom_buf_.pop_front();
  }
}

// =============================================================================
// VELOCITY INTERPOLATION FOR SMOOTH MOTION INTEGRATION
// =============================================================================

FastSLAM2::OdomStamped
FastSLAM2::interpTwist(const OdomStamped& a, const OdomStamped& b, const ros::Time& s) const {
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

// =============================================================================
// PARTICLE MOTION PROPAGATION
// =============================================================================

void FastSLAM2::propagateParticlesTo(const ros::Time& t) {
  if (!particles_initialized_ || odom_buf_.size() < 2) {
    return;
  }

  if (last_prop_stamp_.isZero()) last_prop_stamp_ = odom_buf_.front().t;
  if (t <= last_prop_stamp_) return;

  for (size_t i = 0; i + 1 < odom_buf_.size(); ++i) {
    const auto& a = odom_buf_[i];
    const auto& b = odom_buf_[i+1];

    // Overlap of [a.t, b.t] with [last_prop_stamp_, t]
    const ros::Time seg_start = std::max(a.t, last_prop_stamp_);
    const ros::Time seg_end   = std::min(b.t, t);
    if (seg_end <= seg_start) continue;

    // Interpolate endpoints and integrate with trapezoidal rule
    const auto ta = interpTwist(a, b, seg_start);
    const auto tb = interpTwist(a, b, seg_end);

    const double dt = (seg_end - seg_start).toSec();
    const double vx_avg = 0.5*(ta.vx + tb.vx);
    const double vy_avg = 0.5*(ta.vy + tb.vy);
    const double r_avg  = 0.5*(ta.r  + tb.r);

    for (auto& p : P_) {
      // Twists are already body-frame; motion model handles transform and noise.
      motion_.propagate(p, vx_avg, vy_avg, r_avg, dt);
    }
  }

  last_prop_stamp_ = t;
}

// =============================================================================
// CONE OBSERVATIONS CALLBACK - main SLAM update (RB model)
// =============================================================================

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

  // 1) Propagate particles to observation time
  propagateParticlesTo(msg->header.stamp);

  // 2) Get LiDAR extrinsics (base->lidar) at this stamp
  LidarExtrinsics ex;
  if (!lookupLidarExtrinsics(msg->header.stamp, ex) || !ex.valid) {
    return;
  }

  // 3) Build RB measurements directly from /tracked_cones (LiDAR frame)
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

  // 4) Core SLAM update (RB model with extrinsics)
  processMeasurementAt(msg->header.stamp, meas, ex);

  // 5) Publish visualization and estimates
  publishViz(msg->header.stamp);
}

// =============================================================================
// LANDMARK UTILITIES
// =============================================================================

int FastSLAM2::nearestLandmarkIdx(const Particle& p,
                                  const Eigen::Vector2d& z,
                                  double& out_dist_sq)
{
  int best = -1;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < (int)p.map.size(); ++i) {
    const auto& lm = p.map[i];
    const double d2 = (z - lm.mu).squaredNorm();
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  out_dist_sq = best_d2;
  return best;
}

// =============================================================================
// CORE MEASUREMENT UPDATE - the heart of FastSLAM 2.0 (RB model)
// =============================================================================

void FastSLAM2::processMeasurementAt(const ros::Time& /*t*/,
                                     const std::vector<MeasRB>& meas_vec,
                                     const LidarExtrinsics& ex)
{
  for (auto& p : P_) {
    double log_w_inc = 0.0;

    for (const auto& m : meas_vec) {
      // Measurement covariance in RB
      Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
      R(0,0) = m.r_var;
      R(1,1) = m.b_var;

      // 1) Data association with Mahalanobis gating in RB space
      AssocResult a = assoc_.associate(p, m, ex, R);

      if (a.lm_index >= 0) {
        // CASE A: Matched existing landmark -> EKF update in WORLD using RB Jacobian
        auto& lm = p.map[a.lm_index];

        const Eigen::Matrix2d Sinv = a.S.inverse();
        const Eigen::Matrix2d K    = lm.Sigma * a.H.transpose() * Sinv;

        lm.mu    = lm.mu + K * a.nu;
        lm.Sigma = (Eigen::Matrix2d::Identity() - K * a.H) * lm.Sigma;

        lm.hits++;
        if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

        const double detS = std::max(1e-18, a.S.determinant());
        const double expo = -0.5 * (a.nu.transpose() * Sinv * a.nu)(0,0);
        log_w_inc += expo - 0.5*std::log(detS) - std::log(2*M_PI);

      } else {
        // CASE B: No associated landmark -> check near-duplicate in WORLD by Euclidean distance
        Eigen::Vector2d z_world; Eigen::Matrix2d J;
        measRBToWorld(p.x, p.y, p.yaw, ex, m.r, m.b, z_world, J);

        double d2;
        const int j = nearestLandmarkIdx(p, z_world, d2);
        const double min_d2 = min_new_lm_dist_ * min_new_lm_dist_;

        if (j >= 0 && d2 < min_d2) {
          // MERGE with nearest landmark using inflated RB noise
          auto& lm = p.map[j];

          double r_h, b_h; Eigen::Matrix2d H;
          predictMeasurementRB(p.x, p.y, p.yaw, lm.mu, ex, r_h, b_h, H);

          Eigen::Vector2d nu;
          nu << (m.r - r_h), wrapToPi(m.b - b_h);

          const Eigen::Matrix2d Rm   = R * merge_R_scale_;
          const Eigen::Matrix2d S    = H * lm.Sigma * H.transpose() + Rm;
          const Eigen::Matrix2d Sinv = S.inverse();
          const Eigen::Matrix2d K    = lm.Sigma * H.transpose() * Sinv;

          lm.mu    = lm.mu + K * nu;
          lm.Sigma = (Eigen::Matrix2d::Identity() - K * H) * lm.Sigma;

          lm.hits++;
          if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

          const double detS = std::max(1e-18, S.determinant());
          const double expo = -0.5 * (nu.transpose() * Sinv * nu)(0,0);
          log_w_inc += expo - 0.5*std::log(detS) - std::log(2*M_PI);

        } else {
          // CREATE a brand-new landmark in WORLD from RB
          Landmark lm;
          Eigen::Matrix2d Jrb;
          measRBToWorld(p.x, p.y, p.yaw, ex, m.r, m.b, lm.mu, Jrb);
          const Eigen::Matrix2d Rw = Jrb * R * Jrb.transpose();

          lm.Sigma = Rw + Eigen::Matrix2d::Identity() * landmark_prior_var_;
          lm.hits  = 1;
          lm.confirmed = (confirm_hits_ <= 1);
          p.map.push_back(lm);
        }
      }
    }

    // Accumulate log-likelihood into log(weight) safely
    p.weight = std::log(std::max(1e-300, p.weight)) + log_w_inc;
  }

  // Normalize weights using log-sum-exp trick
  double max_logw = -1e300;
  for (const auto& p : P_) max_logw = std::max(max_logw, p.weight);

  double sum_w = 0.0;
  for (auto& p : P_) {
    p.weight = std::exp(p.weight - max_logw);
    sum_w += p.weight;
  }

  if (sum_w <= 0.0) {
    const double w0 = 1.0 / std::max<int>(1, P_.size());
    for (auto& p : P_) p.weight = w0;
  } else {
    for (auto& p : P_) p.weight /= sum_w;
  }

  // Adaptive resampling
  double inv_neff = 0.0;
  for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;

  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_, rng_);
    ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Resampled particles: Neff=%.1f < %.1f",
                      neff, neff_ratio_ * P_.size());
  }

  // Cache best particle and weighted mean
  best_idx_ = 0; double bestw = -1.0;
  for (int i = 0; i < (int)P_.size(); ++i) {
    if (P_[i].weight > bestw) { bestw = P_[i].weight; best_idx_ = i; }
  }

  double cx=0, cy=0, cyaw_c=0, cyaw_s=0;
  for (const auto& p : P_) {
    cx += p.weight * p.x;
    cy += p.weight * p.y;
    cyaw_c += p.weight * std::cos(p.yaw);
    cyaw_s += p.weight * std::sin(p.yaw);
  }
  mean_x_ = cx; mean_y_ = cy; mean_yaw_ = std::atan2(cyaw_s, cyaw_c);
}

// =============================================================================
// VISUALIZATION AND OUTPUT PUBLISHING
// =============================================================================

void FastSLAM2::publishViz(const ros::Time& t) {
  // Publish particle arrows
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

    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.05;  // small lift

    tf2::Quaternion q; q.setRPY(0, 0, p.yaw);
    m.pose.orientation = tf2::toMsg(q);

    m.scale.x = 0.3;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.color.a = 0.3 + 0.7 * (p.weight * P_.size());
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;

    particle_markers.markers.push_back(m);
  }
  pub_particles_.publish(particle_markers);

  // Publish particle pose array
  geometry_msgs::PoseArray poses;
  poses.header.stamp = t;
  poses.header.frame_id = map_frame_;
  for (const auto& p : P_) {
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,p.yaw);
    pose.orientation = tf2::toMsg(q);
    poses.poses.push_back(pose);
  }
  pub_particles_posearray_.publish(poses);

  // Publish landmark markers for best particle
  visualization_msgs::MarkerArray landmark_markers;
  const auto& bestP = P_[std::max(0, std::min<int>(best_idx_, (int)P_.size()-1))];
  int lm_id = 0;
  size_t confirmed_count = 0;

  for (const auto& lm : bestP.map) {
    if (!lm.confirmed) continue;
    confirmed_count++;

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
    m.color.a = 1.0;
    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;

    landmark_markers.markers.push_back(m);
  }
  pub_map_markers_.publish(landmark_markers);

  // Publish odometry estimates
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

  ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Published viz: particles=%zu landmarks_confirmed=%zu (best_particle=%d)",
                    P_.size(), confirmed_count, best_idx_);
}
