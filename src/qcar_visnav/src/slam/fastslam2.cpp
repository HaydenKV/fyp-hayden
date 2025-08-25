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

using namespace qcar_visnav::slam;

FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise()), assoc_(chi2_gate_)
{
  // ---- Frames (run entirely in ODOM) ----
  pnh.param("frames/map_frame",  map_frame_,  std::string("odom"));
  pnh.param("frames/odom_frame", odom_frame_, std::string("odom"));
  pnh.param("frames/base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("frames/lidar_frame", lidar_frame_, std::string("lidar"));

  // ---- Core params ----
  pnh.param("particles", N_, 120);
  pnh.param("resample_neff_ratio", neff_ratio_, 0.4);
  pnh.param("association/chi2_gate", chi2_gate_, 16.27);

  // landmark management
  pnh.param("confirm_hits",      confirm_hits_,      2);
  pnh.param("min_new_lm_dist",   min_new_lm_dist_,   0.35);
  pnh.param("merge_R_scale",     merge_R_scale_,     4.0);

  // init behavior
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, true);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.02);
  pnh.param("initial_spread/y",   spread_y_,   0.02);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.01);

  // twist semantics
  pnh.param("odom_twist_in_world", odom_twist_in_world_, false);

  // ---- Motion noise ----
  MotionNoise q;
  pnh.param("motion_noise/sigma_x",   q.sigma_x,   0.02);
  pnh.param("motion_noise/sigma_y",   q.sigma_y,   0.02);
  pnh.param("motion_noise/sigma_yaw", q.sigma_yaw, 0.005);
  motion_ = MotionModel(q);

  // ---- DA gate ----
  assoc_ = DataAssociation(chi2_gate_);

  // ---- Allocate particles ----
  P_.resize(N_);
  const double w0 = 1.0 / std::max(1, N_);
  for (int i = 0; i < N_; ++i) {
    P_[i].x = 0.0; P_[i].y = 0.0; P_[i].yaw = 0.0;
    P_[i].weight = w0;
    P_[i].id = i;
  }

  if (seed_from_params_) {
    ROS_INFO("[FastSLAM2] Seeding from params: init(%.3f, %.3f, %.3f rad), spread(%.3f, %.3f, %.3f)",
             init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
    initializeParticlesFrom(init_x_, init_y_, init_yaw_, spread_x_, spread_y_, spread_yaw_);
  } else {
    ROS_INFO("[FastSLAM2] Will initialize from first /odom pose.");
  }

  // ---- Publishers ----
  pub_particles_            = nh.advertise<visualization_msgs::MarkerArray>("slam/particles", 1);
  pub_particles_posearray_  = nh.advertise<geometry_msgs::PoseArray>("slam/particles_pose", 1);
  pub_map_markers_          = nh.advertise<visualization_msgs::MarkerArray>("slam/map_markers", 1);
  pub_slam_odom_            = nh.advertise<nav_msgs::Odometry>("slam/odom", 1);
  pub_slam_odom_mean_       = nh.advertise<nav_msgs::Odometry>("slam/odom_mean", 1);

  // ---- Subscribers ----
  sub_odom_  = nh.subscribe<nav_msgs::Odometry>("/odom", 50, &FastSLAM2::cbOdom, this);              // TRUE states
  sub_cones_ = nh.subscribe<qcar_visnav::ConeArray>("/tracked_cones", 10, &FastSLAM2::cbCones, this);

  ROS_INFO("[FastSLAM2] frames map=%s odom=%s base=%s lidar=%s  N=%d gate=%.2f neff=%.2f "
           "motion_noise(%.3f,%.3f,%.4f) confirm_hits=%d min_new_lm_dist=%.2f merge_R_scale=%.1f twist_is_world=%s",
           map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), lidar_frame_.c_str(),
           N_, chi2_gate_, neff_ratio_, q.sigma_x, q.sigma_y, q.sigma_yaw,
           confirm_hits_, min_new_lm_dist_, merge_R_scale_,
           odom_twist_in_world_ ? "true" : "false");
}

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

  ROS_INFO("[FastSLAM2] Particles initialized at (%.3f, %.3f, %.1f deg), spread(%.3f, %.3f, %.3f)",
           x, y, yaw * 180.0/M_PI, sx, sy, syaw);
}

void FastSLAM2::spinOnce() {}

void FastSLAM2::cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  // Optional: snap initial seed to first /odom pose (stays in odom frame)
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

  // store twist sample (truth)
  OdomStamped o;
  o.t  = msg->header.stamp;
  o.vx = msg->twist.twist.linear.x;
  o.vy = msg->twist.twist.linear.y;
  o.r  = msg->twist.twist.angular.z;
  odom_buf_.push_back(o);

  // Trim to ~2s
  const double window = 2.0;
  while (!odom_buf_.empty() && (o.t - odom_buf_.front().t).toSec() > window) {
    odom_buf_.pop_front();
  }
}

FastSLAM2::OdomStamped
FastSLAM2::interpTwist(const OdomStamped& a, const OdomStamped& b, const ros::Time& s) const {
  const double seg = (b.t - a.t).toSec();
  if (seg <= 0.0) return a;
  const double frac = (s - a.t).toSec() / seg;
  OdomStamped r;
  r.t  = s;
  r.vx = (1.0 - frac)*a.vx + frac*b.vx;
  r.vy = (1.0 - frac)*a.vy + frac*b.vy;
  r.r  = (1.0 - frac)*a.r  + frac*b.r;
  return r;
}

void FastSLAM2::cbCones(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!particles_initialized_) {
    ROS_WARN_THROTTLE(2.0, "[FastSLAM2] cones received before particles initialized; ignoring");
    return;
  }

  // A) propagate particles to detection time (incremental)
  propagateParticlesTo(msg->header.stamp);

  // B) Convert polar -> odom Cartesian
  std::vector<Eigen::Vector2d> zs;
  std::vector<Eigen::Matrix2d> Rs;
  zs.reserve(msg->cones.size());
  Rs.reserve(msg->cones.size());

  for (const auto& c : msg->cones) {
    geometry_msgs::Point pt_odom;
    if (!transformConeToOdom(tfbuf_, msg->header, c.range, c.bearing, odom_frame_, pt_odom))
      continue;

    Eigen::Vector2d z(pt_odom.x, pt_odom.y);

    // polar -> Cartesian covariance
    Eigen::Matrix2d J;
    J << std::cos(c.bearing), -c.range*std::sin(c.bearing),
         std::sin(c.bearing),  c.range*std::cos(c.bearing);

    Eigen::Matrix2d Rp = Eigen::Matrix2d::Zero();
    Rp(0,0) = std::max<double>(1e-8,  c.r_var);
    Rp(1,1) = std::max<double>(1e-12, c.bearing_var);

    Eigen::Matrix2d R = J * Rp * J.transpose();

    zs.push_back(z);
    Rs.push_back(R);
  }

  if (zs.empty()) return;

  // C) update + resampling
  processMeasurementAt(msg->header.stamp, zs, Rs);

  // D) publish
  publishViz(msg->header.stamp);
}

void FastSLAM2::propagateParticlesTo(const ros::Time& t) {
  if (!particles_initialized_ || odom_buf_.size() < 2) return;

  if (last_prop_stamp_.isZero()) last_prop_stamp_ = odom_buf_.front().t;
  if (t <= last_prop_stamp_) return;

  // integrate only [last_prop_stamp_, t]
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
      double vx_b = vx_avg, vy_b = vy_avg;
      if (odom_twist_in_world_) {
        worldToBody(p.yaw, vx_avg, vy_avg, vx_b, vy_b);
      }
      motion_.propagate(p, vx_b, vy_b, r_avg, dt);
    }
  }

  last_prop_stamp_ = t;
}

// nearest landmark by Euclidean distance; returns index or -1
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

void FastSLAM2::processMeasurementAt(const ros::Time& /*t*/,
                                     const std::vector<Eigen::Vector2d>& zs,
                                     const std::vector<Eigen::Matrix2d>& Rs)
{
  for (auto& p : P_) {
    double log_w_inc = 0.0;

    for (size_t i = 0; i < zs.size(); ++i) {
      const Eigen::Vector2d& z = zs[i];
      const Eigen::Matrix2d& R = Rs[i];

      // Mahalanobis association
      AssocResult a = assoc_.associate(p, z, R);

      if (a.lm_index >= 0) {
        auto& lm = p.map[a.lm_index];
        const Eigen::Vector2d nu = z - lm.mu;
        const Eigen::Matrix2d S  = lm.Sigma + R;
        const Eigen::Matrix2d K  = lm.Sigma * S.inverse();
        lm.mu     = lm.mu + K * nu;
        lm.Sigma  = (Eigen::Matrix2d::Identity() - K) * lm.Sigma;

        lm.hits++;
        if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

        const double detS = std::max(1e-18, S.determinant());
        const double expo = -0.5 * nu.transpose() * S.inverse() * nu;
        log_w_inc += expo - 0.5*std::log(detS) - std::log(2*M_PI);

      } else {
        // Not associated: check for near-duplicate before spawn
        double d2;
        const int j = nearestLandmarkIdx(p, z, d2);
        const double min_d2 = min_new_lm_dist_ * min_new_lm_dist_;

        if (j >= 0 && d2 < min_d2) {
          // MERGE into nearest (soft update with inflated R)
          auto& lm = p.map[j];
          const Eigen::Matrix2d Rm = R * merge_R_scale_;
          const Eigen::Vector2d nu = z - lm.mu;
          const Eigen::Matrix2d S  = lm.Sigma + Rm;
          const Eigen::Matrix2d K  = lm.Sigma * S.inverse();
          lm.mu     = lm.mu + K * nu;
          lm.Sigma  = (Eigen::Matrix2d::Identity() - K) * lm.Sigma;

          lm.hits++;
          if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

          const double detS = std::max(1e-18, S.determinant());
          const double expo = -0.5 * nu.transpose() * S.inverse() * nu;
          log_w_inc += expo - 0.5*std::log(detS) - std::log(2*M_PI);

        } else {
          // Truly new landmark
          Landmark lm;
          lm.mu = z;
          lm.Sigma = R + Eigen::Matrix2d::Identity() * 0.25; // inflate on init
          lm.hits = 1;
          lm.confirmed = (confirm_hits_ <= 1);
          p.map.push_back(lm);
        }
      }
    }

    // weight bookkeeping (log domain)
    p.weight = std::log(std::max(1e-300, p.weight)) + log_w_inc;
  }

  // normalize weights (log -> linear)
  double max_logw = -1e300;
  for (const auto& p : P_) max_logw = std::max(max_logw, p.weight);
  double sum_w = 0.0;
  for (auto& p : P_) { p.weight = std::exp(p.weight - max_logw); sum_w += p.weight; }
  if (sum_w <= 0.0) {
    const double w0 = 1.0 / std::max<int>(1, P_.size());
    for (auto& p : P_) p.weight = w0;
  } else {
    for (auto& p : P_) p.weight /= sum_w;
  }

  // resample if needed
  double inv_neff = 0.0;
  for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;
  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_);
    ROS_INFO_THROTTLE(1.0, "[FS2] resampled, Neff=%.1f", neff);
  }

  // cache best & mean
  best_idx_ = 0; double bestw = -1.0;
  for (int i = 0; i < (int)P_.size(); ++i) if (P_[i].weight > bestw) { bestw = P_[i].weight; best_idx_ = i; }

  double cx=0, cy=0, cyaw_c=0, cyaw_s=0;
  for (const auto& p : P_) {
    cx += p.weight * p.x;
    cy += p.weight * p.y;
    cyaw_c += p.weight * std::cos(p.yaw);
    cyaw_s += p.weight * std::sin(p.yaw);
  }
  mean_x_ = cx; mean_y_ = cy; mean_yaw_ = std::atan2(cyaw_s, cyaw_c);
}

void FastSLAM2::publishViz(const ros::Time& t) {
  // particles (arrows)
  visualization_msgs::MarkerArray mks;
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
    m.pose.position.z = 0.05;

    tf2::Quaternion q; q.setRPY(0, 0, p.yaw);
    m.pose.orientation = tf2::toMsg(q);

    m.scale.x = 0.3;
    m.scale.y = 0.05;
    m.scale.z = 0.05;
    m.color.a = 0.5;
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;
    mks.markers.push_back(m);
  }
  pub_particles_.publish(mks);

  // particles posearray
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

  // landmarks (best particle)
  visualization_msgs::MarkerArray lmks;
  const auto& bestP = P_[ std::max(0, std::min<int>(best_idx_, (int)P_.size()-1)) ];
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
    lmks.markers.push_back(m);
  }
  pub_map_markers_.publish(lmks);
  ROS_INFO_THROTTLE(1.0, "[FS2] confirmed landmarks: %zu", confirmed_count);

  // odometry (best + mean)
  nav_msgs::Odometry odom_best;
  odom_best.header.stamp = t;
  odom_best.header.frame_id = map_frame_;
  odom_best.child_frame_id  = base_frame_;
  odom_best.pose.pose.position.x = bestP.x;
  odom_best.pose.pose.position.y = bestP.y;
  { tf2::Quaternion q; q.setRPY(0,0,bestP.yaw);
    odom_best.pose.pose.orientation = tf2::toMsg(q); }
  pub_slam_odom_.publish(odom_best);

  nav_msgs::Odometry odom_mean = odom_best;
  odom_mean.pose.pose.position.x = mean_x_;
  odom_mean.pose.pose.position.y = mean_y_;
  { tf2::Quaternion q; q.setRPY(0,0,mean_yaw_);
    odom_mean.pose.pose.orientation = tf2::toMsg(q); }
  pub_slam_odom_mean_.publish(odom_mean);
}
