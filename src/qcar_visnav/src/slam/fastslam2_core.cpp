#include "qcar_visnav/slam/fastslam2_core.h"
#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/motion_model.h"
#include "qcar_visnav/slam/landmark_ekf.h"
#include "qcar_visnav/slam/data_assoc.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>
#include <algorithm>

using namespace qcar_visnav::slam;

// Add near the top (tunable floors):
static constexpr double R_RANGE_VAR_FLOOR  = 0.10 * 0.10;   // (m^2) >= 10 cm std on range in polar domain
static constexpr double R_BEAR_VAR_FLOOR   = (3.0*M_PI/180.0)*(3.0*M_PI/180.0); // (rad^2) >= 3 deg on bearing

// Cartesian noise inflation (extra safety in odom XY)
static constexpr double R_CART_INFLATE_XY  = 0.20 * 0.20;   // add 20 cm std^2 on top (per axis)

// New landmark initial covariance (cartesian, generous)
static constexpr double NEW_LM_INIT_STD    = 0.60;          // 60 cm std per axis

static inline std_msgs::ColorRGBA colorFor(int idx) {
  std_msgs::ColorRGBA c; c.a=0.9f;
  switch (idx % 5) {
    case 0: c.r=1.0f; c.g=0.4f; c.b=0.0f; break;
    case 1: c.r=0.2f; c.g=0.8f; c.b=0.2f; break;
    case 2: c.r=0.2f; c.g=0.6f; c.b=1.0f; break;
    case 3: c.r=1.0f; c.g=0.2f; c.b=0.6f; break;
    default: c.r=1.0f; c.g=1.0f; c.b=0.2f; break;
  }
  return c;
}

FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh)
{
  // topics
  pnh_.param("tracked_topic", P_.topic_tracked, P_.topic_tracked);
  pnh_.param("odom_topic",    P_.topic_odom,    P_.topic_odom);
  pnh_.param("pub_particles", P_.pub_particles, P_.pub_particles);
  pnh_.param("pub_landmarks", P_.pub_landmarks, P_.pub_landmarks);
  pnh_.param("pub_weights",   P_.pub_weights,   P_.pub_weights);

  // frames
  pnh_.param("map_frame",  P_.map_frame,  P_.map_frame);
  pnh_.param("odom_frame", P_.odom_frame, P_.odom_frame);
  pnh_.param("base_frame", P_.base_frame, P_.base_frame);
  pnh_.param("lidar_frame",P_.lidar_frame,P_.lidar_frame);

  // core params
  int N= P_.particles; pnh_.param("particles", N, N); P_.particles = std::max(1, N);
  pnh_.param("resample_neff_ratio", P_.neff_ratio, P_.neff_ratio);
  pnh_.param("chi2_gate", P_.chi2_gate, P_.chi2_gate);
  pnh_.param("new_landmark_likelihood_min", P_.new_lm_lik_min, P_.new_lm_lik_min);

  double m0=P_.meas_noise_xy.x(), m1=P_.meas_noise_xy.y();
  pnh_.param("meas_noise_xy_x", m0, m0);
  pnh_.param("meas_noise_xy_y", m1, m1);
  P_.meas_noise_xy = Eigen::Vector2d(m0,m1);

  double nv=P_.odom_v_std, nw=P_.odom_yawrate_std;
  // allow legacy vector param [v, yawrate] or separated scalars
  pnh_.param("odom_noise_vr", nv, nv);
  pnh_.param("odom_v_std", nv, nv);
  pnh_.param("odom_yawrate_std", nw, nw);
  P_.odom_v_std = nv; P_.odom_yawrate_std = nw;

  pnh_.param("miss_in_fov_penalty", P_.miss_in_fov_penalty, P_.miss_in_fov_penalty);
  pnh_.param("color_mismatch_penalty", P_.color_mismatch_penalty, P_.color_mismatch_penalty);

  // simple LiDAR->base 2D extrinsics (optional)
  double t_bl_x = 0.0, t_bl_y = 0.0, yaw_bl_deg = 0.0;
  pnh_.param("lidar_to_base_x", t_bl_x, t_bl_x);
  pnh_.param("lidar_to_base_y", t_bl_y, t_bl_y);
  pnh_.param("lidar_to_base_yaw_deg", yaw_bl_deg, yaw_bl_deg);
  P_.t_bl = Eigen::Vector2d(t_bl_x, t_bl_y);
  P_.yaw_bl = yaw_bl_deg * M_PI / 180.0;




  // subs/pubs
  sub_cones_ = nh_.subscribe<qcar_visnav::ConeArray>(P_.topic_tracked, 5, &FastSLAM2::conesCb, this);
  sub_odom_  = nh_.subscribe<nav_msgs::Odometry>(P_.topic_odom, 100, &FastSLAM2::odomCb, this);
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>(P_.pub_particles, 1, false);
  pub_landmarks_ = nh_.advertise<visualization_msgs::MarkerArray>(P_.pub_landmarks, 1, false);
  pub_weights_   = nh_.advertise<std_msgs::Float32MultiArray>(P_.pub_weights, 1, false);

  ROS_INFO_STREAM("[fastslam2_core] topics: tracked="<<P_.topic_tracked
    << " odom="<<P_.topic_odom<< " pubs=["<<P_.pub_particles<<","<<P_.pub_landmarks<<","<<P_.pub_weights<<"]"
    << " frames odom="<<P_.odom_frame<<" base="<<P_.base_frame<<" lidar="<<P_.lidar_frame
    << " particles="<<P_.particles);

  ensureInitParticles();
}

void FastSLAM2::ensureInitParticles() {
  if (!particles_.empty()) return;
  particles_.resize(P_.particles);
  const double w = 1.0 / (double)P_.particles;
  for (auto& p : particles_) { p.pose.setZero(); p.weight = w; p.map.clear(); }
}

void FastSLAM2::spinOnce() {
  // All compute is event-driven for now (odom + cones callbacks).
  // This hook is kept in case we later add time-driven publishing/diag.
}

void FastSLAM2::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  // Store latest for propagation
  last_v_ = msg->twist.twist.linear.x;
  last_yawrate_ = msg->twist.twist.angular.z;
  last_odom_stamp_ = msg->header.stamp;
  have_odom_ = true;
}

Eigen::Matrix2d FastSLAM2::polarCovToCart(double r, double th, double r_var, double th_var) const {
  // Floor polar variances (ConeArray can be too confident or zeros)
  const double rv  = std::max(R_RANGE_VAR_FLOOR,  r_var);
  const double tv  = std::max(R_BEAR_VAR_FLOOR,   th_var);

  // Jacobian d[x,y]/d[r,th]
  Eigen::Matrix2d J;
  J << std::cos(th), -r*std::sin(th),
       std::sin(th),  r*std::cos(th);

  Eigen::Matrix2d Rp = Eigen::Matrix2d::Zero();
  Rp(0,0) = rv;
  Rp(1,1) = tv;

  Eigen::Matrix2d Rxy = J * Rp * J.transpose();

  // Inflate in Cartesian (accounts for tracker model mismatch etc.)
  Rxy(0,0) += R_CART_INFLATE_XY;
  Rxy(1,1) += R_CART_INFLATE_XY;
  return Rxy;
}


Eigen::Vector2d FastSLAM2::lidarPolarToOdomXY(const Eigen::Vector3d& base_pose,
                                               double r, double th) const
{
  // LiDAR polar -> LiDAR Cartesian
  const Eigen::Vector2d p_l(r*std::cos(th), r*std::sin(th));
  // LiDAR -> Base
  const Eigen::Matrix2d R_bl = Rot2(P_.yaw_bl);
  const Eigen::Vector2d p_b = R_bl * p_l + P_.t_bl;
  // Base -> Odom
  const Eigen::Matrix2d R_ob = Rot2(base_pose.z());
  const Eigen::Vector2d p_o = R_ob * p_b + base_pose.head<2>();
  return p_o;
}

void FastSLAM2::integrateOdom(double stamp_sec) {
  if (!have_odom_) return;
  const double dt = std::max(0.0, stamp_sec - last_odom_stamp_.toSec());
  if (dt <= 0.0) return;

  // noise
  std::normal_distribution<double> Nv(0.0, P_.odom_v_std);
  std::normal_distribution<double> Nw(0.0, P_.odom_yawrate_std);

  for (auto& p : particles_) {
    double v = last_v_ + Nv(gen_);
    double w = last_yawrate_ + Nw(gen_);
    propagate_pose(p.pose, v, w, dt);
  }
  // advance reference
  last_odom_stamp_ = ros::Time(stamp_sec);
}

void FastSLAM2::conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (particles_.empty()) ensureInitParticles();

  // 1) Propagate to measurement time using last odom
  integrateOdom(msg->header.stamp.toSec());

  // 2) Build detections (LiDAR polar)
  struct Det { double r, th, r_var, th_var; int color; double color_conf; };
  std::vector<Det> dets; dets.reserve(msg->cones.size());
  for (const auto& c : msg->cones) {
    if (!std::isfinite(c.range) || !std::isfinite(c.bearing)) continue;
    dets.push_back({c.range, c.bearing, std::max(1e-8,c.r_var), std::max(1e-10,c.bearing_var),
                    (int)c.color, c.color_conf});
  }
  if (dets.empty()) return;

  // 3) Per-particle update & weight increment
  double max_logw = -1e100;
  std::vector<double> logw(particles_.size(), 0.0);

  for (size_t ip=0; ip<particles_.size(); ++ip) {
    auto& Pk = particles_[ip];

    // collect landmark means/covs for association
    std::vector<Eigen::Vector2d> mus; mus.reserve(Pk.map.size());
    std::vector<Eigen::Matrix2d> covs; covs.reserve(Pk.map.size());
    for (const auto& lm : Pk.map) { mus.push_back(lm.mu); covs.push_back(lm.Sigma); }

    int matched = 0;

    for (const auto& d : dets) {
      // detection in ODOM using particle pose
      const Eigen::Vector2d z_o = lidarPolarToOdomXY(Pk.pose, d.r, d.th);
      const Eigen::Matrix2d R_o = polarCovToCart(d.r, d.th, d.r_var, d.th_var);

      // NN + gate
      auto [j, d2] = nn_gated(mus, covs, z_o, R_o, P_.chi2_gate);

      // likelihood increment (log domain), basic Gaussian with H=I
      auto log_gauss = [&](double d2v, const Eigen::Matrix2d& S){
        double logdet = std::log(std::max(1e-12, S.determinant()));
        return -0.5*(d2v + std::log( (2*M_PI)*(2*M_PI) ) + logdet);
      };

      if (j >= 0) {
        // update landmark j
        ekf_update_landmark(Pk.map[j].mu, Pk.map[j].Sigma, z_o, R_o);
        Pk.map[j].hits = std::min(Pk.map[j].hits+1, 1000000);
        Pk.map[j].misses = 0;

        // weight increment by innovation likelihood (using post S ~ Sigma+R)
        const Eigen::Matrix2d S = Pk.map[j].Sigma + R_o;
        logw[ip] += log_gauss(d2, S);

        // optional color penalty if color known & mismatched
        if (d.color > 0 && Pk.map[j].color > 0 && d.color != Pk.map[j].color) {
          logw[ip] += std::log(std::max(1e-6, P_.color_mismatch_penalty));
        } else if (Pk.map[j].color == 0 && d.color > 0 && d.color_conf > 0.6) {
          // adopt color on first confident observation
          Pk.map[j].color = d.color;
          Pk.map[j].color_conf = d.color_conf;
        }

        ++matched;
      } else {
        // no association -> maybe new landmark if likelihood low enough elsewhere
        // initialize new landmark with z_o and a reasonable initial covariance
        Landmark L;
        L.mu = z_o;
        // start with measurement cov inflated a bit
        // generous init covariance to favor matching on next frames
        L.Sigma = Eigen::Matrix2d::Identity() * (NEW_LM_INIT_STD * NEW_LM_INIT_STD);

        L.hits = 1; L.misses = 0;
        L.color = (d.color>0 && d.color_conf>0.6) ? d.color : 0;
        L.color_conf = (L.color>0) ? d.color_conf : 0.0;
        Pk.map.push_back(L);

        // expand association arrays for consistency
        mus.push_back(L.mu);
        covs.push_back(L.Sigma);

        // give a conservative likelihood bump for accepted new LM
        logw[ip] += std::log(std::max(1e-6, P_.new_lm_lik_min));
      }
    }

    // mild penalty if too few associations (robustness)
    if (matched == 0) {
      logw[ip] += std::log(std::max(1e-6, P_.miss_in_fov_penalty));
    }

    max_logw = std::max(max_logw, logw[ip]);
  }

  // 4) Normalize weights (prevent underflow with log-max trick)
  double sumw = 0.0;
  for (size_t i=0;i<particles_.size();++i) {
    particles_[i].weight *= std::exp(logw[i] - max_logw);
    sumw += particles_[i].weight;
  }
  if (sumw <= 0.0) { // fallback
    const double w = 1.0 / (double)particles_.size();
    for (auto& p: particles_) p.weight = w;
  } else {
    for (auto& p: particles_) p.weight /= sumw;
  }

  // 5) Resample if Neff low
  const double Neff = neff(particles_);
  if (Neff < P_.neff_ratio * particles_.size()) {
    particles_ = systematic_resample(particles_, gen_);
  }

  // 6) Publish visuals
  publishParticles(msg->header.stamp);
  publishLandmarks(msg->header.stamp);
  publishWeights(msg->header.stamp);
}

void FastSLAM2::publishParticles(const ros::Time& t) {
  geometry_msgs::PoseArray pa;
  pa.header.stamp = t;
  pa.header.frame_id = P_.odom_frame;
  pa.poses.reserve(particles_.size());
  for (const auto& p : particles_) {
    geometry_msgs::Pose pose;
    pose.position.x = p.pose.x();
    pose.position.y = p.pose.y();
    pose.position.z = 0.0;
    const double cy = std::cos(p.pose.z()*0.5), sy = std::sin(p.pose.z()*0.5);
    pose.orientation.w = cy;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = sy;
    pa.poses.push_back(pose);
  }
  if (pub_particles_) pub_particles_.publish(pa);
}

void FastSLAM2::publishWeights(const ros::Time& t) {
  if (!pub_weights_) return;
  std_msgs::Float32MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].label = "weights";
  msg.layout.dim[0].size = particles_.size();
  msg.layout.dim[0].stride = particles_.size();
  msg.data.reserve(particles_.size());
  for (const auto& p : particles_) msg.data.push_back(static_cast<float>(p.weight));
  pub_weights_.publish(msg);
}

void FastSLAM2::publishLandmarks(const ros::Time& t) {
  // Use highest-weight particle's map for visualization
  int best = 0; double bestw = -1.0;
  for (int i=0;i<(int)particles_.size();++i) if (particles_[i].weight > bestw) { bestw = particles_[i].weight; best = i; }
  const auto& M = particles_[best].map;

  visualization_msgs::MarkerArray arr;
  // delete-all to keep RViz tidy
  {
    visualization_msgs::Marker m;
    m.header.stamp = t;
    m.header.frame_id = P_.odom_frame;
    m.ns = "landmarks";
    m.id = 0;
    m.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(m);
  }

  // spheres
  int id = 1;
  for (size_t i=0;i<M.size(); ++i) {
    const auto& lm = M[i];
    visualization_msgs::Marker s;
    s.header.stamp = t;
    s.header.frame_id = P_.odom_frame;
    s.ns = "landmarks";
    s.id = id++;
    s.type = visualization_msgs::Marker::SPHERE;
    s.action = visualization_msgs::Marker::ADD;
    s.pose.position.x = lm.mu.x();
    s.pose.position.y = lm.mu.y();
    s.pose.position.z = 0.05;
    s.pose.orientation.w = 1.0;
    s.scale.x = 0.24; s.scale.y = 0.24; s.scale.z = 0.24;
    s.color = colorFor(lm.color>0 ? lm.color : (int)i);
    s.lifetime = ros::Duration(0.0);
    arr.markers.push_back(s);
  }

  if (pub_landmarks_) pub_landmarks_.publish(arr);
}
