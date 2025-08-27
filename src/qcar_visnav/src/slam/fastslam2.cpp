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

// =============================================================================
// CONSTRUCTOR - SYSTEM INITIALIZATION
// =============================================================================

FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : tfl_(tfbuf_), motion_(MotionNoise()), assoc_(chi2_gate_)
{
  ROS_INFO("[FastSLAM2] Initializing FastSLAM 2.0 system...");
  
  // ---- COORDINATE FRAMES SETUP ----
  // All computation happens in ODOM frame for stability (world-fixed)
  pnh.param("frames/map_frame",  map_frame_,  std::string("odom"));
  pnh.param("frames/odom_frame", odom_frame_, std::string("odom"));
  pnh.param("frames/base_frame", base_frame_, std::string("base_footprint"));
  pnh.param("frames/lidar_frame", lidar_frame_, std::string("lidar"));
  
  // ---- ODOMETRY SOURCE CONFIGURATION ----
  bool use_ekf_odom = false;
  std::string truth_topic = "/odom";
  std::string ekf_topic = "/qcar/ekf/odom";
  
  pnh.param("odometry/use_ekf", use_ekf_odom, false);
  pnh.param("odometry/truth_topic", truth_topic, truth_topic);
  pnh.param("odometry/ekf_topic", ekf_topic, ekf_topic);
  
  odom_topic_ = use_ekf_odom ? ekf_topic : truth_topic;
  ROS_INFO("[FastSLAM2] Using odometry from: %s", odom_topic_.c_str());

  // ---- PARTICLE FILTER CORE PARAMETERS ----
  pnh.param("particles", N_, 120);                    // Number of hypotheses
  pnh.param("resample_neff_ratio", neff_ratio_, 0.4); // Diversity maintenance threshold
  pnh.param("association/chi2_gate", chi2_gate_, 16.27); // Association gate (loose initially)

  // ---- LANDMARK MANAGEMENT PARAMETERS ----
  pnh.param("confirm_hits",      confirm_hits_,      2);     // Reliability filter
  pnh.param("min_new_lm_dist",   min_new_lm_dist_,   0.35); // Duplicate prevention [m]
  pnh.param("merge_R_scale",     merge_R_scale_,     4.0);  // Conservative merging

  // ---- INITIALIZATION BEHAVIOR ----
  pnh.param("seed_from_params", seed_from_params_, true);
  pnh.param("overwrite_with_odom_on_first_msg", overwrite_with_odom_on_first_msg_, true);
  pnh.param("init/x",   init_x_,   0.0);
  pnh.param("init/y",   init_y_,   0.0);
  pnh.param("init/yaw", init_yaw_, 0.0);
  pnh.param("initial_spread/x",   spread_x_,   0.02);
  pnh.param("initial_spread/y",   spread_y_,   0.02);
  pnh.param("initial_spread/yaw", spread_yaw_, 0.01);

  // ---- ODOMETRY INTERPRETATION ----
  pnh.param("odom_twist_in_world", odom_twist_in_world_, false);

  // ---- MOTION MODEL SETUP ----
  MotionNoise q;
  pnh.param("motion_noise/sigma_x",   q.sigma_x,   0.02);   // Process noise in X [m/s^0.5]
  pnh.param("motion_noise/sigma_y",   q.sigma_y,   0.02);   // Process noise in Y [m/s^0.5]
  pnh.param("motion_noise/sigma_yaw", q.sigma_yaw, 0.005);  // Process noise in yaw [rad/s^0.5]
  motion_ = MotionModel(q);

  // ---- DATA ASSOCIATION SETUP ----
  assoc_ = DataAssociation(chi2_gate_);

  // ---- PARTICLE INITIALIZATION ----
  // Create N particles with equal weights, empty maps
  P_.resize(N_);
  const double w0 = 1.0 / std::max(1, N_);
  for (int i = 0; i < N_; ++i) {
    P_[i].x = 0.0; P_[i].y = 0.0; P_[i].yaw = 0.0;  // Start at origin
    P_[i].weight = w0;                               // Uniform weight distribution
    P_[i].id = i;                                    // Unique particle ID
    P_[i].map.clear();                              // Start with empty landmark maps
  }

  // Initialize particles from config parameters if requested
  if (seed_from_params_) {
    ROS_INFO("[FastSLAM2] Seeding particles from params: init(%.3f, %.3f, %.3f°), spread(%.3f, %.3f, %.3f°)",
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
  ROS_INFO("  Motion noise: σ_x=%.3f σ_y=%.3f σ_yaw=%.4f", q.sigma_x, q.sigma_y, q.sigma_yaw);
  ROS_INFO("  Landmarks: confirm_hits=%d min_dist=%.2fm merge_scale=%.1f", 
           confirm_hits_, min_new_lm_dist_, merge_R_scale_);
  ROS_INFO("  Odometry: topic=%s world_frame=%s", 
           odom_topic_.c_str(), odom_twist_in_world_ ? "true" : "false");
}

// =============================================================================
// PARTICLE INITIALIZATION
// =============================================================================

void FastSLAM2::initializeParticlesFrom(double x, double y, double yaw,
                                        double sx, double sy, double syaw)
{
  ROS_INFO("[FastSLAM2] Initializing %d particles around pose (%.3f, %.3f, %.1f°)", 
           N_, x, y, yaw*180/M_PI);
  
  // Random number generation for particle spread
  std::mt19937 rng{std::random_device{}()};
  std::normal_distribution<double> nx(0.0, sx);   // X displacement noise
  std::normal_distribution<double> ny(0.0, sy);   // Y displacement noise  
  std::normal_distribution<double> nyw(0.0, syaw); // Yaw displacement noise

  // Initialize each particle with Gaussian noise around center pose
  for (auto& part : P_) {
    part.x = x + nx(rng);     // Add random X offset
    part.y = y + ny(rng);     // Add random Y offset
    part.yaw = yaw + nyw(rng); // Add random yaw offset
    // part.map remains empty (no landmarks yet)
    // part.weight remains uniform
  }
  
  particles_initialized_ = true;
  last_prop_stamp_ = ros::Time(0);  // Reset propagation timestamp

  // Cache initial pose estimates
  best_idx_ = 0;
  mean_x_ = x; mean_y_ = y; mean_yaw_ = yaw;

  ROS_INFO("[FastSLAM2] Particles initialized successfully");
}

// =============================================================================
// MAIN PROCESSING LOOP (called by main())
// =============================================================================

void FastSLAM2::spinOnce() {
  // Currently empty - all processing is event-driven via callbacks
  // Could add periodic tasks here (e.g., cleanup, diagnostics)
}

// =============================================================================
// ODOMETRY CALLBACK - MOTION DATA COLLECTION
// =============================================================================

void FastSLAM2::cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
  // ---- OPTIONAL INITIALIZATION FROM FIRST ODOMETRY MESSAGE ----
  if (!snapped_to_first_odom_ && overwrite_with_odom_on_first_msg_) {
    // Extract pose from odometry message
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; 
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; 
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Initialize all particles around this pose
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
    snapped_to_first_odom_ = true;
    
    ROS_INFO("[FastSLAM2] Snapped particles to first /odom pose: (%.3f, %.3f, %.1f°)", 
             p.x, p.y, yaw*180/M_PI);
  }

  // ---- FALLBACK INITIALIZATION (if not using params) ----
  if (!particles_initialized_ && !seed_from_params_) {
    const auto& p = msg->pose.pose.position;
    tf2::Quaternion q; 
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw; 
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    initializeParticlesFrom(p.x, p.y, yaw, spread_x_, spread_y_, spread_yaw_);
  }

  // ---- STORE VELOCITY DATA FOR MOTION INTEGRATION ----
  // We maintain a ring buffer of recent velocity commands for smooth
  // interpolation between cone observation times
  OdomStamped o;
  o.t  = msg->header.stamp;                    // Timestamp
  o.vx = msg->twist.twist.linear.x;           // Forward velocity [m/s]
  o.vy = msg->twist.twist.linear.y;           // Lateral velocity [m/s]
  o.r  = msg->twist.twist.angular.z;          // Yaw rate [rad/s]
  odom_buf_.push_back(o);

  // ---- MAINTAIN RING BUFFER SIZE ----
  // Keep only last ~2 seconds of data (sufficient for interpolation)
  const double window = 2.0;
  while (!odom_buf_.empty() && (o.t - odom_buf_.front().t).toSec() > window) {
    odom_buf_.pop_front();
  }
  
  // Debug output (throttled)
  ROS_DEBUG_THROTTLE(1.0, "[FastSLAM2] Odom buffer size: %zu, latest vel: (%.2f, %.2f, %.2f)", 
                     odom_buf_.size(), o.vx, o.vy, o.r);
}

// =============================================================================
// VELOCITY INTERPOLATION FOR SMOOTH MOTION INTEGRATION
// =============================================================================

FastSLAM2::OdomStamped
FastSLAM2::interpTwist(const OdomStamped& a, const OdomStamped& b, const ros::Time& s) const {
  const double seg = (b.t - a.t).toSec();
  if (seg <= 0.0) return a;  // Degenerate case
  
  const double frac = (s - a.t).toSec() / seg;  // Interpolation factor [0,1]
  
  OdomStamped result;
  result.t  = s;
  result.vx = (1.0 - frac)*a.vx + frac*b.vx;   // Linear interpolation
  result.vy = (1.0 - frac)*a.vy + frac*b.vy;
  result.r  = (1.0 - frac)*a.r  + frac*b.r;
  return result;
}

// =============================================================================
// CONE OBSERVATIONS CALLBACK - MAIN SLAM UPDATE
// =============================================================================

void FastSLAM2::cbCones(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!particles_initialized_) {
    ROS_WARN_THROTTLE(2.0, "[FastSLAM2] Cone observations received before particles initialized; ignoring");
    return;
  }

  ROS_DEBUG("[FastSLAM2] Processing %zu cone observations at t=%.3f", 
            msg->cones.size(), msg->header.stamp.toSec());

  // ---- STEP 1: PROPAGATE PARTICLES TO OBSERVATION TIME ----
  // Move all particles forward using stored odometry data
  propagateParticlesTo(msg->header.stamp);

  // ---- STEP 2: TRANSFORM OBSERVATIONS FROM LIDAR TO ODOM FRAME ----
  // Particles work in ODOM frame, so we need all observations in same frame
  std::vector<Eigen::Vector2d> zs;  // Cone positions in ODOM [x,y]
  std::vector<Eigen::Matrix2d> Rs;  // Cone uncertainties in ODOM [2x2 covariance]
  zs.reserve(msg->cones.size());
  Rs.reserve(msg->cones.size());

  for (const auto& c : msg->cones) {
    // Transform cone from LiDAR polar coordinates to ODOM Cartesian
    geometry_msgs::Point pt_odom;
    if (!transformConeToOdom(tfbuf_, msg->header, c.range, c.bearing, odom_frame_, pt_odom)) {
      ROS_DEBUG("Failed to transform cone (r=%.2f, θ=%.2f°) to ODOM frame", 
                c.range, c.bearing*180/M_PI);
      continue;  // Skip this observation
    }

    Eigen::Vector2d z(pt_odom.x, pt_odom.y);  // Cone position in ODOM

    // ---- TRANSFORM UNCERTAINTY FROM POLAR TO CARTESIAN ----
    // Jacobian matrix for polar-to-Cartesian transformation:
    // [x]   [r*cos(θ)]     ∂[x,y]/∂[r,θ] = [cos(θ)  -r*sin(θ)]
    // [y] = [r*sin(θ)]  →                   [sin(θ)   r*cos(θ)]
    Eigen::Matrix2d J;
    J << std::cos(c.bearing), -c.range*std::sin(c.bearing),
         std::sin(c.bearing),  c.range*std::cos(c.bearing);

    // Polar covariance matrix
    Eigen::Matrix2d Rp = Eigen::Matrix2d::Zero();
    Rp(0,0) = std::max<double>(1e-8,  c.r_var);      // Range variance
    Rp(1,1) = std::max<double>(1e-12, c.bearing_var); // Bearing variance

    // Transform: R_cartesian = J * R_polar * J^T
    Eigen::Matrix2d R = J * Rp * J.transpose();

    zs.push_back(z);
    Rs.push_back(R);
  }

  if (zs.empty()) {
    ROS_DEBUG("[FastSLAM2] No valid cone observations after transformation");
    return;
  }

  ROS_DEBUG("[FastSLAM2] Successfully transformed %zu cone observations", zs.size());

  // ---- STEP 3: CORE SLAM UPDATE ----
  // Update particle filter with measurements (association + weight update + resampling)
  processMeasurementAt(msg->header.stamp, zs, Rs);

  // ---- STEP 4: PUBLISH RESULTS ----
  publishViz(msg->header.stamp);
}

// =============================================================================
// PARTICLE MOTION PROPAGATION
// =============================================================================

void FastSLAM2::propagateParticlesTo(const ros::Time& t) {
  // Guard conditions
  if (!particles_initialized_ || odom_buf_.size() < 2) {
    ROS_DEBUG("[FastSLAM2] Cannot propagate: particles_init=%s odom_buf_size=%zu", 
              particles_initialized_ ? "true" : "false", odom_buf_.size());
    return;
  }

  // Initialize propagation timestamp
  if (last_prop_stamp_.isZero()) last_prop_stamp_ = odom_buf_.front().t;
  if (t <= last_prop_stamp_) return;  // Already propagated to this time

  ROS_DEBUG("[FastSLAM2] Propagating particles from t=%.3f to t=%.3f", 
            last_prop_stamp_.toSec(), t.toSec());

  // ---- INTEGRATE MOTION OVER TIME SEGMENTS ----
  // We integrate piecewise between odometry samples for smooth motion
  for (size_t i = 0; i + 1 < odom_buf_.size(); ++i) {
    const auto& a = odom_buf_[i];      // Earlier odometry sample
    const auto& b = odom_buf_[i+1];    // Later odometry sample

    // Find intersection of [a.t, b.t] with [last_prop_stamp_, t]
    const ros::Time seg_start = std::max(a.t, last_prop_stamp_);
    const ros::Time seg_end   = std::min(b.t, t);
    if (seg_end <= seg_start) continue;  // No overlap

    // Interpolate velocities at segment endpoints for smooth integration
    const auto ta = interpTwist(a, b, seg_start);
    const auto tb = interpTwist(a, b, seg_end);

    // Trapezoidal rule integration parameters
    const double dt = (seg_end - seg_start).toSec();
    const double vx_avg = 0.5*(ta.vx + tb.vx);  // Average forward velocity
    const double vy_avg = 0.5*(ta.vy + tb.vy);  // Average lateral velocity
    const double r_avg  = 0.5*(ta.r  + tb.r);   // Average yaw rate

    ROS_DEBUG("[FastSLAM2] Integrating segment dt=%.4f: v_avg=(%.3f,%.3f,%.3f)", 
              dt, vx_avg, vy_avg, r_avg);

    // ---- PROPAGATE EACH PARTICLE ----
    for (auto& p : P_) {
      double vx_b = vx_avg, vy_b = vy_avg;  // Body-frame velocities
      
      // Handle coordinate frame conversion if needed
      if (odom_twist_in_world_) {
        // Convert world-frame velocities to body-frame for this particle
        worldToBody(p.yaw, vx_avg, vy_avg, vx_b, vy_b);
      }
      // Note: if odom_twist_in_world_ == false, velocities are already in body frame
      
      // Apply motion model with uncertainty
      motion_.propagate(p, vx_b, vy_b, r_avg, dt);
      
      // The motion model:
      // 1. Transforms body velocities to world frame using particle's current yaw
      // 2. Integrates: p.x += vx_world*dt, p.y += vy_world*dt, p.yaw += r*dt  
      // 3. Adds Gaussian noise proportional to sqrt(dt) for proper diffusion
    }
  }

  last_prop_stamp_ = t;  // Update propagation timestamp
  ROS_DEBUG("[FastSLAM2] Particle propagation complete");
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
  
  // Simple Euclidean distance search (not Mahalanobis - used for duplicate detection)
  for (int i = 0; i < (int)p.map.size(); ++i) {
    const auto& lm = p.map[i];
    const double d2 = (z - lm.mu).squaredNorm();  // ||z - landmark_pos||²
    if (d2 < best_d2) { 
      best_d2 = d2; 
      best = i; 
    }
  }
  
  out_dist_sq = best_d2;
  return best;
}

// =============================================================================
// CORE MEASUREMENT UPDATE - THE HEART OF FASTSLAM 2.0
// =============================================================================

void FastSLAM2::processMeasurementAt(const ros::Time& /*t*/,
                                     const std::vector<Eigen::Vector2d>& zs,
                                     const std::vector<Eigen::Matrix2d>& Rs)
{
  ROS_DEBUG("[FastSLAM2] Processing measurement update with %zu observations", zs.size());
  
  // ---- FOR EACH PARTICLE: UPDATE PERSONAL MAP AND COMPUTE LIKELIHOOD ----
  for (auto& p : P_) {
    double log_w_inc = 0.0;  // Log-likelihood increment for this particle
    
    // Process each cone observation for this particle
    for (size_t i = 0; i < zs.size(); ++i) {
      const Eigen::Vector2d& z = zs[i];  // Observation: where we see cone [x,y] in ODOM
      const Eigen::Matrix2d& R = Rs[i];  // Observation uncertainty (2x2 covariance)

      ROS_DEBUG("[FastSLAM2] Particle %d processing obs %zu: z=(%.2f,%.2f)", 
                p.id, i, z.x(), z.y());

      // ---- STEP 1: DATA ASSOCIATION ----
      // Try to match this observation to an existing landmark in particle's map
      AssocResult a = assoc_.associate(p, z, R);

      if (a.lm_index >= 0) {
        // ---- CASE 1: OBSERVATION MATCHES EXISTING LANDMARK ----
        auto& lm = p.map[a.lm_index];  // Reference to matched landmark
        
        ROS_DEBUG("[FastSLAM2] Particle %d: obs %zu → landmark %d (d²=%.3f)", 
                  p.id, i, a.lm_index, a.d2);
        
        // Extended Kalman Filter update of landmark position
        const Eigen::Vector2d nu = z - lm.mu;           // Innovation (observed - expected)
        const Eigen::Matrix2d S  = lm.Sigma + R;        // Innovation covariance
        const Eigen::Matrix2d K  = lm.Sigma * S.inverse(); // Kalman gain
        
        // Update landmark state
        lm.mu     = lm.mu + K * nu;                      // Refine position estimate
        lm.Sigma  = (Eigen::Matrix2d::Identity() - K) * lm.Sigma; // Reduce uncertainty

        // Update landmark metadata
        lm.hits++;
        if (!lm.confirmed && lm.hits >= confirm_hits_) {
          lm.confirmed = true;  // Landmark becomes reliable
          ROS_DEBUG("[FastSLAM2] Particle %d: landmark %d CONFIRMED (hits=%d)", 
                    p.id, a.lm_index, lm.hits);
        }

        // ---- COMPUTE OBSERVATION LIKELIHOOD ----
        // How well did this particle predict the observation?
        const double detS = std::max(1e-18, S.determinant());
        const double expo = -0.5 * nu.transpose() * S.inverse() * nu;
        const double log_likelihood = expo - 0.5*std::log(detS) - std::log(2*M_PI);
        log_w_inc += log_likelihood;
        
        ROS_DEBUG("[FastSLAM2] Particle %d: landmark update, log_like=%.3f", p.id, log_likelihood);

      } else {
        // ---- CASE 2: NO ASSOCIATION - HANDLE NEW OBSERVATION ----
        
        ROS_DEBUG("[FastSLAM2] Particle %d: obs %zu has no association", p.id, i);
        
        // Check if observation is too close to existing landmarks (duplicate detection)
        double d2;
        const int j = nearestLandmarkIdx(p, z, d2);
        const double min_d2 = min_new_lm_dist_ * min_new_lm_dist_;

        if (j >= 0 && d2 < min_d2) {
          // ---- SUBCASE 2A: TOO CLOSE TO EXISTING LANDMARK - MERGE ----
          auto& lm = p.map[j];
          
          ROS_DEBUG("[FastSLAM2] Particle %d: obs %zu merging with landmark %d (d²=%.3f)", 
                    p.id, i, j, d2);
          
          // Conservative merge using inflated measurement noise
          const Eigen::Matrix2d Rm = R * merge_R_scale_;  // Inflate uncertainty
          
          // EKF update with inflated noise (more conservative)
          const Eigen::Vector2d nu = z - lm.mu;
          const Eigen::Matrix2d S  = lm.Sigma + Rm;
          const Eigen::Matrix2d K  = lm.Sigma * S.inverse();
          lm.mu     = lm.mu + K * nu;
          lm.Sigma  = (Eigen::Matrix2d::Identity() - K) * lm.Sigma;

          lm.hits++;
          if (!lm.confirmed && lm.hits >= confirm_hits_) lm.confirmed = true;

          // Compute likelihood (with inflated R)
          const double detS = std::max(1e-18, S.determinant());
          const double expo = -0.5 * nu.transpose() * S.inverse() * nu;
          log_w_inc += expo - 0.5*std::log(detS) - std::log(2*M_PI);

        } else {
          // ---- SUBCASE 2B: CREATE TRULY NEW LANDMARK ----
          
          ROS_DEBUG("[FastSLAM2] Particle %d: creating new landmark from obs %zu", p.id, i);
          
          Landmark lm;
          lm.mu = z;                                      // Initialize at observation location
          lm.Sigma = R + Eigen::Matrix2d::Identity() * 0.25; // Initial uncertainty (obs + prior)
          lm.hits = 1;                                    // First observation
          lm.confirmed = (confirm_hits_ <= 1);            // Confirm immediately if threshold ≤ 1
          p.map.push_back(lm);                           // Add to particle's map
          
          ROS_DEBUG("[FastSLAM2] Particle %d: new landmark created, map_size=%zu", 
                    p.id, p.map.size());
        }
      }
    }

    // ---- UPDATE PARTICLE WEIGHT ----
    // Convert from log domain: new_weight = old_weight * likelihood
    p.weight = std::log(std::max(1e-300, p.weight)) + log_w_inc;
    
    ROS_DEBUG("[FastSLAM2] Particle %d: log_weight_increment=%.3f, map_size=%zu", 
              p.id, log_w_inc, p.map.size());
  }

  // ---- NORMALIZE WEIGHTS ----
  // Convert back from log domain and ensure weights sum to 1
  double max_logw = -1e300;
  for (const auto& p : P_) max_logw = std::max(max_logw, p.weight);
  
  double sum_w = 0.0;
  for (auto& p : P_) { 
    p.weight = std::exp(p.weight - max_logw);  // Numerical stability
    sum_w += p.weight; 
  }
  
  if (sum_w <= 0.0) {
    // Degenerate case - reset to uniform weights
    const double w0 = 1.0 / std::max<int>(1, P_.size());
    for (auto& p : P_) p.weight = w0;
    sum_w = 1.0;
    ROS_WARN("[FastSLAM2] Weight normalization failed - reset to uniform");
  } else {
    // Normalize to sum to 1
    for (auto& p : P_) p.weight /= sum_w;
  }

  // ---- ADAPTIVE RESAMPLING ----
  // Resample when effective particle count drops too low (diversity loss)
  double inv_neff = 0.0;
  for (const auto& p : P_) inv_neff += p.weight * p.weight;
  const double neff = (inv_neff > 0.0) ? (1.0 / inv_neff) : 0.0;
  
  if (neff < neff_ratio_ * P_.size()) {
    systematicResample(P_);  // Duplicate good particles, remove bad ones
    ROS_INFO("[FastSLAM2] Resampled particles: Neff=%.1f < %.1f (threshold)", 
             neff, neff_ratio_ * P_.size());
  }

  // ---- CACHE RESULTS FOR OUTPUT ----
  // Find best particle (highest weight)
  best_idx_ = 0; 
  double bestw = -1.0;
  for (int i = 0; i < (int)P_.size(); ++i) {
    if (P_[i].weight > bestw) { 
      bestw = P_[i].weight; 
      best_idx_ = i; 
    }
  }

  // Compute weighted mean pose
  double cx=0, cy=0, cyaw_c=0, cyaw_s=0;
  for (const auto& p : P_) {
    cx += p.weight * p.x;
    cy += p.weight * p.y;
    cyaw_c += p.weight * std::cos(p.yaw);  // Circular statistics for angles
    cyaw_s += p.weight * std::sin(p.yaw);
  }
  mean_x_ = cx; 
  mean_y_ = cy; 
  mean_yaw_ = std::atan2(cyaw_s, cyaw_c);
  
  ROS_DEBUG("[FastSLAM2] Measurement update complete: best_particle=%d neff=%.1f mean_pose=(%.2f,%.2f,%.1f°)", 
            best_idx_, neff, mean_x_, mean_y_, mean_yaw_*180/M_PI);
}

// =============================================================================
// VISUALIZATION AND OUTPUT PUBLISHING  
// =============================================================================

void FastSLAM2::publishViz(const ros::Time& t) {
  // ---- PUBLISH PARTICLE ARROWS FOR RVIZ ----
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

    // Particle pose
    m.pose.position.x = p.x;
    m.pose.position.y = p.y;
    m.pose.position.z = 0.05;  // Slight elevation

    // Particle orientation
    tf2::Quaternion q; q.setRPY(0, 0, p.yaw);
    m.pose.orientation = tf2::toMsg(q);

    // Arrow appearance (scale with weight for visual feedback)
    m.scale.x = 0.3;  // Length
    m.scale.y = 0.05; // Width
    m.scale.z = 0.05; // Height
    m.color.a = 0.3 + 0.7 * (p.weight * P_.size()); // Alpha scales with weight
    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; // Red particles
    
    particle_markers.markers.push_back(m);
  }
  pub_particles_.publish(particle_markers);

  // ---- PUBLISH PARTICLE POSE ARRAY ----
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

  // ---- PUBLISH LANDMARK MARKERS (BEST PARTICLE'S MAP) ----
  visualization_msgs::MarkerArray landmark_markers;
  const auto& bestP = P_[std::max(0, std::min<int>(best_idx_, (int)P_.size()-1))];
  int lm_id = 0;
  size_t confirmed_count = 0;
  
  for (const auto& lm : bestP.map) {
    if (!lm.confirmed) continue;  // Only show confirmed landmarks
    confirmed_count++;
    
    visualization_msgs::Marker m;
    m.header.stamp = t;
    m.header.frame_id = map_frame_;
    m.ns = "landmarks";
    m.id = lm_id++;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    
    // Landmark position
    m.pose.position.x = lm.mu.x();
    m.pose.position.y = lm.mu.y();
    m.pose.position.z = 0.05;
    
    // Landmark appearance
    m.scale.x = m.scale.y = m.scale.z = 0.18;  // Sphere size
    m.color.a = 1.0;
    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;  // Blue landmarks
    
    landmark_markers.markers.push_back(m);
  }
  pub_map_markers_.publish(landmark_markers);
  
  // ---- PUBLISH SLAM ODOMETRY ESTIMATES ----
  
  // Best particle pose
  nav_msgs::Odometry odom_best;
  odom_best.header.stamp = t;
  odom_best.header.frame_id = map_frame_;
  odom_best.child_frame_id  = base_frame_;
  odom_best.pose.pose.position.x = bestP.x;
  odom_best.pose.pose.position.y = bestP.y;
  odom_best.pose.pose.position.z = 0.0;
  tf2::Quaternion q_best; q_best.setRPY(0,0,bestP.yaw);
  odom_best.pose.pose.orientation = tf2::toMsg(q_best);
  pub_slam_odom_.publish(odom_best);

  // Weighted mean pose  
  nav_msgs::Odometry odom_mean = odom_best;
  odom_mean.pose.pose.position.x = mean_x_;
  odom_mean.pose.pose.position.y = mean_y_;
  tf2::Quaternion q_mean; q_mean.setRPY(0,0,mean_yaw_);
  odom_mean.pose.pose.orientation = tf2::toMsg(q_mean);
  pub_slam_odom_mean_.publish(odom_mean);
  
  // Status logging
  ROS_INFO_THROTTLE(1.0, "[FastSLAM2] Published viz: particles=%zu landmarks=%zu (best_particle=%d)", 
                    P_.size(), confirmed_count, best_idx_);
}