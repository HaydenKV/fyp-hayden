#include "qcar_visnav/slam/fastslam2_core.h"
#include "qcar_visnav/slam/resampling.h"
#include "qcar_visnav/slam/motion_model.h"
#include "qcar_visnav/slam/landmark_ekf.h"
#include "qcar_visnav/slam/data_assoc.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <cmath>
#include <algorithm>

using namespace qcar_visnav::slam;

// Constants
static constexpr double R_RANGE_VAR_FLOOR  = 0.10 * 0.10;   
static constexpr double R_BEAR_VAR_FLOOR   = (3.0*M_PI/180.0)*(3.0*M_PI/180.0); 
static constexpr double R_CART_INFLATE_XY  = 0.20 * 0.20;   
static constexpr double NEW_LM_INIT_STD    = 0.60;          

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
  : nh_(nh), pnh_(pnh), tf_buffer_(ros::Duration(10.0)), tf_listener_(tf_buffer_)
{
  // topics
  pnh_.param("tracked_topic", P_.topic_tracked, P_.topic_tracked);
  pnh_.param("odom_topic",    P_.topic_odom,    P_.topic_odom);
  pnh_.param("pub_particles", P_.pub_particles, P_.pub_particles);
  pnh_.param("pub_landmarks", P_.pub_landmarks, P_.pub_landmarks);
  pnh_.param("pub_weights",   P_.pub_weights,   P_.pub_weights);
  pnh_.param("pub_slam_odom", P_.pub_slam_odom, P_.pub_slam_odom);

  // frames
  pnh_.param("map_frame",  P_.map_frame,  P_.map_frame);
  pnh_.param("odom_frame", P_.odom_frame, P_.odom_frame);
  pnh_.param("base_frame", P_.base_frame, P_.base_frame);
  pnh_.param("lidar_frame",P_.lidar_frame,P_.lidar_frame);

  // DEBUG MODE PARAMETER - NEW!
  pnh_.param("debug_use_ground_truth", debug_use_ground_truth_, false);
  
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
  pnh_.param("odom_v_std", nv, nv);
  pnh_.param("odom_yawrate_std", nw, nw);
  P_.odom_v_std = nv; P_.odom_yawrate_std = nw;

  pnh_.param("miss_in_fov_penalty", P_.miss_in_fov_penalty, P_.miss_in_fov_penalty);
  pnh_.param("color_mismatch_penalty", P_.color_mismatch_penalty, P_.color_mismatch_penalty);

  // LiDAR->base extrinsics
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
  pub_slam_odom_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(P_.pub_slam_odom, 1, false);

  // ADDED: Ground truth subscriber for debug mode
  if (debug_use_ground_truth_) {
    sub_ground_truth_ = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/state", 10, &FastSLAM2::groundTruthCb, this);
    ROS_WARN("[fastslam2_core] DEBUG MODE: Using ground truth odometry instead of EKF!");
  }

  ROS_INFO_STREAM("[fastslam2_core] topics: tracked="<<P_.topic_tracked
    << " odom="<<P_.topic_odom<< " pubs=["<<P_.pub_particles<<","<<P_.pub_landmarks<<","<<P_.pub_weights<<","<<P_.pub_slam_odom<<"]"
    << " frames odom="<<P_.odom_frame<<" base="<<P_.base_frame<<" lidar="<<P_.lidar_frame
    << " particles="<<P_.particles << " debug_mode=" << debug_use_ground_truth_);

  ensureInitParticles();
}

void FastSLAM2::ensureInitParticles() {
  if (!particles_.empty()) return;
  particles_.resize(P_.particles);
  const double w = 1.0 / (double)P_.particles;
  
  // Initialize particles with small spread around origin
  std::normal_distribution<double> noise_xy(0.0, 0.05);  // 5cm initial spread
  std::normal_distribution<double> noise_yaw(0.0, 0.02); // ~1 degree initial spread
  
  for (auto& p : particles_) { 
    p.pose.x() = noise_xy(gen_);
    p.pose.y() = noise_xy(gen_);
    p.pose.z() = noise_yaw(gen_);
    p.weight = w; 
    p.map.clear(); 
  }
  
  // Set map initialization flag
  if (!map_initialized_) {
    initial_pose_ = Eigen::Vector3d::Zero();  // Map origin at first particle location
    map_initialized_ = true;
  }
  
  ROS_INFO("[fastslam2_core] Initialized %d particles with small spread", P_.particles);
}

void FastSLAM2::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  if (debug_use_ground_truth_) return;  // Skip EKF odom in debug mode
  
  // Store latest for propagation
  last_v_ = msg->twist.twist.linear.x;
  last_yawrate_ = msg->twist.twist.angular.z;
  last_odom_stamp_ = msg->header.stamp;
  have_odom_ = true;
}

// ADDED: Ground truth callback for debug mode
void FastSLAM2::groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!debug_use_ground_truth_) return;
  
  // Extract velocity from ground truth (more reliable than EKF for debugging)
  last_v_ = msg->twist.twist.linear.x;
  last_yawrate_ = msg->twist.twist.angular.z;
  last_odom_stamp_ = msg->header.stamp;
  have_odom_ = true;
  
  // Store ground truth pose for validation
  ground_truth_pose_.x() = msg->pose.pose.position.x;
  ground_truth_pose_.y() = msg->pose.pose.position.y;
  
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  ground_truth_pose_.z() = yaw;
}

Eigen::Matrix2d FastSLAM2::polarCovToCart(double r, double th, double r_var, double th_var) const {
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

  // Inflate in Cartesian
  Rxy(0,0) += R_CART_INFLATE_XY;
  Rxy(1,1) += R_CART_INFLATE_XY;
  return Rxy;
}

// COMPLETELY REWRITTEN: Proper landmark initialization in MAP frame
Eigen::Vector2d FastSLAM2::getLandmarkPositionInMap(const Eigen::Vector3d& particle_pose_in_map,
                                                    double lidar_range, double lidar_bearing) const
{
  // STEP 1: LiDAR polar → LiDAR Cartesian
  Eigen::Vector2d cone_in_lidar;
  cone_in_lidar.x() = lidar_range * std::cos(lidar_bearing);
  cone_in_lidar.y() = lidar_range * std::sin(lidar_bearing);
  
  // STEP 2: LiDAR → Base (apply sensor extrinsics)
  Eigen::Matrix2d R_base_lidar = Rot2(P_.yaw_bl);
  Eigen::Vector2d cone_in_base = R_base_lidar * cone_in_lidar + P_.t_bl;
  
  // STEP 3: Base → Map (using particle's pose in map frame)
  // This is the KEY TRANSFORMATION that was missing!
  Eigen::Matrix2d R_map_base = Rot2(particle_pose_in_map.z());
  Eigen::Vector2d cone_in_map = R_map_base * cone_in_base + particle_pose_in_map.head<2>();
  
  return cone_in_map;  // Landmark in global MAP coordinates
}

void FastSLAM2::integrateOdom(double stamp_sec) {
  if (!have_odom_) return;
  const double dt = std::max(0.0, stamp_sec - last_odom_stamp_.toSec());
  if (dt <= 0.0 || dt > 0.5) return;  // Skip huge dt jumps

  // IMPROVED: Scale noise with time and add minimum noise floor
  const double dt_clamped = std::min(dt, 0.2);
  const double noise_scale = std::sqrt(dt_clamped);
  
  std::normal_distribution<double> Nv(0.0, P_.odom_v_std * noise_scale);
  std::normal_distribution<double> Nw(0.0, P_.odom_yawrate_std * noise_scale);

  // DEBUG MODE: Reduce noise significantly for ground truth validation
  double noise_reduction = debug_use_ground_truth_ ? 0.1 : 1.0;

  for (auto& p : particles_) {
    double v_noisy = last_v_ + Nv(gen_) * noise_reduction;
    double w_noisy = last_yawrate_ + Nw(gen_) * noise_reduction;
    propagate_pose(p.pose, v_noisy, w_noisy, dt_clamped);
  }
  
  last_odom_stamp_ = ros::Time(stamp_sec);
}

void FastSLAM2::conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (particles_.empty()) ensureInitParticles();

  // 1) Propagate particles to measurement time
  integrateOdom(msg->header.stamp.toSec());

  // 2) Parse cone observations
  struct Detection { 
    double range, bearing, r_var, bearing_var; 
    int color; 
    double color_conf; 
  };
  
  std::vector<Detection> detections;
  detections.reserve(msg->cones.size());
  
  for (const auto& cone : msg->cones) {
    if (!std::isfinite(cone.range) || !std::isfinite(cone.bearing)) continue;
    if (cone.range < 0.1 || cone.range > 20.0) continue;  // Sanity check
    
    detections.push_back({
      cone.range, cone.bearing,
      std::max(1e-8, cone.r_var), std::max(1e-10, cone.bearing_var),
      (int)cone.color, cone.color_conf
    });
  }
  
  if (detections.empty()) {
    ROS_DEBUG("[fastslam2_core] No valid cone detections");
    return;
  }

  // 3) FastSLAM 2.0 Update - COMPLETELY REWRITTEN for proper coordinate handling
  std::vector<double> log_weights(particles_.size(), 0.0);
  double max_log_weight = -1e100;

  for (size_t particle_idx = 0; particle_idx < particles_.size(); ++particle_idx) {
    auto& particle = particles_[particle_idx];
    
    // Collect existing landmark positions and covariances for association
    std::vector<Eigen::Vector2d> landmark_positions;
    std::vector<Eigen::Matrix2d> landmark_covariances;
    landmark_positions.reserve(particle.map.size());
    landmark_covariances.reserve(particle.map.size());
    
    for (const auto& landmark : particle.map) {
      landmark_positions.push_back(landmark.mu);
      landmark_covariances.push_back(landmark.Sigma);
    }

    int successful_associations = 0;

    // Process each cone detection
    for (const auto& detection : detections) {
      // CRITICAL FIX: Transform detection to MAP frame using particle's pose
      Eigen::Vector2d landmark_pos_in_map = getLandmarkPositionInMap(
        particle.pose, detection.range, detection.bearing
      );
      
      // Convert polar covariance to Cartesian in MAP frame
      Eigen::Matrix2d measurement_cov = polarCovToCart(
        detection.range, detection.bearing, 
        detection.r_var, detection.bearing_var
      );

      // Data association in MAP frame (C++11 compatible)
      auto association_result = nn_gated(
        landmark_positions, landmark_covariances, 
        landmark_pos_in_map, measurement_cov, P_.chi2_gate
      );
      int associated_idx = association_result.first;
      double mahalanobis_dist = association_result.second;

      if (associated_idx >= 0) {
        // UPDATE EXISTING LANDMARK
        auto& landmark = particle.map[associated_idx];
        
        // EKF update in MAP frame (H = Identity)
        ekf_update_landmark(landmark.mu, landmark.Sigma, landmark_pos_in_map, measurement_cov);
        
        landmark.hits = std::min(landmark.hits + 1, 1000000);
        landmark.misses = 0;

        // Weight update using innovation likelihood
        Eigen::Vector2d innovation = landmark_pos_in_map - landmark.mu;
        Eigen::Matrix2d innovation_cov = landmark.Sigma + measurement_cov;
        
        // Gaussian likelihood (log domain)
        double det = innovation_cov.determinant();
        if (det > 1e-12) {
          double log_likelihood = -0.5 * (
            innovation.transpose() * innovation_cov.inverse() * innovation + 
            std::log(2.0 * M_PI * det)
          );
          log_weights[particle_idx] += log_likelihood;
        }

        // Color consistency bonus/penalty
        if (detection.color > 0 && landmark.color > 0) {
          if (detection.color == landmark.color) {
            log_weights[particle_idx] += std::log(1.2);  // Small bonus
          } else {
            log_weights[particle_idx] += std::log(P_.color_mismatch_penalty);
          }
        } else if (landmark.color == 0 && detection.color > 0 && detection.color_conf > 0.6) {
          landmark.color = detection.color;
          landmark.color_conf = detection.color_conf;
        }

        ++successful_associations;
        
      } else {
        // CREATE NEW LANDMARK - FIXED coordinate frame
        Landmark new_landmark;
        new_landmark.mu = landmark_pos_in_map;  // Position in MAP frame
        new_landmark.Sigma = Eigen::Matrix2d::Identity() * (NEW_LM_INIT_STD * NEW_LM_INIT_STD);
        new_landmark.hits = 1;
        new_landmark.misses = 0;
        new_landmark.color = (detection.color > 0 && detection.color_conf > 0.6) ? detection.color : 0;
        new_landmark.color_conf = new_landmark.color > 0 ? detection.color_conf : 0.0;
        
        particle.map.push_back(new_landmark);
        
        // Update association arrays for future detections in this particle
        landmark_positions.push_back(new_landmark.mu);
        landmark_covariances.push_back(new_landmark.Sigma);
        
        // Conservative likelihood for new landmarks
        log_weights[particle_idx] += std::log(std::max(1e-6, P_.new_lm_lik_min));
      }
    }

    // Penalty for poor association performance
    if (successful_associations == 0 && !detections.empty()) {
      log_weights[particle_idx] += std::log(std::max(1e-6, P_.miss_in_fov_penalty));
    }

    max_log_weight = std::max(max_log_weight, log_weights[particle_idx]);
  }

  // 4) Normalize weights (prevent numerical underflow)
  double total_weight = 0.0;
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].weight *= std::exp(log_weights[i] - max_log_weight);
    total_weight += particles_[i].weight;
  }
  
  if (total_weight <= 1e-12) {
    // Fallback: uniform weights
    const double uniform_weight = 1.0 / particles_.size();
    for (auto& p : particles_) p.weight = uniform_weight;
  } else {
    // Normalize
    for (auto& p : particles_) p.weight /= total_weight;
  }

  // 5) Resampling
  const double effective_particles = neff(particles_);
  if (effective_particles < P_.neff_ratio * particles_.size()) {
    particles_ = systematic_resample(particles_, gen_);
    ROS_DEBUG("[fastslam2_core] Resampled: Neff=%.1f < %.1f", 
              effective_particles, P_.neff_ratio * particles_.size());
  }

  // 6) Publish all outputs
  publishParticles(msg->header.stamp);
  publishLandmarks(msg->header.stamp);
  publishWeights(msg->header.stamp);
  publishSlamOdom(msg->header.stamp);
  publishMapToOdomTF(msg->header.stamp);
  
  // DEBUG: Print diagnostics
  if (debug_use_ground_truth_ && particles_.size() > 0) {
    Eigen::Vector3d estimated_pose = getWeightedMeanPose();
    ROS_INFO_THROTTLE(1.0, "[DEBUG] GT: (%.2f,%.2f,%.1f°) Est: (%.2f,%.2f,%.1f°) Landmarks: %zu", 
                      ground_truth_pose_.x(), ground_truth_pose_.y(), ground_truth_pose_.z()*180/M_PI,
                      estimated_pose.x(), estimated_pose.y(), estimated_pose.z()*180/M_PI,
                      particles_[getBestParticleIndex()].map.size());
  }
}

void FastSLAM2::spinOnce() {
  // All computation is event-driven (callbacks), so this is just a hook
  // for potential future time-driven publishing/diagnostics
}

int FastSLAM2::getBestParticleIndex() const {
  int best_idx = 0;
  double best_weight = -1.0;
  for (int i = 0; i < (int)particles_.size(); ++i) {
    if (particles_[i].weight > best_weight) {
      best_weight = particles_[i].weight;
      best_idx = i;
    }
  }
  return best_idx;
}

Eigen::Vector3d FastSLAM2::getWeightedMeanPose() const {
  Eigen::Vector3d mean_pose = Eigen::Vector3d::Zero();
  double total_weight = 0.0;
  double sum_cos = 0.0, sum_sin = 0.0;
  
  for (const auto& particle : particles_) {
    mean_pose.head<2>() += particle.weight * particle.pose.head<2>();
    sum_cos += particle.weight * std::cos(particle.pose.z());
    sum_sin += particle.weight * std::sin(particle.pose.z());
    total_weight += particle.weight;
  }
  
  if (total_weight > 1e-12) {
    mean_pose.head<2>() /= total_weight;
    mean_pose.z() = std::atan2(sum_sin / total_weight, sum_cos / total_weight);
  }
  
  return mean_pose;
}

void FastSLAM2::publishSlamOdom(const ros::Time& t) {
  if (!pub_slam_odom_) return;
  
  Eigen::Vector3d pose_estimate = getWeightedMeanPose();
  
  geometry_msgs::PoseWithCovarianceStamped slam_pose;
  slam_pose.header.stamp = t;
  slam_pose.header.frame_id = P_.map_frame;
  
  slam_pose.pose.pose.position.x = pose_estimate.x();
  slam_pose.pose.pose.position.y = pose_estimate.y();
  slam_pose.pose.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, pose_estimate.z());
  slam_pose.pose.pose.orientation = tf2::toMsg(q);
  
  // Compute covariance from particle spread
  Eigen::Matrix3d pose_cov = Eigen::Matrix3d::Zero();
  double total_weight = 0.0;
  
  for (const auto& p : particles_) {
    Eigen::Vector3d diff = p.pose - pose_estimate;
    // Handle angle wraparound
    while (diff.z() >  M_PI) diff.z() -= 2*M_PI;
    while (diff.z() < -M_PI) diff.z() += 2*M_PI;
    
    pose_cov += p.weight * (diff * diff.transpose());
    total_weight += p.weight;
  }
  
  if (total_weight > 1e-12) {
    pose_cov /= total_weight;
  } else {
    pose_cov = Eigen::Matrix3d::Identity() * 0.01;
  }
  
  // Fill 6x6 covariance matrix
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      if (i < 3 && j < 3) {
        slam_pose.pose.covariance[i*6 + j] = pose_cov(i, j);
      } else {
        slam_pose.pose.covariance[i*6 + j] = (i == j) ? 1e-6 : 0.0;
      }
    }
  }
  
  pub_slam_odom_.publish(slam_pose);
}

void FastSLAM2::publishMapToOdomTF(const ros::Time& t) {
  // Get current robot pose estimate in map frame
  Eigen::Vector3d robot_pose_map = getWeightedMeanPose();
  
  // For Block 5: publish identity transform (refined in Block 6)
  // In a proper implementation, this would be: T_map_odom = T_map_base * T_base_odom^-1
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = t;
  transform.header.frame_id = P_.map_frame;
  transform.child_frame_id = P_.odom_frame;
  
  // Identity transform for now
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = 0.0;
  transform.transform.rotation.y = 0.0;
  transform.transform.rotation.z = 0.0;
  transform.transform.rotation.w = 1.0;
  
  tf_broadcaster_.sendTransform(transform);
}

void FastSLAM2::publishParticles(const ros::Time& t) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = t;
  pose_array.header.frame_id = P_.map_frame;  // Particles in MAP frame
  pose_array.poses.reserve(particles_.size());
  
  for (const auto& particle : particles_) {
    geometry_msgs::Pose pose;
    pose.position.x = particle.pose.x();
    pose.position.y = particle.pose.y();
    pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, particle.pose.z());
    pose.orientation = tf2::toMsg(q);
    
    pose_array.poses.push_back(pose);
  }
  
  if (pub_particles_) pub_particles_.publish(pose_array);
}

void FastSLAM2::publishLandmarks(const ros::Time& t) {
  int best_particle = getBestParticleIndex();
  const auto& landmark_map = particles_[best_particle].map;

  visualization_msgs::MarkerArray marker_array;
  
  // Clear previous markers
  visualization_msgs::Marker delete_marker;
  delete_marker.header.stamp = t;
  delete_marker.header.frame_id = P_.map_frame;  // MAP frame
  delete_marker.ns = "slam_landmarks";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  // Create landmark markers
  for (size_t i = 0; i < landmark_map.size(); ++i) {
    const auto& landmark = landmark_map[i];
    
    visualization_msgs::Marker marker;
    marker.header.stamp = t;
    marker.header.frame_id = P_.map_frame;  // MAP frame
    marker.ns = "slam_landmarks";
    marker.id = i + 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = landmark.mu.x();
    marker.pose.position.y = landmark.mu.y();
    marker.pose.position.z = 0.1;  // Lift above ground
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    
    marker.color = colorFor(landmark.color > 0 ? landmark.color : (int)i);
    marker.lifetime = ros::Duration(0.0);
    
    marker_array.markers.push_back(marker);
  }

  if (pub_landmarks_) pub_landmarks_.publish(marker_array);
}

void FastSLAM2::publishWeights(const ros::Time& t) {
  if (!pub_weights_) return;
  
  std_msgs::Float32MultiArray weight_msg;
  weight_msg.layout.dim.resize(1);
  weight_msg.layout.dim[0].label = "particle_weights";
  weight_msg.layout.dim[0].size = particles_.size();
  weight_msg.layout.dim[0].stride = particles_.size();
  weight_msg.data.reserve(particles_.size());
  
  for (const auto& particle : particles_) {
    weight_msg.data.push_back(static_cast<float>(particle.weight));
  }
  
  pub_weights_.publish(weight_msg);
}