// Complete FastSLAM 2.0 Core Implementation
// Fixed all linker errors - complete method implementations

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
static constexpr double R_RANGE_VAR_FLOOR  = 0.05 * 0.05;
static constexpr double R_BEAR_VAR_FLOOR   = (2.0*M_PI/180.0)*(2.0*M_PI/180.0); 
static constexpr double R_CART_INFLATE_XY  = 0.10 * 0.10;
static constexpr double NEW_LM_INIT_STD    = 0.50;

// Constructor implementation
FastSLAM2::FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh) 
  : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_) {
  
  // Load parameters
  pnh_.param("particles", P_.particles, 80);
  pnh_.param("neff_ratio", P_.neff_ratio, 0.7);
  pnh_.param("chi2_gate", P_.chi2_gate, 7.38);
  pnh_.param("new_landmark_likelihood_min", P_.new_lm_lik_min, 0.4);
  pnh_.param("odom_v_std", P_.odom_v_std, 0.12);
  pnh_.param("odom_yawrate_std", P_.odom_yawrate_std, 0.15);
  pnh_.param("debug_use_ground_truth", debug_use_ground_truth_, false);
  
  double meas_noise_x, meas_noise_y;
  pnh_.param("meas_noise_xy_x", meas_noise_x, 0.15);
  pnh_.param("meas_noise_xy_y", meas_noise_y, 0.15);
  P_.meas_noise_xy = Eigen::Vector2d(meas_noise_x, meas_noise_y);
  
  // Frame parameters
  pnh_.param("map_frame", P_.map_frame, std::string("map"));
  pnh_.param("odom_frame", P_.odom_frame, std::string("odom"));
  pnh_.param("base_frame", P_.base_frame, std::string("base_footprint"));
  
  // Topic parameters
  pnh_.param("topic_tracked", P_.topic_tracked, std::string("/tracked_cones"));
  pnh_.param("topic_odom", P_.topic_odom, std::string("/ekf/odom"));
  pnh_.param("pub_particles", P_.pub_particles, std::string("/slam/particles"));
  pnh_.param("pub_landmarks", P_.pub_landmarks, std::string("/map_markers"));
  pnh_.param("pub_weights", P_.pub_weights, std::string("/slam/weights"));
  pnh_.param("pub_slam_odom", P_.pub_slam_odom, std::string("/slam/odom"));
  
  // Initialize subscribers
  sub_cones_ = nh_.subscribe(P_.topic_tracked, 10, &FastSLAM2::conesCb, this);
  sub_odom_ = nh_.subscribe(P_.topic_odom, 50, &FastSLAM2::odomCb, this);
  if (debug_use_ground_truth_) {
    sub_ground_truth_ = nh_.subscribe("/ground_truth/odom", 10, &FastSLAM2::groundTruthCb, this);
  }
  
  // Initialize publishers
  pub_particles_ = nh_.advertise<geometry_msgs::PoseArray>(P_.pub_particles, 1);
  pub_landmarks_ = nh_.advertise<visualization_msgs::MarkerArray>(P_.pub_landmarks, 1);
  pub_weights_ = nh_.advertise<std_msgs::Float32MultiArray>(P_.pub_weights, 1);
  pub_slam_odom_ = nh_.advertise<nav_msgs::Odometry>(P_.pub_slam_odom, 1);
  
  ROS_INFO("[FastSLAM2] Initialized with %d particles", P_.particles);
}

// spinOnce implementation
void FastSLAM2::spinOnce() {
  // Ensure particles are initialized
  if (particles_.empty()) {
    ensureInitParticles();
  }
}

// Odometry callback implementation
void FastSLAM2::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  have_odom_ = true;
  last_odom_stamp_ = msg->header.stamp;
  
  // Extract velocity from odometry message
  last_v_ = msg->twist.twist.linear.x;
  last_yawrate_ = msg->twist.twist.angular.z;
  
  ROS_DEBUG_THROTTLE(2.0, "[FastSLAM2] Odom: v=%.2f, w=%.2f", last_v_, last_yawrate_);
}

// Ground truth callback (for debugging)
void FastSLAM2::groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg) {
  ground_truth_pose_.x() = msg->pose.pose.position.x;
  ground_truth_pose_.y() = msg->pose.pose.position.y;
  
  tf2::Quaternion quat;
  tf2::fromMsg(msg->pose.pose.orientation, quat);
  double roll, pitch, yaw;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ground_truth_pose_.z() = yaw;
}

// Initialize particles
void FastSLAM2::ensureInitParticles() {
  if (!particles_.empty()) return;
  
  particles_.resize(P_.particles);
  
  // Initialize particles around origin with small spread
  std::normal_distribution<double> noise_x(0.0, 0.1);
  std::normal_distribution<double> noise_y(0.0, 0.1);
  std::normal_distribution<double> noise_yaw(0.0, 0.05);
  
  for (auto& particle : particles_) {
    if (debug_use_ground_truth_ && ground_truth_pose_.norm() > 0) {
      // Initialize around ground truth for debugging
      particle.pose = ground_truth_pose_;
      particle.pose.x() += noise_x(gen_);
      particle.pose.y() += noise_y(gen_);
      particle.pose.z() += noise_yaw(gen_);
    } else {
      // Initialize around origin
      particle.pose = Eigen::Vector3d(noise_x(gen_), noise_y(gen_), noise_yaw(gen_));
    }
    
    particle.weight = 1.0 / P_.particles;
    particle.map.clear();
  }
  
  map_initialized_ = true;
  ROS_INFO("[FastSLAM2] Initialized %d particles", P_.particles);
}

// Particle pose propagation with odometry
void FastSLAM2::integrateOdom(double stamp_sec) {
  if (!have_odom_) return;
  const double dt = std::max(0.0, stamp_sec - last_odom_stamp_.toSec());
  if (dt <= 0.0 || dt > 0.5) return;  // Skip huge dt jumps

  // Scale noise properly with time
  const double dt_clamped = std::min(dt, 0.2);
  const double noise_scale = std::sqrt(dt_clamped);
  
  std::normal_distribution<double> Nv(0.0, P_.odom_v_std * noise_scale);
  std::normal_distribution<double> Nw(0.0, P_.odom_yawrate_std * noise_scale);

  // DEBUG MODE: Reduce noise for validation
  const double noise_reduction = debug_use_ground_truth_ ? 0.1 : 1.0;

  for (auto& particle : particles_) {
    // Simple motion model: constant velocity
    double v_noisy = last_v_ + Nv(gen_) * noise_reduction;
    double w_noisy = last_yawrate_ + Nw(gen_) * noise_reduction;
    
    // Integrate motion
    particle.pose.x() += v_noisy * std::cos(particle.pose.z()) * dt;
    particle.pose.y() += v_noisy * std::sin(particle.pose.z()) * dt;
    particle.pose.z() += w_noisy * dt;
    
    // Wrap angle
    while (particle.pose.z() > M_PI) particle.pose.z() -= 2*M_PI;
    while (particle.pose.z() < -M_PI) particle.pose.z() += 2*M_PI;
  }
  
  last_odom_stamp_ = ros::Time(stamp_sec);
}

// Transform cone detection from LiDAR to map frame
Eigen::Vector2d FastSLAM2::getLandmarkPositionInMap(const Eigen::Vector3d& robot_pose_in_map,
                                                    double lidar_range, double lidar_bearing) const {
  // Convert polar detection to Cartesian in base frame (assuming LiDAR at base)
  Eigen::Vector2d cone_in_base;
  cone_in_base.x() = lidar_range * std::cos(lidar_bearing);
  cone_in_base.y() = lidar_range * std::sin(lidar_bearing);
  
  // Transform to map frame
  Eigen::Matrix2d R_map_base = Rot2(robot_pose_in_map.z());
  Eigen::Vector2d cone_in_map = R_map_base * cone_in_base + robot_pose_in_map.head<2>();
  
  return cone_in_map;
}

// Polar to Cartesian covariance conversion
Eigen::Matrix2d FastSLAM2::polarCovToCart(double r, double th, double r_var, double th_var) const {
  double c = std::cos(th);
  double s = std::sin(th);
  
  // Jacobian of polar to Cartesian conversion [dx/dr, dx/dth; dy/dr, dy/dth]
  Eigen::Matrix2d J;
  J << c, -r*s,
       s,  r*c;
  
  // Polar covariance matrix
  Eigen::Matrix2d R_polar = Eigen::Matrix2d::Zero();
  R_polar(0,0) = std::max(r_var, R_RANGE_VAR_FLOOR);
  R_polar(1,1) = std::max(th_var, R_BEAR_VAR_FLOOR);
  
  // Transform: Cov_cart = J * Cov_polar * J^T + inflation
  Eigen::Matrix2d cov_cart = J * R_polar * J.transpose();
  cov_cart(0,0) += R_CART_INFLATE_XY;
  cov_cart(1,1) += R_CART_INFLATE_XY;
  
  return cov_cart;
}

// Main cone detection callback - FastSLAM 2.0 measurement update
void FastSLAM2::conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!have_odom_) return;
  
  // Ensure particles are initialized
  if (particles_.empty()) {
    ensureInitParticles();
  }
  
  // First propagate particles with odometry to message timestamp
  integrateOdom(msg->header.stamp.toSec());
  
  // Extract cone detections
  std::vector<ConeDetection> detections;
  detections.reserve(msg->cones.size());
  
  for (const auto& cone : msg->cones) {
    if (cone.range < 0.5 || cone.range > 15.0) continue; // Range filter
    detections.push_back({cone.range, cone.bearing, cone.r_var, cone.bearing_var, 
                         cone.color, cone.color_conf, cone.id});
  }
  
  if (detections.empty()) return;

  // FastSLAM 2.0 measurement-informed proposal
  std::vector<double> log_weights(particles_.size(), 0.0);
  
  for (int particle_idx = 0; particle_idx < (int)particles_.size(); ++particle_idx) {
    auto& particle = particles_[particle_idx];
    
    // Collect existing landmark positions and covariances for data association
    std::vector<Eigen::Vector2d> landmark_positions;
    std::vector<Eigen::Matrix2d> landmark_covariances;
    landmark_positions.reserve(particle.map.size());
    landmark_covariances.reserve(particle.map.size());
    
    for (const auto& landmark : particle.map) {
      landmark_positions.push_back(landmark.mu);
      landmark_covariances.push_back(landmark.Sigma);
    }

    // FastSLAM 2.0 improvement: Refine particle pose using measurements
    if (!landmark_positions.empty() && !detections.empty()) {
      particle.pose = refinePoseWithMeasurements(particle.pose, particle.map, detections);
    }

    int successful_associations = 0;

    // Process each cone detection
    for (const auto& detection : detections) {
      // Transform detection to MAP frame using refined particle pose
      Eigen::Vector2d landmark_pos_in_map = getLandmarkPositionInMap(
        particle.pose, detection.range, detection.bearing
      );
      
      // Convert polar covariance to Cartesian in MAP frame
      Eigen::Matrix2d measurement_cov = polarCovToCart(
        detection.range, detection.bearing, 
        detection.r_var, detection.bearing_var
      );

      // Data association in MAP frame
      auto association_result = nn_gated(
        landmark_positions, landmark_covariances, 
        landmark_pos_in_map, measurement_cov, P_.chi2_gate
      );
      int associated_idx = association_result.first;
      double mahalanobis_dist = association_result.second;

      if (associated_idx >= 0) {
        // UPDATE EXISTING LANDMARK in MAP frame
        auto& landmark = particle.map[associated_idx];
        
        // EKF update in MAP frame (H = Identity for landmark position)
        ekf_update_landmark(landmark.mu, landmark.Sigma, landmark_pos_in_map, measurement_cov);
        
        landmark.hits = std::min(landmark.hits + 1, 100);
        landmark.misses = 0;

        // Calculate innovation likelihood for particle weight
        Eigen::Vector2d innovation = landmark_pos_in_map - landmark.mu;
        Eigen::Matrix2d innovation_cov = landmark.Sigma + measurement_cov;
        
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
            log_weights[particle_idx] += std::log(1.1);  // Small bonus
          } else {
            log_weights[particle_idx] += std::log(P_.color_mismatch_penalty);
          }
        } else if (landmark.color == 0 && detection.color > 0 && detection.color_conf > 0.6) {
          landmark.color = detection.color;
          landmark.color_conf = detection.color_conf;
        }

        ++successful_associations;
        
      } else {
        // CREATE NEW LANDMARK - proper initialization in MAP frame
        double likelihood = calculateNewLandmarkLikelihood(detection);
        
        if (likelihood > P_.new_lm_lik_min) {
          Landmark new_landmark;
          new_landmark.mu = landmark_pos_in_map;  // Already in MAP frame
          new_landmark.Sigma = Eigen::Matrix2d::Identity() * (NEW_LM_INIT_STD * NEW_LM_INIT_STD);
          new_landmark.hits = 1;
          new_landmark.misses = 0;
          new_landmark.color = (detection.color > 0 && detection.color_conf > 0.6) ? 
                              detection.color : 0;
          new_landmark.color_conf = new_landmark.color > 0 ? detection.color_conf : 0.0;
          
          particle.map.push_back(new_landmark);
          
          // Add likelihood for new landmark creation
          log_weights[particle_idx] += std::log(likelihood);
        }
      }
    }

    // Apply penalties for poor association
    if (detections.size() > 0) {
      double association_ratio = (double)successful_associations / detections.size();
      if (association_ratio < 0.3) {
        log_weights[particle_idx] += std::log(0.5);  // Penalty for poor association
      }
    }
  }

  // Update particle weights (convert from log domain)
  double max_log_weight = *std::max_element(log_weights.begin(), log_weights.end());
  for (int i = 0; i < (int)particles_.size(); ++i) {
    particles_[i].weight *= std::exp(log_weights[i] - max_log_weight);
  }

  // Normalize weights
  double total_weight = 0.0;
  for (const auto& p : particles_) total_weight += p.weight;
  if (total_weight > 1e-12) {
    for (auto& p : particles_) p.weight /= total_weight;
  }

  // Resample if needed
  double neff = calculateNeff();
  if (neff < P_.neff_ratio * particles_.size()) {
    resampleParticles();
  }

  // Publish all outputs
  publishParticles(msg->header.stamp);
  publishLandmarks(msg->header.stamp);
  publishWeights(msg->header.stamp);
  publishSlamOdom(msg->header.stamp);
  publishMapToOdomTF(msg->header.stamp);
}

// FastSLAM 2.0 measurement-informed pose refinement
Eigen::Vector3d FastSLAM2::refinePoseWithMeasurements(const Eigen::Vector3d& predicted_pose, 
                                                      const std::vector<Landmark>& landmarks,
                                                      const std::vector<ConeDetection>& detections) const {
  if (landmarks.empty() || detections.empty()) return predicted_pose;
  
  // Simple approach: find correspondences and compute centroid correction
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;
  
  for (const auto& detection : detections) {
    Eigen::Vector2d predicted_landmark = getLandmarkPositionInMap(
      predicted_pose, detection.range, detection.bearing
    );
    
    // Find closest landmark
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector2d closest_landmark;
    for (const auto& landmark : landmarks) {
      double dist = (landmark.mu - predicted_landmark).norm();
      if (dist < min_dist && dist < 1.0) { // 1m threshold
        min_dist = dist;
        closest_landmark = landmark.mu;
      }
    }
    
    if (min_dist < 1.0) {
      correspondences.push_back({predicted_landmark, closest_landmark});
    }
  }
  
  if (correspondences.empty()) return predicted_pose;
  
  // Compute centroid correction
  Eigen::Vector2d centroid_pred = Eigen::Vector2d::Zero();
  Eigen::Vector2d centroid_actual = Eigen::Vector2d::Zero();
  
  for (const auto& corr : correspondences) {
    centroid_pred += corr.first;
    centroid_actual += corr.second;
  }
  centroid_pred /= correspondences.size();
  centroid_actual /= correspondences.size();
  
  // Compute translation correction
  Eigen::Vector2d translation_correction = centroid_actual - centroid_pred;
  
  // Apply moderate correction (don't jump too far)
  const double max_correction = 0.5; // 50cm max correction per step
  if (translation_correction.norm() > max_correction) {
    translation_correction = translation_correction.normalized() * max_correction;
  }
  
  Eigen::Vector3d refined_pose = predicted_pose;
  refined_pose.head<2>() += 0.3 * translation_correction; // Conservative correction
  
  return refined_pose;
}

// Publish particles as PoseArray
void FastSLAM2::publishParticles(const ros::Time& t) {
  if (particles_.empty()) return;
  
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = t;
  pose_array.header.frame_id = P_.map_frame;
  
  for (const auto& particle : particles_) {
    geometry_msgs::Pose pose;
    pose.position.x = particle.pose.x();
    pose.position.y = particle.pose.y();
    pose.position.z = 0.0;
    
    tf2::Quaternion quat;
    quat.setRPY(0, 0, particle.pose.z());
    pose.orientation = tf2::toMsg(quat);
    
    pose_array.poses.push_back(pose);
  }
  
  pub_particles_.publish(pose_array);
}

// Publish landmarks as MarkerArray
void FastSLAM2::publishLandmarks(const ros::Time& t) {
  if (particles_.empty()) return;
  
  // Use best particle's map for visualization
  int best_idx = getBestParticleIndex();
  const auto& best_map = particles_[best_idx].map;
  
  visualization_msgs::MarkerArray marker_array;
  
  for (size_t i = 0; i < best_map.size(); ++i) {
    const auto& landmark = best_map[i];
    
    visualization_msgs::Marker marker;
    marker.header.stamp = t;
    marker.header.frame_id = P_.map_frame;
    marker.ns = "landmarks";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Position
    marker.pose.position.x = landmark.mu.x();
    marker.pose.position.y = landmark.mu.y();
    marker.pose.position.z = 0.15; // Half height
    
    // Orientation
    marker.pose.orientation.w = 1.0;
    
    // Scale (size represents uncertainty)
    double uncertainty = std::sqrt(landmark.Sigma.trace());
    marker.scale.x = 0.2 + uncertainty * 0.5;
    marker.scale.y = 0.2 + uncertainty * 0.5;
    marker.scale.z = 0.3;
    
    // Color based on cone color
    marker.color.a = 0.8;
    if (landmark.color == 1) { // Blue
      marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    } else if (landmark.color == 2) { // Yellow
      marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
    } else { // Unknown/Orange
      marker.color.r = 1.0; marker.color.g = 0.5; marker.color.b = 0.0;
    }
    
    marker_array.markers.push_back(marker);
  }
  
  pub_landmarks_.publish(marker_array);
}

// Publish particle weights
void FastSLAM2::publishWeights(const ros::Time& t) {
  if (particles_.empty()) return;
  
  std_msgs::Float32MultiArray weights_msg;
  weights_msg.data.reserve(particles_.size());
  
  for (const auto& particle : particles_) {
    weights_msg.data.push_back(static_cast<float>(particle.weight));
  }
  
  pub_weights_.publish(weights_msg);
}

// Publish SLAM odometry estimate
void FastSLAM2::publishSlamOdom(const ros::Time& t) {
  if (particles_.empty()) return;
  
  Eigen::Vector3d mean_pose = getWeightedMeanPose();
  
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = t;
  odom_msg.header.frame_id = P_.map_frame;
  odom_msg.child_frame_id = P_.base_frame;
  
  // Position
  odom_msg.pose.pose.position.x = mean_pose.x();
  odom_msg.pose.pose.position.y = mean_pose.y();
  odom_msg.pose.pose.position.z = 0.0;
  
  // Orientation
  tf2::Quaternion quat;
  quat.setRPY(0, 0, mean_pose.z());
  odom_msg.pose.pose.orientation = tf2::toMsg(quat);
  
  // Calculate pose covariance from particle spread
  Eigen::Matrix3d pose_cov = Eigen::Matrix3d::Zero();
  for (const auto& particle : particles_) {
    Eigen::Vector3d diff = particle.pose - mean_pose;
    // Wrap angle difference
    while (diff.z() > M_PI) diff.z() -= 2*M_PI;
    while (diff.z() < -M_PI) diff.z() += 2*M_PI;
    
    pose_cov += particle.weight * (diff * diff.transpose());
  }
  
  // Copy to ROS covariance matrix (6x6, but we only fill 3x3 subset)
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (i < 2 && j < 2) {
        odom_msg.pose.covariance[i*6 + j] = pose_cov(i, j);
      } else if (i == 2 && j == 2) {
        odom_msg.pose.covariance[5*6 + 5] = pose_cov(2, 2); // yaw variance
      }
    }
  }
  
  pub_slam_odom_.publish(odom_msg);
}

// Publish map->odom transform
void FastSLAM2::publishMapToOdomTF(const ros::Time& t) {
  Eigen::Vector3d robot_pose_map = getWeightedMeanPose();
  
  // Get current odom->base transform
  geometry_msgs::TransformStamped odom_to_base_tf;
  try {
    odom_to_base_tf = tf_buffer_.lookupTransform(
      P_.odom_frame, P_.base_frame, t, ros::Duration(0.1)
    );
  } catch (tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "[FastSLAM2] Could not get odom->base transform: %s", ex.what());
    return;
  }
  
  // Extract odom->base pose
  Eigen::Vector3d robot_pose_odom;
  robot_pose_odom.x() = odom_to_base_tf.transform.translation.x;
  robot_pose_odom.y() = odom_to_base_tf.transform.translation.y;
  
  tf2::Quaternion q;
  tf2::fromMsg(odom_to_base_tf.transform.rotation, q);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  robot_pose_odom.z() = yaw;
  
  // Compute map->odom transform
  Eigen::Vector3d map_to_odom_pose;
  map_to_odom_pose.head<2>() = robot_pose_map.head<2>() - 
    Rot2(robot_pose_map.z()) * robot_pose_odom.head<2>();
  map_to_odom_pose.z() = robot_pose_map.z() - robot_pose_odom.z();
  
  // Wrap angle
  while (map_to_odom_pose.z() > M_PI) map_to_odom_pose.z() -= 2*M_PI;
  while (map_to_odom_pose.z() < -M_PI) map_to_odom_pose.z() += 2*M_PI;
  
  // Publish transform
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = t;
  transform.header.frame_id = P_.map_frame;
  transform.child_frame_id = P_.odom_frame;
  
  transform.transform.translation.x = map_to_odom_pose.x();
  transform.transform.translation.y = map_to_odom_pose.y();
  transform.transform.translation.z = 0.0;
  
  tf2::Quaternion quat;
  quat.setRPY(0, 0, map_to_odom_pose.z());
  transform.transform.rotation = tf2::toMsg(quat);
  
  tf_broadcaster_.sendTransform(transform);
  
  ROS_DEBUG_THROTTLE(1.0, "[FastSLAM2] Published map->odom: (%.2f,%.2f,%.1f°)", 
                     map_to_odom_pose.x(), map_to_odom_pose.y(), 
                     map_to_odom_pose.z() * 180.0 / M_PI);
}

// Get weighted mean pose from particles
Eigen::Vector3d FastSLAM2::getWeightedMeanPose() const {
  if (particles_.empty()) return Eigen::Vector3d::Zero();
  
  // Handle circular mean for angle
  double sum_cos = 0.0, sum_sin = 0.0;
  Eigen::Vector2d mean_pos = Eigen::Vector2d::Zero();
  
  for (const auto& particle : particles_) {
    mean_pos += particle.weight * particle.pose.head<2>();
    sum_cos += particle.weight * std::cos(particle.pose.z());
    sum_sin += particle.weight * std::sin(particle.pose.z());
  }
  
  double mean_yaw = std::atan2(sum_sin, sum_cos);
  
  Eigen::Vector3d mean_pose;
  mean_pose.head<2>() = mean_pos;
  mean_pose.z() = mean_yaw;
  
  return mean_pose;
}

// Get index of particle with highest weight
int FastSLAM2::getBestParticleIndex() const {
  if (particles_.empty()) return -1;
  
  int best_idx = 0;
  double max_weight = particles_[0].weight;
  
  for (int i = 1; i < (int)particles_.size(); ++i) {
    if (particles_[i].weight > max_weight) {
      max_weight = particles_[i].weight;
      best_idx = i;
    }
  }
  
  return best_idx;
}

// Calculate effective number of particles
double FastSLAM2::calculateNeff() const {
  double sum_weights_squared = 0.0;
  for (const auto& p : particles_) {
    sum_weights_squared += p.weight * p.weight;
  }
  return sum_weights_squared > 1e-12 ? 1.0 / sum_weights_squared : 0.0;
}

// Systematic resampling
void FastSLAM2::resampleParticles() {
  if (particles_.empty()) return;
  
  std::vector<Particle> new_particles;
  new_particles.reserve(particles_.size());
  
  // Build cumulative distribution
  std::vector<double> cdf(particles_.size());
  cdf[0] = particles_[0].weight;
  for (int i = 1; i < (int)particles_.size(); ++i) {
    cdf[i] = cdf[i-1] + particles_[i].weight;
  }
  
  // Systematic resampling
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  double step = 1.0 / particles_.size();
  double start = uniform(gen_) * step;
  
  for (int i = 0; i < (int)particles_.size(); ++i) {
    double target = start + i * step;
    int idx = std::lower_bound(cdf.begin(), cdf.end(), target) - cdf.begin();
    idx = std::min(idx, (int)particles_.size() - 1);
    
    new_particles.push_back(particles_[idx]);
    new_particles.back().weight = 1.0 / particles_.size(); // Reset weights
  }
  
  particles_ = new_particles;
  
  ROS_DEBUG("[FastSLAM2] Resampled %zu particles", particles_.size());
}

// Calculate likelihood for new landmark creation
double FastSLAM2::calculateNewLandmarkLikelihood(const ConeDetection& detection) const {
  // Base likelihood decreases with range (further landmarks less certain)
  double range_factor = std::exp(-detection.range / 10.0);
  
  // Increase likelihood if good color confidence
  double color_factor = detection.color > 0 ? 
    (1.0 + detection.color_conf) : 1.0;
  
  // Combined likelihood
  return P_.new_lm_lik_min * range_factor * color_factor;
}

// Rotation matrix helper
Eigen::Matrix2d FastSLAM2::Rot2(double theta) const {
  Eigen::Matrix2d R;
  double c = std::cos(theta);
  double s = std::sin(theta);
  R << c, -s,
       s,  c;
  return R;
}