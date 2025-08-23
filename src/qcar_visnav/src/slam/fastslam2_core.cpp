// Fixed FastSLAM 2.0 Core Implementation
// Key fixes: proper coordinate transforms, particle propagation, measurement-informed proposals

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

// Constants - adjusted for better performance
static constexpr double R_RANGE_VAR_FLOOR  = 0.05 * 0.05;   // Reduced floor
static constexpr double R_BEAR_VAR_FLOOR   = (2.0*M_PI/180.0)*(2.0*M_PI/180.0); 
static constexpr double R_CART_INFLATE_XY  = 0.10 * 0.10;   // Reduced inflation
static constexpr double NEW_LM_INIT_STD    = 0.50;          // Reduced initial std

// CRITICAL FIX 1: Proper particle pose propagation
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

  // FIXED: Actually propagate each particle's pose
  for (auto& particle : particles_) {
    // Add odometry motion with noise
    const double v_noisy = last_v_ + Nv(gen_) * noise_reduction;
    const double w_noisy = last_yawrate_ + Nw(gen_) * noise_reduction;
    
    // Integrate motion (simple bicycle model)
    particle.pose.x() += v_noisy * std::cos(particle.pose.z()) * dt;
    particle.pose.y() += v_noisy * std::sin(particle.pose.z()) * dt;
    particle.pose.z() += w_noisy * dt;
    
    // Wrap angle
    while (particle.pose.z() > M_PI) particle.pose.z() -= 2*M_PI;
    while (particle.pose.z() < -M_PI) particle.pose.z() += 2*M_PI;
  }

  ROS_DEBUG_THROTTLE(1.0, "[FastSLAM2] Propagated %zu particles, dt=%.3f, v=%.2f, w=%.2f", 
                     particles_.size(), dt, last_v_, last_yawrate_);
}

// CRITICAL FIX 2: Proper landmark coordinate transformation 
Eigen::Vector2d FastSLAM2::getLandmarkPositionInMap(const Eigen::Vector3d& particle_pose_in_map,
                                                    double lidar_range, double lidar_bearing) const
{
  // Step 1: LiDAR polar -> LiDAR Cartesian
  Eigen::Vector2d cone_in_lidar;
  cone_in_lidar.x() = lidar_range * std::cos(lidar_bearing);
  cone_in_lidar.y() = lidar_range * std::sin(lidar_bearing);
  
  // Step 2: LiDAR -> Base (apply sensor extrinsics)
  Eigen::Matrix2d R_base_lidar = Rot2(P_.yaw_bl);
  Eigen::Vector2d cone_in_base = R_base_lidar * cone_in_lidar + P_.t_bl;
  
  // Step 3: Base -> Map (using particle's pose in map frame)
  // This is the CRITICAL transformation that was causing landmarks at origin!
  Eigen::Matrix2d R_map_base = Rot2(particle_pose_in_map.z());
  Eigen::Vector2d cone_in_map = R_map_base * cone_in_base + particle_pose_in_map.head<2>();
  
  return cone_in_map;
}

// CRITICAL FIX 3: FastSLAM 2.0 measurement-informed proposal distribution
void FastSLAM2::conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
  if (!have_odom_) return;
  
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

  // FIXED: Implement FastSLAM 2.0 measurement-informed proposal
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

    // FASTSLAM 2.0 IMPROVEMENT: Refine particle pose using measurements
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
        // CREATE NEW LANDMARK - FIXED: proper initialization in MAP frame
        double likelihood = calculateNewLandmarkLikelihood(detection);
        
        if (likelihood > P_.new_lm_lik_min) {
          Landmark new_landmark;
          new_landmark.mu = landmark_pos_in_map;  // Already in MAP frame
          new_landmark.Sigma = Eigen::Matrix2d::Identity() * (NEW_LM_INIT_STD * NEW_LM_INIT_STD);
          new_landmark.hits = 1;
          new_landmark.misses = 0;
          new_landmark.color = (detection.color > 0 && detection.color_conf > 0.6) ? detection.color : 0;
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

// CRITICAL FIX 4: FastSLAM 2.0 measurement-informed pose refinement
Eigen::Vector3d FastSLAM2::refinePoseWithMeasurements(const Eigen::Vector3d& predicted_pose, 
                                                       const std::vector<Landmark>& landmarks,
                                                       const std::vector<ConeDetection>& detections) const {
  if (landmarks.empty() || detections.empty()) return predicted_pose;
  
  // Find best matching pairs for pose refinement
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> correspondences;
  
  for (const auto& detection : detections) {
    Eigen::Vector2d predicted_landmark_map = getLandmarkPositionInMap(
      predicted_pose, detection.range, detection.bearing
    );
    
    // Find closest existing landmark
    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector2d best_match;
    bool found_match = false;
    
    for (const auto& landmark : landmarks) {
      double dist = (landmark.mu - predicted_landmark_map).norm();
      if (dist < min_dist && dist < 1.0) { // 1m association threshold
        min_dist = dist;
        best_match = landmark.mu;
        found_match = true;
      }
    }
    
    if (found_match) {
      correspondences.emplace_back(predicted_landmark_map, best_match);
    }
  }
  
  if (correspondences.size() < 2) return predicted_pose; // Need at least 2 correspondences
  
  // Simple pose correction using weighted least squares
  Eigen::Vector2d centroid_pred = Eigen::Vector2d::Zero();
  Eigen::Vector2d centroid_actual = Eigen::Vector2d::Zero();
  
  for (const auto& pair : correspondences) {
    centroid_pred += pair.first;
    centroid_actual += pair.second;
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

// CRITICAL FIX 5: Proper map-to-odom transform publication
void FastSLAM2::publishMapToOdomTF(const ros::Time& t) {
  Eigen::Vector3d robot_pose_map = getWeightedMeanPose();
  
  // FIXED: Compute proper map->odom transform
  // T_map_odom = T_map_base * T_base_odom^-1
  
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

// HELPER: Calculate effective number of particles
double FastSLAM2::calculateNeff() const {
  double sum_weights_squared = 0.0;
  for (const auto& p : particles_) {
    sum_weights_squared += p.weight * p.weight;
  }
  return sum_weights_squared > 1e-12 ? 1.0 / sum_weights_squared : 0.0;
}

// HELPER: Systematic resampling
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

// HELPER: Calculate likelihood for new landmark creation
double FastSLAM2::calculateNewLandmarkLikelihood(const ConeDetection& detection) const {
  // Base likelihood decreases with range (further landmarks less certain)
  double range_factor = std::exp(-detection.range / 10.0);
  
  // Increase likelihood if good color confidence
  double color_factor = detection.color > 0 ? (1.0 + detection.color_conf) : 1.0;
  
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