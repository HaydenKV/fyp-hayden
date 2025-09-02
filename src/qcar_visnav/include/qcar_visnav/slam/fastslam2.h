#pragma once
#include <ros/ros.h>
#include <deque>
#include <string>
#include <vector>
#include <limits>
#include <random>

#include <nav_msgs/Odometry.h>
#include <qcar_visnav/ConeArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Core>

#include "particle.h"
#include "motion_model.h"
#include "measurement_model.h"
#include "data_association.h"

namespace qcar_visnav { namespace slam {

/**
 * Enhanced FastSLAM 2.0 for QCar
 *
 * - Particles over robot pose
 * - Per-particle EKF landmarks (world frame)
 * - Enhanced data association (Hungarian/JCBB)
 * - Improved pose proposal with information weighting
 * - Velocity-aware motion model
 * - World-space merge-before-birth
 * - Birth buffer with K-hit promotion
 */
class FastSLAM2 {
public:
  FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  // ROS interface
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_cones_;
  ros::Publisher  pub_particles_;
  ros::Publisher  pub_particles_posearray_;
  ros::Publisher  pub_map_markers_;
  ros::Publisher  pub_slam_odom_;
  ros::Publisher  pub_slam_odom_mean_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  // --- Config params (YAML-driven) ---
  std::string map_frame_, odom_frame_, base_frame_, lidar_frame_;

  // PF core
  int    N_{80};
  double neff_ratio_{0.5};

  // Enhanced data association
  std::string association_method_{"hungarian"};  // "greedy", "hungarian", "jcbb"
  double chi2_gate_{7.38};              // RB Mahalanobis gate
  double chi2_gate_world_{9.21};        // World-space χ² gate for merge-before-birth
  double ambiguity_threshold_{0.7};     // Threshold for ambiguous associations
  bool   use_jcbb_{false};              // Enable JCBB for highest quality
  std::unique_ptr<DataAssociation> data_assoc_;

  // Enhanced motion model parameters
  double velocity_noise_scale_{0.4};
  double min_velocity_for_scaling_{0.12};
  double turning_noise_scale_{0.6};
  double min_yawrate_for_turning_{0.08};
  double max_position_std_{0.08};
  double max_yaw_std_{0.12};

  // Landmarks / births / penalties
  int    confirm_hits_{3};
  double min_new_lm_dist_{0.25};
  double merge_R_scale_{4.0};
  double landmark_prior_var_{0.15};

  int    birth_required_hits_{3};
  int    birth_max_age_{10};
  double birth_promote_radius_{0.25};

  double unconfirmed_R_scale_{1.8};
  int    prune_unconfirmed_misses_{10};
  int    prune_stale_misses_{35};

  // Enhanced pose proposal
  double proposal_max_sigma_trace_{0.25};
  int    proposal_sample_every_k_{0};
  double min_information_for_proposal_{2.0};
  double high_quality_threshold_{0.8};

  // Enhanced weight penalties
  double new_landmark_penalty_{0.8};
  double fov_miss_penalty_{0.3};
  double high_quality_bonus_{0.1};
  double information_bonus_scale_{0.05};

  // Sensor FoV
  double fov_range_max_{18.0};
  double fov_bearing_rad_{M_PI * 85.0 / 180.0};

  // Initialization
  bool   seed_from_params_{true};
  bool   overwrite_with_odom_on_first_msg_{true};
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0};
  double spread_x_{0.03}, spread_y_{0.03}, spread_yaw_{0.05};
  std::string odom_topic_;
  double odom_buffer_window_sec_{2.0};
  bool   mapping_enabled_{true};
  bool   freeze_after_first_loop_{false};

  // --- State ---
  std::vector<Particle> P_;
  MotionModel     motion_;
  MotionNoise     motion_noise_;

  bool particles_initialized_{false};
  bool snapped_to_first_odom_{false};

  struct OdomStamped {
    ros::Time t;
    double vx, vy, r;  // body-frame twist
  };
  std::deque<OdomStamped> odom_buf_;
  ros::Time last_prop_stamp_{ros::Time(0)};

  int    best_idx_{0};
  double mean_x_{0.0}, mean_y_{0.0}, mean_yaw_{0.0};

  // RNG for resampling / sampling proposal
  std::mt19937 rng_{std::random_device{}()};

  // --- Callbacks / core ---
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void cbCones(const qcar_visnav::ConeArray::ConstPtr& msg);

  void initializeParticlesFrom(double x, double y, double yaw,
                               double sx, double sy, double syaw);
  void propagateParticlesTo(const ros::Time& t);
  OdomStamped interpTwist(const OdomStamped& a,
                          const OdomStamped& b,
                          const ros::Time& s) const;

  void processMeasurementAt(const ros::Time& t,
                            const std::vector<MeasRB>& meas_vec,
                            const LidarExtrinsics& ex);

  void publishViz(const ros::Time& t);

  // TF helper for dynamic LiDAR extrinsics
  bool lookupLidarExtrinsics(const ros::Time& t, LidarExtrinsics& ex) const;

  // --- Enhanced helper functions ---
  bool inFoV(const Particle& p,
             const Eigen::Vector2d& mu_world,
             const LidarExtrinsics& ex) const;

  double worldMaha2(const Eigen::Vector2d& z_world,
                    const Eigen::Matrix2d& Rw,
                    const Landmark& lm) const;

  // Enhanced pose proposal with information weighting
  void computeEnhancedPoseProposal(Particle& p,
                                   const GlobalAssignment& assignment,
                                   const std::vector<MeasRB>& meas_vec,
                                   const LidarExtrinsics& ex,
                                   double dt,
                                   double& log_proposal_correction);

  // Process unmatched measurements with world-space merging
  void processUnmatchedMeasurements(Particle& p,
                                    const std::vector<int>& unmatched_indices,
                                    const std::vector<MeasRB>& meas_vec,
                                    const LidarExtrinsics& ex,
                                    int& promotions_this_frame);

  // Update matched landmarks using EKF
  void updateMatchedLandmarks(Particle& p,
                              const GlobalAssignment& assignment,
                              const std::vector<MeasRB>& meas_vec,
                              const LidarExtrinsics& ex,
                              std::vector<bool>& lm_seen);

  // Apply weight penalties and bonuses
  void applyWeightAdjustments(Particle& p,
                              const GlobalAssignment& assignment,
                              const std::vector<bool>& lm_seen,
                              const LidarExtrinsics& ex,
                              int promotions_this_frame);

  // Age and promote birth tracks
  void manageBirthTracks(Particle& p, int& promotions_this_frame);
};

}} // namespace