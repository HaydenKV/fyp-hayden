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

namespace qcar_visnav { namespace slam {

/**
 * FastSLAM 2.0 for QCar
 *
 * - Particles over robot pose
 * - Per-particle EKF landmarks (world)
 * - RB measurement model
 * - Improved pose proposal (FS2.0)
 * - Per-particle exclusive meas↔LM assignment
 * - World-space merge-before-birth
 * - Birth buffer with K-hit promotion
 * - Optional penalties to discourage duplicates/FoV misses
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
  int    N_{120};
  double neff_ratio_{0.4};
  double chi2_gate_{9.21};         // RB Mahalanobis gate (per measurement)
  double chi2_gate_world_{9.21};   // World-space χ² gate for merge-before-birth

  // Data association (reserved hook)
  double euclid_gate_{0.5};

  // Landmarks / births / penalties
  int    confirm_hits_{2};
  double min_new_lm_dist_{0.35};   // retained for completeness (not used directly now)
  double merge_R_scale_{4.0};      // retained; merges now use χ² in world
  double landmark_prior_var_{0.25};

  int    birth_required_hits_{3};
  int    birth_max_age_{10};
  double birth_promote_radius_{0.30};

  double unconfirmed_R_scale_{2.0};
  int    prune_unconfirmed_misses_{8}; // reserved
  int    prune_stale_misses_{25};      // reserved

  double proposal_max_sigma_trace_{0.20};
  int    proposal_sample_every_k_{0};

  double new_landmark_penalty_{1.0};
  double fov_miss_penalty_{0.4};

  double fov_range_max_{20.0};
  double fov_bearing_rad_{M_PI/2.0};

  bool   seed_from_params_{true};
  bool   overwrite_with_odom_on_first_msg_{true};
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0};
  double spread_x_{0.02}, spread_y_{0.02}, spread_yaw_{0.01};
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

  // --- Small helpers ---
  bool inFoV(const Particle& p,
             const Eigen::Vector2d& mu_world,
             const LidarExtrinsics& ex) const;

  double worldMaha2(const Eigen::Vector2d& z_world,
                    const Eigen::Matrix2d& Rw,
                    const Landmark& lm) const;

  // Greedy 1-1 assignment: RB gating candidates and return matched pairs
  struct MatchPair {
    int k; // meas index
    int j; // landmark index
    double d2;
    Eigen::Matrix2d S;
    Eigen::Matrix2d Hlm;
    Eigen::Matrix<double,2,3> Gx;
    Eigen::Vector2d nu;
    Eigen::Matrix2d R;
  };
  void buildGreedyMatches(const Particle& p,
                          const std::vector<MeasRB>& meas_vec,
                          const LidarExtrinsics& ex,
                          std::vector<MatchPair>& out_matches,
                          std::vector<int>& out_unmatched_meas) const;
};

}} // namespace
