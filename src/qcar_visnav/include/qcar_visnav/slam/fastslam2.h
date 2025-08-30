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
#include "data_association.h"
#include "measurement_model.h"

namespace qcar_visnav { namespace slam {

/**
 * FastSLAM 2.0 Implementation for QCar
 * (unchanged high-level description)
 */
class FastSLAM2 {
public:
  FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  // ROS I/F
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_cones_;
  ros::Publisher  pub_particles_;
  ros::Publisher  pub_particles_posearray_;
  ros::Publisher  pub_map_markers_;
  ros::Publisher  pub_slam_odom_;
  ros::Publisher  pub_slam_odom_mean_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  // Config params
  std::string map_frame_, odom_frame_, base_frame_, lidar_frame_;
  int    N_{120};
  double neff_ratio_{0.4};
  double chi2_gate_{9.21};
  int    confirm_hits_{2};
  double min_new_lm_dist_{0.35};
  double merge_R_scale_{4.0};
  double landmark_prior_var_{0.25};
  bool   seed_from_params_{true};
  bool   overwrite_with_odom_on_first_msg_{true};
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0};
  double spread_x_{0.02}, spread_y_{0.02}, spread_yaw_{0.01};
  std::string odom_topic_;
  double odom_buffer_window_sec_{2.0};

  // State
  std::vector<Particle> P_;
  MotionModel     motion_;
  DataAssociation assoc_;

  bool particles_initialized_{false};
  bool snapped_to_first_odom_{false};

  struct OdomStamped {
    ros::Time t;
    double vx, vy, r;
  };
  std::deque<OdomStamped> odom_buf_;
  ros::Time last_prop_stamp_{ros::Time(0)};

  int    best_idx_{0};
  double mean_x_{0.0}, mean_y_{0.0}, mean_yaw_{0.0};

  // RNG for resampling
  std::mt19937 rng_{std::random_device{}()};

  // --- callbacks / core ---
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void cbCones(const qcar_visnav::ConeArray::ConstPtr& msg);

  void initializeParticlesFrom(double x, double y, double yaw,
                               double sx, double sy, double syaw);
  void propagateParticlesTo(const ros::Time& t);
  OdomStamped interpTwist(const OdomStamped& a,
                          const OdomStamped& b,
                          const ros::Time& s) const;

  static int nearestLandmarkIdx(const Particle& p,
                                const Eigen::Vector2d& z,
                                double& out_dist_sq);

  void processMeasurementAt(const ros::Time& t,
                            const std::vector<MeasRB>& meas_vec,
                            const LidarExtrinsics& ex);

  void publishViz(const ros::Time& t);

  // TF helper for dynamic LiDAR extrinsics
  bool lookupLidarExtrinsics(const ros::Time& t, LidarExtrinsics& ex) const;
};

}} // namespace
