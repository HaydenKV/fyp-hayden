#pragma once
#include <ros/ros.h>
#include <deque>
#include <string>
#include <vector>
#include <limits>

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

namespace qcar_visnav { namespace slam {

class FastSLAM2 {
public:
  FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  // ----- ROS I/O -----
  ros::Subscriber sub_odom_;             // now /odom (truth)
  ros::Subscriber sub_cones_;
  ros::Publisher  pub_particles_;
  ros::Publisher  pub_particles_posearray_;
  ros::Publisher  pub_map_markers_;
  ros::Publisher  pub_slam_odom_;
  ros::Publisher  pub_slam_odom_mean_;
  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;

  // ----- Frames & params -----
  std::string map_frame_, odom_frame_, base_frame_, lidar_frame_;
  int    N_{120};
  double neff_ratio_{0.4};
  double chi2_gate_{16.27};
  int    confirm_hits_{2};         // hits required to confirm a landmark
  double min_new_lm_dist_{0.35};   // m: suppress spawn if near an existing landmark
  double merge_R_scale_{4.0};      // inflate R when merging near-duplicates

  // init behavior
  bool   seed_from_params_{true};
  bool   overwrite_with_odom_on_first_msg_{true};
  double init_x_{0.0}, init_y_{0.0}, init_yaw_{0.0};
  double spread_x_{0.02}, spread_y_{0.02}, spread_yaw_{0.01};

  // twist semantics
  bool odom_twist_in_world_{false}; // if true, convert world->body per-particle before integrate

  // ----- SLAM state -----
  std::vector<Particle> P_;
  MotionModel     motion_;
  DataAssociation assoc_;
  bool particles_initialized_{false};
  bool snapped_to_first_odom_{false};

  // odom ring buffer (latest ~2s)
  struct OdomStamped { ros::Time t; double vx, vy, r; };
  std::deque<OdomStamped> odom_buf_;
  ros::Time last_prop_stamp_{ros::Time(0)};

  // Cached best & mean (for publish outputs)
  int    best_idx_{0};
  double mean_x_{0.0}, mean_y_{0.0}, mean_yaw_{0.0};

  // ----- Callbacks -----
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  void cbCones(const qcar_visnav::ConeArray::ConstPtr& msg);

  // ----- Core steps -----
  void initializeParticlesFrom(double x, double y, double yaw,
                               double sx, double sy, double syaw);
  void propagateParticlesTo(const ros::Time& t);
  OdomStamped interpTwist(const OdomStamped& a,
                          const OdomStamped& b,
                          const ros::Time& s) const;

  // utility: convert world twist -> body twist for a given yaw
  static inline void worldToBody(double yaw,
                                 double vx_w, double vy_w,
                                 double& vx_b, double& vy_b) {
    const double c = std::cos(yaw), s = std::sin(yaw);
    vx_b =  c*vx_w + s*vy_w;
    vy_b = -s*vx_w + c*vy_w;
  }

  // landmark utilities
  static int nearestLandmarkIdx(const Particle& p,
                                const Eigen::Vector2d& z,
                                double& out_dist_sq);

  void processMeasurementAt(const ros::Time& t,
                            const std::vector<Eigen::Vector2d>& zs,
                            const std::vector<Eigen::Matrix2d>& Rs);

  // ----- Visualization & outputs -----
  void publishViz(const ros::Time& t);
};

}} // namespace
