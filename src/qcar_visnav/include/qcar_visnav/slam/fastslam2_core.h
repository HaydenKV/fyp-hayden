#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <qcar_visnav/ConeArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <random>
#include "types.h"

namespace qcar_visnav {
namespace slam {

struct FS2Params {
  // topics
  std::string topic_tracked{"/tracked_cones"};
  std::string topic_odom{"/ekf/odom"};
  std::string pub_particles{"/slam/particles"};
  std::string pub_landmarks{"/map_markers"};
  std::string pub_weights{"/slam/weights"}; 
  std::string pub_slam_odom{"/slam/odom"};  // ADDED: SLAM pose output

  // frames/extrinsics
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  std::string lidar_frame{"lidar"};

  // core
  int    particles{120};
  double neff_ratio{0.6};
  double chi2_gate{9.21};      // ~99% for 2 dof
  double new_lm_lik_min{0.25}; // threshold to spawn

  // measurement noise (cartesian) — approximate projection of polar
  Eigen::Vector2d meas_noise_xy{0.25, 0.25}; // m^2

  // odom noise (v, yawrate) standard deviations
  double odom_v_std{0.15};
  double odom_yawrate_std{0.20};

  // penalties
  double miss_in_fov_penalty{0.5};
  double color_mismatch_penalty{0.7};

  // lidar-to-base extrinsics (2D)
  Eigen::Vector2d t_bl{Eigen::Vector2d::Zero()};
  double yaw_bl{0.0};
};

class FastSLAM2 {
public:
  explicit FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce(); // no periodic compute now, but we keep the hook

private:
  // callbacks
  void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg);  // ADDED: Debug mode

  // helpers
  void ensureInitParticles();
  void integrateOdom(double stamp);
  void publishParticles(const ros::Time& t);
  void publishLandmarks(const ros::Time& t);
  void publishWeights(const ros::Time& t);
  void publishSlamOdom(const ros::Time& t);    // ADDED: Publish robot pose estimate
  void publishMapToOdomTF(const ros::Time& t); // ADDED: Publish TF transform

  // FIXED: Proper landmark coordinate transformation
  inline Eigen::Vector2d getLandmarkPositionInMap(const Eigen::Vector3d& robot_pose_in_map,
                                                  double lidar_range, double lidar_bearing) const;

  inline Eigen::Matrix2d polarCovToCart(double r, double th,
                                        double r_var, double th_var) const;

  // ADDED: Particle utilities
  int getBestParticleIndex() const;
  Eigen::Vector3d getWeightedMeanPose() const;

private:
  ros::NodeHandle nh_, pnh_;
  FS2Params P_;

  ros::Subscriber sub_cones_, sub_odom_, sub_ground_truth_;  // ADDED: ground truth sub
  ros::Publisher  pub_particles_, pub_landmarks_, pub_weights_;
  ros::Publisher  pub_slam_odom_;  // ADDED: SLAM pose publisher
  
  tf2_ros::TransformBroadcaster tf_broadcaster_;  // ADDED: TF broadcaster
  tf2_ros::Buffer tf_buffer_;                     // ADDED: TF buffer  
  tf2_ros::TransformListener tf_listener_;        // ADDED: TF listener

  // state
  std::vector<Particle> particles_;
  std::mt19937 gen_{std::random_device{}()};

  // last odom (for propagation)
  bool have_odom_{false};
  ros::Time last_odom_stamp_;
  double last_v_{0.0};
  double last_yawrate_{0.0};

  // ADDED: Debug mode and ground truth tracking
  bool debug_use_ground_truth_{false};
  Eigen::Vector3d ground_truth_pose_{Eigen::Vector3d::Zero()};

  // ADDED: Map initialization tracking  
  bool map_initialized_{false};
  Eigen::Vector3d initial_pose_{Eigen::Vector3d::Zero()};
};

} // namespace slam
} // namespace qcar_visnav