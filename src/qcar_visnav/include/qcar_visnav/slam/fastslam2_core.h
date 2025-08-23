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
#include "types.h"  // Use existing types.h

namespace qcar_visnav {
namespace slam {

// Cone detection structure (local to this module)
struct ConeDetection {
  double range;
  double bearing;
  double r_var;
  double bearing_var;
  int color;
  double color_conf;
  int id;
};

// Extended FS2Params with FastSLAM 2.0 specific parameters
struct FS2Params {
  // Topics
  std::string topic_tracked{"/tracked_cones"};
  std::string topic_odom{"/ekf/odom"};
  std::string pub_particles{"/slam/particles"};
  std::string pub_landmarks{"/map_markers"};
  std::string pub_weights{"/slam/weights"}; 
  std::string pub_slam_odom{"/slam/odom"};

  // Frames/extrinsics
  std::string map_frame{"map"};
  std::string odom_frame{"odom"};
  std::string base_frame{"base_footprint"};
  std::string lidar_frame{"lidar"};

  // Core algorithm parameters
  int    particles{80};
  double neff_ratio{0.7};
  double chi2_gate{7.38};          // 97.5% confidence for 2 DOF
  double new_lm_lik_min{0.4};      // Threshold for new landmark creation

  // Noise parameters
  Eigen::Vector2d meas_noise_xy{0.15, 0.15}; // Reduced measurement noise
  double odom_v_std{0.12};         // Linear velocity noise
  double odom_yawrate_std{0.15};   // Angular velocity noise

  // Robustness penalties
  double miss_in_fov_penalty{0.7};
  double color_mismatch_penalty{0.8};

  // LiDAR-to-base extrinsics (2D)
  Eigen::Vector2d t_bl{Eigen::Vector2d::Zero()};
  double yaw_bl{0.0};
  
  // FastSLAM 2.0 specific parameters
  bool pose_refinement_enabled{true};
  double max_pose_correction_m{0.5};
  double pose_correction_weight{0.3};
};

class FastSLAM2 {
public:
  explicit FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  // Callback functions
  void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg);

  // Core SLAM functions
  void ensureInitParticles();
  void integrateOdom(double stamp);
  
  // FastSLAM 2.0 specific functions
  Eigen::Vector3d refinePoseWithMeasurements(const Eigen::Vector3d& predicted_pose, 
                                             const std::vector<Landmark>& landmarks,
                                             const std::vector<ConeDetection>& detections) const;
  
  // Coordinate transformation functions
  Eigen::Vector2d getLandmarkPositionInMap(const Eigen::Vector3d& robot_pose_in_map,
                                          double lidar_range, double lidar_bearing) const;
  Eigen::Matrix2d polarCovToCart(double r, double th, double r_var, double th_var) const;
  Eigen::Matrix2d Rot2(double theta) const;

  // Particle management
  double calculateNeff() const;
  void resampleParticles();
  int getBestParticleIndex() const;
  Eigen::Vector3d getWeightedMeanPose() const;
  
  // Landmark management
  double calculateNewLandmarkLikelihood(const ConeDetection& detection) const;
  
  // Publishing functions
  void publishParticles(const ros::Time& t);
  void publishLandmarks(const ros::Time& t);
  void publishWeights(const ros::Time& t);
  void publishSlamOdom(const ros::Time& t);
  void publishMapToOdomTF(const ros::Time& t);

  // ROS infrastructure
  ros::NodeHandle nh_, pnh_;
  FS2Params P_;

  // Subscribers and publishers
  ros::Subscriber sub_cones_, sub_odom_, sub_ground_truth_;
  ros::Publisher  pub_particles_, pub_landmarks_, pub_weights_;
  ros::Publisher  pub_slam_odom_;
  
  // TF handling
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Algorithm state
  std::vector<Particle> particles_;
  std::mt19937 gen_{std::random_device{}()};

  // Odometry state
  bool have_odom_{false};
  ros::Time last_odom_stamp_;
  double last_v_{0.0};
  double last_yawrate_{0.0};

  // Debug mode state
  bool debug_use_ground_truth_{false};
  Eigen::Vector3d ground_truth_pose_{Eigen::Vector3d::Zero()};

  // Map initialization
  bool map_initialized_{false};
  Eigen::Vector3d initial_pose_{Eigen::Vector3d::Zero()};
};

} // namespace slam
} // namespace qcar_visnav