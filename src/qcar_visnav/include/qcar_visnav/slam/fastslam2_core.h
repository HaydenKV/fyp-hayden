#pragma once
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <qcar_visnav/ConeArray.h>
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
  std::string pub_weights{"/slam/weights"}; // optional

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

  // helpers
  void ensureInitParticles();
  void integrateOdom(double stamp);
  void publishParticles(const ros::Time& t);
  void publishLandmarks(const ros::Time& t);
  void publishWeights(const ros::Time& t);

  // measurement projection: lidar polar -> odom Cartesian using particle pose
  inline Eigen::Vector2d lidarPolarToOdomXY(const Eigen::Vector3d& base_pose,
                                            double r, double th) const;

  inline Eigen::Matrix2d polarCovToCart(double r, double th,
                                        double r_var, double th_var) const;

private:
  ros::NodeHandle nh_, pnh_;
  FS2Params P_;

  ros::Subscriber sub_cones_, sub_odom_;
  ros::Publisher  pub_particles_, pub_landmarks_, pub_weights_;

  // state
  std::vector<Particle> particles_;
  std::mt19937 gen_{std::random_device{}()};

  // last odom (for propagation)
  bool have_odom_{false};
  ros::Time last_odom_stamp_;
  double last_v_{0.0};
  double last_yawrate_{0.0};
};

} // namespace slam
} // namespace qcar_visnav
