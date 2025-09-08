#pragma once
#include <deque>
#include <random>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>

#include "qcar_visnav/ConeArray.h"
#include "qcar_visnav/slam/particle.h"
#include "qcar_visnav/slam/motion_model.h"
#include "qcar_visnav/slam/data_association.h"
#include "qcar_visnav/slam/measurement_model.h"
#include "qcar_visnav/slam/resampling.h"

namespace qcar_visnav { namespace slam {

class FastSLAM2 {
public:
  FastSLAM2(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  struct OdomStamped {
    ros::Time t;
    double vx{0.0}, vy{0.0}, r{0.0}; // body-frame twist
  };

  // === Params ===
  std::string map_frame_{"odom"}, odom_frame_{"odom"};
  std::string base_frame_{"base_footprint"}, lidar_frame_{"lidar"};
  std::string odom_topic_{"/odom"};

  int    N_{80};
  double neff_ratio_{0.5};

  MotionNoise motion_noise_;
  MotionModel motion_;

  // Association mode: "id" or "nn_rb"
  std::string assoc_mode_{"id"};
  double chi2_gate_rb_{5.99};

  // Landmark management
  double lm_init_var_{0.15};
  int    lm_confirm_hits_{2};
  int    lm_lock_hits_{6};
  double lm_lock_cov_trace_{0.02};

  bool   seed_from_params_{true};
  bool   overwrite_with_odom_on_first_msg_{false};
  double init_x_{0}, init_y_{0}, init_yaw_{0};
  double spread_x_{0.03}, spread_y_{0.03}, spread_yaw_{0.05};
  double odom_buffer_window_sec_{2.0};

  // === State ===
  std::vector<Particle> P_;
  bool particles_initialized_{false};
  bool snapped_to_first_odom_{false};
  int  best_idx_{0};
  double mean_x_{0}, mean_y_{0}, mean_yaw_{0};

  std::deque<OdomStamped> odom_buf_;
  ros::Time last_prop_stamp_;

  // ROS
  ros::Subscriber sub_odom_, sub_cones_;
  ros::Publisher pub_particles_posearray_;
  ros::Publisher pub_landmarks_posearray_;
  ros::Publisher pub_landmarks_posearray_confirmed_;
  ros::Publisher pub_landmarks_posearray_locked_;
  ros::Publisher pub_unmatched_posearray_;
  ros::Publisher pub_slam_odom_, pub_slam_odom_mean_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_{tfbuf_};

  std::mt19937 rng_{std::random_device{}()};

  // === Methods ===
  void initializeParticlesFrom(double x, double y, double yaw,
                               double sx, double sy, double syaw);

  // Odom
  void cbOdom(const nav_msgs::Odometry::ConstPtr& msg);
  OdomStamped interpTwist(const OdomStamped& a,
                          const OdomStamped& b,
                          const ros::Time& s) const;
  void propagateParticlesTo(const ros::Time& t);

  // Cones
  void cbCones(const qcar_visnav::ConeArray::ConstPtr& msg);
  bool lookupLidarExtrinsics(const ros::Time& t, LidarExtrinsics& ex) const;

  // Core SLAM step
  void processMeasurementsAt(const ros::Time& t,
                             const std::vector<MeasRB>& meas_vec,
                             const LidarExtrinsics& ex);

  // Measurement likelihood (log p(z | x, lm)) — optional
  double measLogLikelihood(const Landmark& lm,
                           const Particle& p,
                           const MeasRB& z,
                           const LidarExtrinsics& ex) const;

  // Output
  void publishCoreOutputs(const ros::Time& t,
                          const std::vector<Eigen::Vector2d>& unmatched_world);
};

}} // namespace
