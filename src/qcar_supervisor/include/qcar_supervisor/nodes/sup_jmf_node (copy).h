#ifndef QCAR_SUP_JMF_NODE_H
#define QCAR_SUP_JMF_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#include "qcar_supervisor/JMF/JMF.h"
#include "qcar_visnav/estimation/model/kinematic_model.h"

namespace qcar_supervisor
{

class SupJmfNode
{
public:
  SupJmfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~SupJmfNode() = default;

private:
  // ── Callbacks ───────────────────────────────────────────────────────
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void noisyPosCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void velCallback(const nav_msgs::Odometry::ConstPtr& msg);            // /qcar/ekf/odom
  void timerPublishCallback(const ros::TimerEvent& ev);

  // ── Core publishing ──────────────────────────────────────────────────
  void publish();

  // ── ROS handles ──────────────────────────────────────────────────────
  ros::NodeHandle        nh_, pnh_;
  ros::Subscriber        imu_sub_;
  ros::Subscriber        pose_sub_;
  ros::Subscriber        joint_sub_;
  ros::Subscriber        noisy_pos_sub_;
  ros::Subscriber        vel_sub_;
  ros::Publisher         fused_pub_;
  ros::Publisher         modes_pub_;
  ros::Publisher         mode_pub_;
  ros::Timer             publish_timer_;

  // ── Timing & intermediate state ─────────────────────────────────────
  ros::Time              last_time_;
  double                 delta_meas_{0.0};
  double                 omega_{0.0};
  double                 publish_rate_{50.0};

  // ── JMF filter & parameters ─────────────────────────────────────────
  JumpMarkovFilter       jmf_;
  KinematicModel::ModelParams params_;
  Eigen::MatrixXd        Pi_;
};

}  // namespace qcar_supervisor

#endif  // QCAR_SUP_JMF_NODE_H
