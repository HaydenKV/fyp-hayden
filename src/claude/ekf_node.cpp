#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_visnav/estimation/ekf/ekf_core.h"
#include "qcar_visnav/estimation/sensors/gyro.h"
#include "qcar_visnav/estimation/sensors/speed.h"
#include "qcar_visnav/estimation/sensors/pseudo_measurements.h"
#include "qcar_visnav/estimation/utils/odometry_buffer.h"

namespace qcar_nav {

class EkfNode {
public:
  EkfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    // ---- Parameters from YAML ----
    pnh_.param<std::string>("imu_topic", imu_topic_, std::string("/imu"));
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, std::string("/qcar/joint_states"));
    pnh_.param<std::string>("steering_topic", steering_topic_, std::string("/steering/angle"));
    pnh_.param<std::string>("odom_topic", odom_topic_, std::string("/qcar/ekf/odom_body"));

    pnh_.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh_.param("publish_tf", publish_tf_, false);  // Usually false for velocity-only EKF

    // Sensor noise parameters
    pnh_.param("gyro_std", gyro_std_, 0.01);
    pnh_.param("speed_std", speed_std_, 0.01);

    // Vehicle parameters
    pnh_.param("rw", rw_, 0.033);  // wheel radius
    pnh_.param("wheelbase", wheelbase_, 0.258);  // QCar wheelbase
    pnh_.param("dtMaxEst", dtMaxEst_, 0.01);

    // Accelerometer handling
    pnh_.param<std::string>("gravity_mode", gravity_mode_, std::string("off"));
    pnh_.param("startup_calib_time", startup_calib_time_, 2.5);
    pnh_.param("imu_lpf_hz", imu_lpf_hz_, 15.0);

    // Initial velocities (usually zero)
    double init_vx=0.0, init_vy=0.0, init_r=0.0;
    pnh_.param("init_vx", init_vx, 0.0);
    pnh_.param("init_vy", init_vy, 0.0);
    pnh_.param("init_r", init_r, 0.0);

    // Drive wheel joint names
    {
      XmlRpc::XmlRpcValue names;
      if (pnh_.getParam("drive_wheel_joints", names) &&
          names.getType() == XmlRpc::XmlRpcValue::TypeArray && names.size() >= 1) {
        for (int i = 0; i < names.size(); ++i)
          drive_joints_.push_back(static_cast<std::string>(names[i]));
      } else {
        drive_joints_ = {"wheelfl_motor","wheelfr_motor"};
      }
    }

    // Process noise q for [vx, vy, r, bg, bax, bay]
    {
      XmlRpc::XmlRpcValue q_list;
      if (pnh_.getParam("q", q_list) &&
          q_list.getType() == XmlRpc::XmlRpcValue::TypeArray && q_list.size() == 6) {
        for (int i=0; i<6; ++i) model_params_.q(i) = static_cast<double>(q_list[i]);
      } else {
        model_params_.q << 2.0, 2.0, 1.0, 0.001, 0.01, 0.01;  // Default values
      }
    }

    // NHC parameters
    pnh_.param("nhc_enable", nhc_params_.enabled, true);
    pnh_.param("nhc_velocity_threshold", nhc_params_.velocity_threshold, 1.0);
    pnh_.param("nhc_base_std", nhc_params_.base_std, 0.05);
    pnh_.param("nhc_scaling_factor", nhc_params_.scaling_factor, 1.0);

    // ZUPT parameters  
    pnh_.param("zupt_enable", zupt_params_.enabled, true);
    pnh_.param("zupt_velocity_threshold", zupt_params_.velocity_threshold, 0.1);
    pnh_.param("zupt_gyro_threshold", zupt_params_.gyro_threshold, 0.05);
    pnh_.param("zupt_min_duration", zupt_params_.min_duration, 0.3);
    pnh_.param("zupt_velocity_std", zupt_params_.velocity_std, 0.01);
    pnh_.param("zupt_yaw_rate_std", zupt_params_.yaw_rate_std, 0.01);

    // Buffer parameters
    int buffer_size;
    pnh_.param("odom_buffer_size", buffer_size, 200);
    odom_buffer_ = std::make_unique<OdometryBuffer>(buffer_size);

    // ---- Publishers/Subscribers ----
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &EkfNode::imuCb, this);
    js_sub_ = nh_.subscribe(joint_states_topic_, 50, &EkfNode::jointStatesCb, this);
    
    if (!steering_topic_.empty()) {
      steering_sub_ = nh_.subscribe(steering_topic_, 50, &EkfNode::steeringCb, this);
    }

    // Ground truth for comparison
    ground_truth_sub_ = nh_.subscribe("/odom", 1, &EkfNode::groundTruthCb, this);

    // ---- EKF Initialization ----
    // State: [vx, vy, r, bg, bax, bay]
    SREKF::Vec mu0 = SREKF::Vec::Zero();
    mu0(0) = init_vx;
    mu0(1) = init_vy;
    mu0(2) = init_r;
    // Biases start at zero

    // Initial covariance
    const double P0_vx = 0.25;   // (0.5 m/s)^2
    const double P0_vy = 0.25;   
    const double P0_r = 0.09;    // (0.3 rad/s)^2
    const double P0_bg = 1e-2;
    const double P0_bax = 1e-2;
    const double P0_bay = 1e-2;

    SREKF::Mat S0 = SREKF::Mat::Zero();
    S0(0,0) = std::sqrt(P0_vx);
    S0(1,1) = std::sqrt(P0_vy);
    S0(2,2) = std::sqrt(P0_r);
    S0(3,3) = std::sqrt(P0_bg);
    S0(4,4) = std::sqrt(P0_bax);
    S0(5,5) = std::sqrt(P0_bay);

    ekf_.setInitial(mu0, S0);

    // Set model parameters
    model_params_.L = wheelbase_;
    model_.setParams(model_params_);

    // Runtime variables
    last_imu_stamp_ = ros::Time(0);
    current_steering_angle_ = 0.0;
    a_meas_x_lp_ = 0.0;
    a_meas_y_lp_ = 0.0;
    g_x_ = 0.0;
    g_y_ = 0.0;
    calib_done_ = (gravity_mode_ != "startup_avg");

    ROS_INFO("[EKF] Velocity-based EKF node initialized");
    ROS_INFO("[EKF] State: [vx, vy, r, bg, bax, bay] - 6 DOF");
    ROS_INFO("[EKF] Topics: IMU=%s, Joints=%s, Steering=%s, Output=%s", 
             imu_topic_.c_str(), joint_states_topic_.c_str(), 
             steering_topic_.c_str(), odom_topic_.c_str());
  }

private:
  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Publisher odom_pub_;
  ros::Subscriber imu_sub_, js_sub_, steering_sub_, ground_truth_sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Topics and frames
  std::string imu_topic_, joint_states_topic_, steering_topic_, odom_topic_;
  std::string odom_frame_, base_frame_;
  bool publish_tf_;

  // Parameters
  double gyro_std_, speed_std_;
  double rw_, wheelbase_, dtMaxEst_;
  std::string gravity_mode_;
  double startup_calib_time_, imu_lpf_hz_;
  std::vector<std::string> drive_joints_;

  // EKF components
  using SREKF = EkfCore<KinematicModel::STATE_SIZE>;
  SREKF ekf_;
  KinematicModel model_;
  KinematicModel::ModelParams model_params_;

  // Pseudo-measurement components
  NHCMeasurement::NHCParams nhc_params_;
  ZUPTMeasurement::ZUPTParams zupt_params_;
  ZUPTMeasurement::ZUPTState zupt_state_;

  // Buffer for deskewing
  std::unique_ptr<OdometryBuffer> odom_buffer_;

  // Runtime state
  ros::Time last_imu_stamp_;
  double current_steering_angle_;
  double a_meas_x_lp_, a_meas_y_lp_;  // Low-pass filtered accelerations
  double g_x_, g_y_;  // Gravity estimates
  bool calib_done_;

  // Calibration for gravity
  ros::Time calib_start_;
  double calib_sum_ax_, calib_sum_ay_;
  int calib_count_;

  // Ground truth comparison
  nav_msgs::Odometry latest_ground_truth_;
  bool has_ground_truth_ = false;

  // Callbacks
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
  void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);
  void steeringCb(const std_msgs::Float32::ConstPtr& msg);
  void groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg);

  // Utility functions
  void publishOdom(const ros::Time& stamp);
  void logStateComparison(const nav_msgs::Odometry& estimate);
};

// ===================== IMU Callback =====================
void EkfNode::imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
  const ros::Time stamp = msg->header.stamp;
  double dt = 0.0;

  if (last_imu_stamp_.isZero()) {
    last_imu_stamp_ = stamp;
    return;
  } else {
    dt = (stamp - last_imu_stamp_).toSec();
    if (dt <= 0.0 || dt > 0.1) dt = dtMaxEst_;
  }

  // Gravity calibration
  if (!calib_done_ && gravity_mode_ == "startup_avg") {
    if (calib_count_ == 0) calib_start_ = stamp;
    calib_sum_ax_ += msg->linear_acceleration.x;
    calib_sum_ay_ += msg->linear_acceleration.y;
    ++calib_count_;
    const double elapsed = (stamp - calib_start_).toSec();
    if (elapsed >= startup_calib_time_) {
      g_x_ = (calib_count_>0) ? (calib_sum_ax_ / calib_count_) : 0.0;
      g_y_ = (calib_count_>0) ? (calib_sum_ay_ / calib_count_) : 0.0;
      calib_done_ = true;
      ROS_INFO("[EKF] Gravity calibrated: gx=%.5f, gy=%.5f m/s^2", g_x_, g_y_);
    }
  }

  // Low-pass filter accelerations
  const double wc = 2.0 * M_PI * std::max(1e-3, imu_lpf_hz_);
  const double alpha = std::exp(-wc * dt);
  const double ax_raw = msg->linear_acceleration.x;
  const double ay_raw = msg->linear_acceleration.y;
  a_meas_x_lp_ = alpha * a_meas_x_lp_ + (1.0 - alpha) * ax_raw;
  a_meas_y_lp_ = alpha * a_meas_y_lp_ + (1.0 - alpha) * ay_raw;

  // Prepare model input (gravity-corrected accelerations)
  KinematicModel::ModelInput u;
  u.a_meas_x = a_meas_x_lp_ - g_x_;
  u.a_meas_y = a_meas_y_lp_ - g_y_;
  u.delta = current_steering_angle_;
  u.dt = dt;

  if (gravity_mode_ == "off") {
    u.a_meas_x = 0.0;
    u.a_meas_y = 0.0;
  }
  model_.setInput(u);

  // EKF Predict step
  ekf_.predict(model_, stamp.toSec(), dt);

  // Gyro measurement update
  {
    GyroMeas gyro;
    GyroMeas::ZVec z, h;
    GyroMeas::HVec H;
    gyro.predict(ekf_.mu(), model_params_, h, H);

    double wz = msg->angular_velocity.z;
    z(0,0) = wz;

    Eigen::Matrix<double,1,1> sqrtR;
    sqrtR(0,0) = gyro_std_;
    ekf_.update(z, h, H, sqrtR);
  }

  // NHC pseudo-measurement
  if (NHCMeasurement::shouldApply(ekf_.mu(), nhc_params_)) {
    NHCMeasurement nhc;
    NHCMeasurement::ZVec z_nhc, h_nhc;
    NHCMeasurement::HVec H_nhc;
    nhc.predict(ekf_.mu(), model_params_, h_nhc, H_nhc);

    z_nhc(0) = 0.0;  // Assume vy = 0
    double nhc_std = NHCMeasurement::getMeasurementStd(ekf_.mu(), nhc_params_);
    Eigen::Matrix<double,1,1> sqrtR_nhc;
    sqrtR_nhc(0,0) = nhc_std;
    ekf_.update(z_nhc, h_nhc, H_nhc, sqrtR_nhc);
  }

  // Update ZUPT state
  double current_speed = std::abs(ekf_.mu()(0));  // |vx|
  double current_gyro = std::abs(msg->angular_velocity.z);
  ZUPTMeasurement::updateState(current_speed, current_gyro, stamp, zupt_params_, zupt_state_);

  // ZUPT pseudo-measurement
  if (ZUPTMeasurement::shouldApply(zupt_state_, zupt_params_)) {
    ZUPTMeasurement zupt;
    ZUPTMeasurement::ZVec z_zupt, h_zupt;
    ZUPTMeasurement::HVec H_zupt;
    zupt.predict(ekf_.mu(), model_params_, h_zupt, H_zupt);

    z_zupt.setZero();  // [0, 0, 0] for [vx, vy, r]
    auto R_zupt = ZUPTMeasurement::getMeasurementCovariance(zupt_params_);
    Eigen::Matrix<double,3,3> sqrtR_zupt = R_zupt.llt().matrixL();
    ekf_.update(z_zupt, h_zupt, H_zupt, sqrtR_zupt);
  }

  // Add to odometry buffer (for deskewing)
  nav_msgs::Odometry odom_msg;
  publishOdom(stamp, &odom_msg);
  odom_buffer_->addEntry(odom_msg);

  last_imu_stamp_ = stamp;
}

// ===================== Joint States Callback =====================
void EkfNode::jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
  double w_sum_abs = 0.0;
  int cnt = 0;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& j = msg->name[i];
    if (std::find(drive_joints_.begin(), drive_joints_.end(), j) != drive_joints_.end()) {
      if (i < msg->velocity.size()) {
        w_sum_abs += std::abs(msg->velocity[i]);
        ++cnt;
      }
    }
  }
  
  if (cnt < 1) return;

  const double w_avg = w_sum_abs / static_cast<double>(cnt);
  const double v_meas = w_avg * rw_;  // Convert to linear velocity

  // Speed measurement update (vx)
  SpeedMeas spd;
  SpeedMeas::ZVec z, h;
  SpeedMeas::HVec H;
  spd.predict(ekf_.mu(), model_params_, h, H);
  z(0,0) = v_meas;

  Eigen::Matrix<double,1,1> sqrtR;
  sqrtR(0,0) = speed_std_;
  ekf_.update(z, h, H, sqrtR);
}

// ===================== Steering Callback =====================
void EkfNode::steeringCb(const std_msgs::Float32::ConstPtr& msg) {
  current_steering_angle_ = msg->data;
}

// ===================== Ground Truth Callback =====================
void EkfNode::groundTruthCb(const nav_msgs::Odometry::ConstPtr& msg) {
  latest_ground_truth_ = *msg;
  has_ground_truth_ = true;
}

// ===================== Publish Odometry =====================
void EkfNode::publishOdom(const ros::Time& stamp, nav_msgs::Odometry* odom_msg_out) {
  const auto& x = ekf_.mu();
  const double vx=x(0), vy=x(1), r=x(2);

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;

  // No position in velocity-only EKF
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Velocities in body frame
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = r;

  // Covariance (velocities only)
  Eigen::Matrix<double,6,6> P = ekf_.S().transpose() * ekf_.S();
  odom.twist.covariance[0] = P(0,0);   // vx variance
  odom.twist.covariance[7] = P(1,1);   // vy variance
  odom.twist.covariance[35] = P(2,2);  // r variance
  // Cross-correlations
  odom.twist.covariance[1] = odom.twist.covariance[6] = P(0,1);   // vx-vy
  odom.twist.covariance[5] = odom.twist.covariance[30] = P(0,2);  // vx-r
  odom.twist.covariance[11] = odom.twist.covariance[31] = P(1,2); // vy-r

  odom_pub_.publish(odom);
  logStateComparison(odom);

  if (odom_msg_out) {
    *odom_msg_out = odom;
  }
}

// ===================== State Comparison Logging =====================
void EkfNode::logStateComparison(const nav_msgs::Odometry& estimate) {
  if (!has_ground_truth_) return;

  // Extract true velocities (in body frame if available, otherwise convert)
  double true_vx = latest_ground_truth_.twist.twist.linear.x;
  double true_vy = latest_ground_truth_.twist.twist.linear.y;
  double true_r = latest_ground_truth_.twist.twist.angular.z;

  // Extract estimated velocities
  double est_vx = estimate.twist.twist.linear.x;
  double est_vy = estimate.twist.twist.linear.y;
  double est_r = estimate.twist.twist.angular.z;

  // Extract biases
  const auto& ekf_state = ekf_.mu();
  double est_bg = ekf_state(3);
  double est_bax = ekf_state(4);
  double est_bay = ekf_state(5);

  // Calculate errors
  double err_vx = est_vx - true_vx;
  double err_vy = est_vy - true_vy;
  double err_r = est_r - true_r;
  double err_v_mag = std::sqrt(err_vx*err_vx + err_vy*err_vy);

  // Covariance trace
  Eigen::Matrix<double,6,6> P = ekf_.S().transpose() * ekf_.S();
  double cov_trace = P.diagonal().sum();

  ROS_INFO_THROTTLE(1.0, 
    "[EKF] TRUE: vx=%.2f vy=%.2f r=%.2f | EST: vx=%.2f vy=%.2f r=%.2f | ERR: v_mag=%.3f r=%.2f° | bias: bg=%.3f bax=%.3f bay=%.3f | cov_tr=%.3f",
    true_vx, true_vy, true_r, est_vx, est_vy, est_r,
    err_v_mag, err_r*180/M_PI, est_bg, est_bax, est_bay, cov_trace);
}

} // namespace qcar_nav

// ===================== Main Function =====================
int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_ekf_node");
  ros::NodeHandle nh, pnh("~");
  
  qcar_nav::EkfNode ekf_node(nh, pnh);
  
  ROS_INFO("[EKF] Velocity EKF node started");
  ros::spin();
  
  return 0;
}