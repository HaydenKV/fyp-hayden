// sup_jmf_node.cpp
// Jump Markov Filter node without any position updates
// Only IMU-driven and velocity (EKF-odom) updates remain.

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>
#include "qcar_supervisor/JMF/JMF.h"
#include "qcar_visnav/estimation/model/kinematic_model.h"

using qcar_supervisor::JumpMarkovFilter;
using qcar_supervisor::KinematicModel;
using ModelInput  = KinematicModel::ModelInput;
using ModelParams = KinematicModel::ModelParams;

class JmfNode {
public:
  // wheel‐encoder subscriptions
  ros::Subscriber wheel_fl_sub_, wheel_fr_sub_,
                   wheel_rl_sub_, wheel_rr_sub_;
  // store most recent wheel velocities
  float             wheel_fl_vel_{0},
                    wheel_fr_vel_{0},
                    wheel_rl_vel_{0},
                    wheel_rr_vel_{0};
  JmfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , delta_meas_(0.0)
    , omega_(0.0)
  {
    // 0) Timing init
    last_time_ = ros::Time::now();
    ROS_INFO_STREAM("[JMF] Private namespace: " << pnh_.getNamespace());

    // allocate per‐mode R arrays
    R_imu_modes_.resize(JumpMarkovFilter::M);
    R_vel_modes_.resize(JumpMarkovFilter::M);

    // 1) Load per‐mode ModelParams and R matrices
    static const std::array<std::string, JumpMarkovFilter::M> mode_names = {
      "healthy",
      "motor_fault",
      "steering_fault",
      "imu_fault",
      "encoder_fault"
    };

    for (int i = 0; i < JumpMarkovFilter::M; ++i) {
      ros::NodeHandle mnh(pnh_, "mode_params/" + mode_names[i]);

      // 1a) model length
      mnh.param("L", mode_params_[i].L, 0.258);
      ROS_INFO_STREAM("[JMF] Loaded mode " << mode_names[i]
                      << " L=" << mode_params_[i].L);

      // 1b) process‐noise vector
      std::vector<double> q_vec;
      mnh.getParam("q", q_vec);
      mode_params_[i].q = Eigen::Map<
        const Eigen::Matrix<double, KinematicModel::STATE_SIZE, 1>
      >(q_vec.data());
      ROS_INFO_STREAM("[JMF] mode_params/" << mode_names[i]
                      << " q = " << mode_params_[i].q.transpose());

      // 1c) measurement noise covariances (IMU)
      std::vector<double> buf;
      if (!mnh.getParam("R_imu", buf) || buf.size() != 4) {
        ROS_WARN_STREAM("[JMF] mode "<<mode_names[i]<<": R_imu missing or wrong size; using default 2x2 diag 0.01");
        R_imu_modes_[i] = Eigen::Matrix2d::Identity() * 0.01;
      } else {
        R_imu_modes_[i] = Eigen::Map<const Eigen::Matrix<double,2,2,Eigen::RowMajor>>(buf.data());
      }
      // symmetrize + SPD guard
      R_imu_modes_[i] = 0.5 * (R_imu_modes_[i] + R_imu_modes_[i].transpose());
      R_imu_modes_[i].diagonal().array() += 1e-8;

      // 1d) measurement noise covariances (velocity)
      if (!mnh.getParam("R_vel", buf) || buf.size() != 4) {
        ROS_WARN_STREAM("[JMF] mode "<<mode_names[i]<<": R_vel missing or wrong size; using default 2x2 diag 0.01");
        R_vel_modes_[i] = Eigen::Matrix2d::Identity() * 0.01;
      } else {
        R_vel_modes_[i] = Eigen::Map<const Eigen::Matrix<double,2,2,Eigen::RowMajor>>(buf.data());
      }
      // symmetrize + SPD guard
      R_vel_modes_[i] = 0.5 * (R_vel_modes_[i] + R_vel_modes_[i].transpose());
      R_vel_modes_[i].diagonal().array() += 1e-8;

      ROS_INFO_STREAM("[JMF] mode_params/"<<mode_names[i]<<" R_imu =\n"<<R_imu_modes_[i]);
      ROS_INFO_STREAM("[JMF] mode_params/"<<mode_names[i]<<" R_vel =\n"<<R_vel_modes_[i]);
    }

    // 2) Load mode‐transition matrix Pi
    std::vector<double> pi_flat;
    pnh_.param("pi", pi_flat, std::vector<double>{});
    int N = std::sqrt(pi_flat.size());
    if (N*N != (int)pi_flat.size()) {
      ROS_FATAL_STREAM("[JMF] pi size " << pi_flat.size()
                       << " is not a perfect square");
      ros::shutdown();
      return;
    }
    Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,
                   Eigen::RowMajor>> Pi_map(pi_flat.data(), N, N);
    Pi_ = Pi_map;

    ROS_INFO_STREAM("[JMF] Loaded Pi =\n" << Pi_);

    // 3) Initialize JumpMarkovFilter
    jmf_.setModelParams(mode_params_);
    auto x0 = KinematicModel::StateVec::Zero();
    // Initial covariance
    // auto P0 = KinematicModel::StateMat::Identity() * 0.1;
    Eigen::Matrix<double,6,6> P0;
    P0.setZero();
    P0.diagonal() << 1.0, 1.0, 0.5, 0.1, 0.1, 0.1;

    // Initial mode probabilities
    // Eigen::VectorXd mode0 = Eigen::VectorXd::Ones(N) / double(N);
    Eigen::VectorXd mode0 = Eigen::VectorXd::Zero(N);
    mode0(0) = 1.0;      // start fully in “healthy”

    jmf_.init(x0, P0, mode0, Pi_);

    // 4) Subscribers (only IMU, EKF-odom, and joints)
    imu_sub_   = nh_.subscribe("/imu",           10, 
                  &JmfNode::imuCallback,    this);
    vel_sub_   = nh_.subscribe("/qcar/ekf/odom", 10, 
                  &JmfNode::velCallback,    this);
    joint_sub_ = nh_.subscribe("/joint_states",  10, 
                  &JmfNode::jointCallback,  this);
    // 4a) add encoder subscriptions
    wheel_fl_sub_ = nh_.subscribe<std_msgs::Float32>(
                     "/wheelfl_motor/velocity", 10,
                     &JmfNode::wheelFLCallback, this);
    wheel_fr_sub_ = nh_.subscribe<std_msgs::Float32>(
                     "/wheelfr_motor/velocity", 10,
                     &JmfNode::wheelFRCallback, this);
    wheel_rl_sub_ = nh_.subscribe<std_msgs::Float32>(
                     "/wheelrl_motor/velocity", 10,
                     &JmfNode::wheelRLCallback, this);
    wheel_rr_sub_ = nh_.subscribe<std_msgs::Float32>(
                     "/wheelrr_motor/velocity", 10,
                     &JmfNode::wheelRRCallback, this);

    // 5) Publishers
    fused_pub_ = nh_.advertise<nav_msgs::Odometry>(
                  "sup_jmf/odom",         10);
    modes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
                  "sup_jmf/mode_prob",    10);
    mode_pub_  = nh_.advertise<std_msgs::Int32>(
                  "sup_jmf/current_mode", 10);

    // 6) Timer for publish
    pnh_.param("publish_rate", publish_rate_, 50.0);
    ROS_INFO_STREAM("[JMF] Publishing at " << publish_rate_ << " Hz");
    publish_timer_ = nh_.createTimer(
      ros::Duration(1.0 / publish_rate_),
      &JmfNode::timerPublishCallback,
      this
    );

    ROS_INFO("[JMF] Node initialized successfully");
  }

private:
  // Hysteresis state (moved out of constructor/callback)
  int    declared_mode_{0};
  int    stable_count_{0};
  const int   minStable_{5};   // need 5 consecutive cycles to switch
  const double Pthresh_{0.8};  // require >80% to consider a new mode
  // ── Warm-up to inflate Pₚᵣₑd before updates ─────────────────
  int    warmup_{0};
  const int WARMUP_LIMIT_{5};

  // 4b) wheel‐velocity callbacks
  void wheelFLCallback(const std_msgs::Float32::ConstPtr& m) { wheel_fl_vel_ = m->data; }
  void wheelFRCallback(const std_msgs::Float32::ConstPtr& m) { wheel_fr_vel_ = m->data; }
  void wheelRLCallback(const std_msgs::Float32::ConstPtr& m) { wheel_rl_vel_ = m->data; }
  void wheelRRCallback(const std_msgs::Float32::ConstPtr& m) { wheel_rr_vel_ = m->data; }

  // IMU‐driven predict + partial update (no position fusion)
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double dt = (msg->header.stamp - last_time_).toSec();
    if (dt <= 0.0) return;
    last_time_ = msg->header.stamp;

    // 1) Always run predict to grow Pₚᵣₑd
    ModelInput u;
    u.a_meas_x = msg->linear_acceleration.x;
    u.a_meas_y = msg->linear_acceleration.y;
    u.delta    = delta_meas_;
    u.dt       = dt;
    jmf_.setInput(u);
    jmf_.predict(msg->header.stamp.toSec(), dt);

    // 2) Skip measurement updates for WARMUP_LIMIT_ cycles
    if (++warmup_ <= WARMUP_LIMIT_) {
      ROS_DEBUG_STREAM("[JMF] Warmup " << warmup_ << "/" << WARMUP_LIMIT_);
      return;
    }

    // 3) After warmup, run your normal IMU update
    auto h = [&](auto const& x){
      Eigen::Vector2d zh;
      zh << x(0) + x(4),
            std::atan(x(2));
      return zh;
    };
    auto H = [&](auto const& x){
      Eigen::Matrix<double,2,6> M = Eigen::Matrix<double,2,6>::Zero();
      M(0,0)=1; M(0,4)=1;
      M(1,2)=1.0/(1.0 + x(2)*x(2));
      return M;
    };
    Eigen::Vector2d z{
      msg->linear_acceleration.x,
      std::atan(delta_meas_)
    };
    int best = jmf_.mostLikelyMode();
    jmf_.update(z, h, H, R_imu_modes_[best]);
  }

  // EKF‐odom velocity update
  void velCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Vector2d z;
    z << msg->twist.twist.linear.x,
         msg->twist.twist.linear.y;

    int best = jmf_.mostLikelyMode();
    const auto& Rvel = R_vel_modes_[best];

    auto h_vel = [&](auto const& x){
      Eigen::Vector2d zh;
      zh << x(0), x(1);
      return zh;
    };
    auto H_vel = [&](auto const&){
      Eigen::Matrix<double,2,6> Hm = Eigen::Matrix<double,2,6>::Zero();
      Hm(0,0)=1; Hm(1,1)=1;
      return Hm;
    };

    jmf_.update(z, h_vel, H_vel, Rvel);
  }

  // Steering & wheel‐rate input (for IMU callback’s u.delta)
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    double sum = 0; int cnt = 0;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "steering_joint")
        delta_meas_ = msg->position[i];
      else if (msg->name[i] == "left_wheel_joint" ||
               msg->name[i] == "right_wheel_joint") {
        sum += msg->velocity[i];
        ++cnt;
      }
    }
    if (cnt > 0) omega_ = sum / cnt;
  }

  // Timer‐driven publish fused odom, modes & tf
  void timerPublishCallback(const ros::TimerEvent&) {
    // 1) First, do your encoder‐based velocity update
    double v_enc = double(wheel_fl_vel_ + wheel_fr_vel_
                    + wheel_rl_vel_ + wheel_rr_vel_) / 4.0;
    Eigen::VectorXd z_enc(1); z_enc << v_enc;

    auto h_enc = [&](auto const& x){
      Eigen::VectorXd zh(1); zh << x(0); return zh;  // measure vx only
    };
    auto H_enc = [&](auto const&){
      Eigen::Matrix<double,1,6> M = Eigen::Matrix<double,1,6>::Zero();
      M(0,0) = 1.0; return M;
    };

    // Build a 1x1 R from your mode's R_vel(0,0) entry
    int best_enc = jmf_.mostLikelyMode();
    Eigen::MatrixXd R1(1,1);
    R1(0,0) = std::max(1e-6, R_vel_modes_[best_enc](0,0));

    jmf_.update(z_enc, h_enc, H_enc, R1);

    auto xf = jmf_.fusedState();
    ros::Time now = ros::Time::now();

    // Publish fused odometry
    nav_msgs::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(xf(2));
    odom.twist.twist.linear.x  = xf(0);
    odom.twist.twist.linear.y  = xf(1);
    odom.twist.twist.angular.z = xf(2);
    fused_pub_.publish(odom);

    // Publish mode probabilities
    auto modes = jmf_.modeProb();
    std_msgs::Float64MultiArray mm;
    mm.data.assign(modes.data(), modes.data()+modes.size());
    modes_pub_.publish(mm);

    auto probs = jmf_.modeProb();
    int best = jmf_.mostLikelyMode();
    double pbest = probs(best);

    if (best != declared_mode_) {
        if (pbest > Pthresh_) {
        if (++stable_count_ >= minStable_) {
            declared_mode_ = best;
            stable_count_ = 0;
        }
        } else {
        stable_count_ = 0;
        }
    } else {
        stable_count_ = 0;
    }

    std_msgs::Int32 mi;  
    mi.data = declared_mode_;  
    mode_pub_.publish(mi);


    // Broadcast base_link TF
    static tf::TransformBroadcaster br;
    tf::Transform tfm;
    tfm.setOrigin({xf(0), xf(1), 0.0});
    tfm.setRotation({
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w
    });
    br.sendTransform(tf::StampedTransform(tfm, now,
                                          "odom", "base_link"));
  }

  // Members
  ros::NodeHandle                     nh_, pnh_;
  ros::Subscriber                     imu_sub_, vel_sub_, joint_sub_;
  ros::Publisher                      fused_pub_, modes_pub_, mode_pub_;
  ros::Timer                          publish_timer_;
  ros::Time                           last_time_;
  double                              delta_meas_, omega_, publish_rate_;
  JumpMarkovFilter                    jmf_;
  std::array<ModelParams, JumpMarkovFilter::M> mode_params_;
  Eigen::MatrixXd                     Pi_;
  std::vector<Eigen::Matrix2d>        R_imu_modes_, R_vel_modes_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sup_jmf_node");
  ros::NodeHandle nh, pnh("~");
  JmfNode node(nh, pnh);
  ros::spin();
  return 0;
}
