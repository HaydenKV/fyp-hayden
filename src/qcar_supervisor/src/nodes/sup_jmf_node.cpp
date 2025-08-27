// sup_jmf_node.cpp
// ==============================
// Jump Markov Filter supervisor node.
// Loads per‐mode parameters from its private namespace (~mode_params/*)
// and publishes fused odometry + mode info under /sup_jmf/*
// at a fixed, user‐configurable rate.
// Integrates IMU, steering/joint_states, EKF velocity, and LiDAR/noisy‐odom poses.
// ==============================

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
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
  JmfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , delta_meas_(0.0)
    , omega_(0.0)
  {
    // 0) Timing init
    last_time_ = ros::Time::now();
    ROS_INFO_STREAM("[JMF] Private namespace: " << pnh_.getNamespace());

    // 1) Load per‐mode KinematicModel::ModelParams
    static const std::array<std::string, JumpMarkovFilter::M> mode_names = {
      "healthy",
      "motor_fault",
      "steering_fault",
      "imu_fault",
      "encoder_fault"
    };
    for(int i = 0; i < JumpMarkovFilter::M; ++i){
      ros::NodeHandle mnh(pnh_, "mode_params/" + mode_names[i]);

      // 1) model length
      mnh.param("L", mode_params_[i].L, 0.258);
      ROS_INFO_STREAM("[JMF] Loaded mode " << mode_names[i]
                      << " L=" << mode_params_[i].L);

      // 2) process‐noise vector via XmlRpc
      XmlRpc::XmlRpcValue xml_q;
      if (!mnh.getParam("q", xml_q))
      {
        ROS_FATAL_STREAM("[JMF] mode_params/"
                        << mode_names[i] << "/q not found");
        ros::shutdown(); return;
      }
      if (xml_q.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          xml_q.size()     != KinematicModel::STATE_SIZE)
      {
        ROS_FATAL_STREAM("[JMF] mode_params/"
                        << mode_names[i] << "/q must have "
                        << KinematicModel::STATE_SIZE << " elements");
        ros::shutdown(); return;
      }

      for (int j = 0; j < KinematicModel::STATE_SIZE; ++j)
      {
        if      (xml_q[j].getType() == XmlRpc::XmlRpcValue::TypeDouble)
          mode_params_[i].q(j) = static_cast<double>(xml_q[j]);
        else if (xml_q[j].getType() == XmlRpc::XmlRpcValue::TypeInt)
          mode_params_[i].q(j) = static_cast<int>(xml_q[j]);
        else
        {
          ROS_FATAL_STREAM("[JMF] mode_params/"
                          << mode_names[i] << "/q["<<j<<"] is not numeric");
          ros::shutdown(); return;
        }
      }

      ROS_INFO_STREAM("[JMF] mode_params/"<<mode_names[i]
                    <<" q = "<<mode_params_[i].q.transpose());
    }

    // 2) Load mode‐transition matrix Pi
    std::vector<double> pi_flat;
    pnh_.param("pi", pi_flat, std::vector<double>{});
    int N = std::sqrt(pi_flat.size());
    if(N*N != (int)pi_flat.size()) {
      ROS_FATAL_STREAM("[JMF] ~pi size " << pi_flat.size()
                       << " is not a perfect square");
      ros::shutdown();
      return;
    }
    Eigen::Map<const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,
                  Eigen::RowMajor>> Pi_map(pi_flat.data(), N, N);
    Pi_ = Pi_map;

    // 3) Initialize JumpMarkovFilter with per‐mode params
    jmf_.setModelParams(mode_params_);
    auto x0 = KinematicModel::StateVec::Zero();
    auto P0 = KinematicModel::StateMat::Identity() * 0.1;
    Eigen::VectorXd mode0 = Eigen::VectorXd::Ones(N) / double(N);
    jmf_.init(x0, P0, mode0, Pi_);

    // 4) Subscribers
    imu_sub_       = nh_.subscribe("/imu",           10, &JmfNode::imuCallback,    this);
    pose_sub_      = nh_.subscribe("/pose_lidar",    10, &JmfNode::poseCallback,   this);
    joint_sub_     = nh_.subscribe("/joint_states",  10, &JmfNode::jointCallback,  this);
    noisy_pos_sub_ = nh_.subscribe("/noisy_odom",    10, &JmfNode::noisyPosCallback,this);
    vel_sub_       = nh_.subscribe("/qcar/ekf/odom", 10, &JmfNode::velCallback,    this);

    // 5) Publishers
    fused_pub_ = nh_.advertise<nav_msgs::Odometry>("sup_jmf/odom",         10);
    modes_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("sup_jmf/mode_prob",10);
    mode_pub_  = nh_.advertise<std_msgs::Int32>      ("sup_jmf/current_mode",10);

    // 6) Timer for fixed‐rate publish
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
  // ── 1) IMU‐driven predict + partial update ──────────────────────────
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double dt = (msg->header.stamp - last_time_).toSec();
    if(dt <= 0.0) return;
    last_time_ = msg->header.stamp;

    ModelInput u;
    u.a_meas_x = msg->linear_acceleration.x;
    u.a_meas_y = msg->linear_acceleration.y;
    u.delta    = delta_meas_;
    u.dt       = dt;

    jmf_.setInput(u);
    jmf_.predict(msg->header.stamp.toSec(), dt);

    auto h = [&](auto const& x){
      Eigen::Vector2d zh;
      zh << x(0) + x(4),
            std::atan(x(2));
      return zh;
    };
    auto H = [&](auto const& x){
      Eigen::Matrix<double,2,6> Hm = Eigen::Matrix<double,2,6>::Zero();
      Hm(0,0)=1; Hm(0,4)=1;
      Hm(1,2)=1.0/(1.0 + x(2)*x(2));
      return Hm;
    };

    Eigen::Matrix2d R = Eigen::Matrix2d::Identity()*0.2;
    Eigen::Vector2d z;
    z << msg->linear_acceleration.x,
         std::atan(delta_meas_);

    jmf_.update(z, h, H, R);
  }

  // ── 2) LiDAR‐pose update ────────────────────────────────────────────
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    Eigen::Vector2d p;
    p << msg->pose.pose.position.x,
         msg->pose.pose.position.y;
    Eigen::Matrix2d Rpos = Eigen::Matrix2d::Identity()*0.0025;
    jmf_.updateWithPosition(p, Rpos);
  }

  // ── 3) Noisy‐odom fallback ──────────────────────────────────────────
  void noisyPosCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Vector2d p;
    p << msg->pose.pose.position.x,
         msg->pose.pose.position.y;
    Eigen::Matrix2d Rpos = Eigen::Matrix2d::Identity()*0.0025;
    jmf_.updateWithPosition(p, Rpos);
  }

  // ── 4) EKF‐odom velocity update ────────────────────────────────────
  void velCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Vector2d z;
    z << msg->twist.twist.linear.x,
         msg->twist.twist.linear.y;
    Eigen::Matrix2d Rvel = Eigen::Matrix2d::Identity()*0.05;

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

  // ── 5) Steering & wheel‐rate input ─────────────────────────────────
  void jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    double sum=0; int cnt=0;
    for(size_t i=0;i<msg->name.size();++i){
      if(msg->name[i]=="steering_joint") delta_meas_=msg->position[i];
      else if(msg->name[i]=="left_wheel_joint" || msg->name[i]=="right_wheel_joint"){
        sum += msg->velocity[i]; ++cnt;
      }
    }
    if(cnt>0) omega_ = sum/cnt;
  }

  // ── Timer‐driven publish ────────────────────────────────────────────
  void timerPublishCallback(const ros::TimerEvent&) {
    publish();
  }

  // ── 6) Publish fused odom, modes & tf ───────────────────────────────
  void publish() {
    auto xf = jmf_.fusedState();
    ros::Time now = ros::Time::now();

    nav_msgs::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(xf(2));
    odom.twist.twist.linear.x  = xf(0);
    odom.twist.twist.linear.y  = xf(1);
    odom.twist.twist.angular.z = xf(2);
    fused_pub_.publish(odom);

    auto modes = jmf_.modeProb();
    std_msgs::Float64MultiArray mm; mm.data.assign(modes.data(), modes.data()+modes.size());
    modes_pub_.publish(mm);

    int best = std::distance(modes.data(),
                             std::max_element(modes.data(), modes.data()+modes.size()));
    std_msgs::Int32 mi; mi.data = best;
    mode_pub_.publish(mi);

    static tf::TransformBroadcaster br;
    tf::Transform tfm;
    tfm.setOrigin({xf(0), xf(1), 0.0});
    tfm.setRotation({odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z,
                     odom.pose.pose.orientation.w});
    br.sendTransform(tf::StampedTransform(tfm, now, "odom", "base_link"));
  }

  // ── Members ─────────────────────────────────────────────────────────
  ros::NodeHandle                               nh_, pnh_;
  ros::Subscriber                               imu_sub_, pose_sub_, joint_sub_,
                                                noisy_pos_sub_, vel_sub_;
  ros::Publisher                                fused_pub_, modes_pub_, mode_pub_;
  ros::Timer                                    publish_timer_;

  ros::Time                                     last_time_;
  double                                        delta_meas_, omega_, publish_rate_;

  JumpMarkovFilter                              jmf_;
  std::array<ModelParams, JumpMarkovFilter::M>  mode_params_;
  Eigen::MatrixXd                               Pi_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sup_jmf_node");
  ros::NodeHandle nh, pnh("~");
  JmfNode node(nh, pnh);
  ros::spin();
  return 0;
}
