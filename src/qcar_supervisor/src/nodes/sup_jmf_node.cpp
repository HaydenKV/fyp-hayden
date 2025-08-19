#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
# include <cmath>

#include "qcar_supervisor/JMF/JMF.h"
#include "qcar_supervisor/model/kinematic_model.h"

using qcar_supervisor::JumpMarkovFilter;
using qcar_supervisor::KinematicModel;
using ModelInput  = KinematicModel::ModelInput;
using ModelParams = KinematicModel::ModelParams;

class JmfNode {
public:
  JmfNode(ros::NodeHandle& nh) {
    // 1. Load common model params
    nh.param("model_length", params_.L, 2.5);
    std::vector<double> q_vec;
    nh.getParam("Q", q_vec);
    for (int i = 0; i < 6; ++i) params_.q(i) = q_vec[i];

    // 2. Load mode‐transition matrix (Pi), assume NxN flattened
    std::vector<double> pi_flat;
    nh.getParam("Pi", pi_flat);
    int N = std::sqrt(pi_flat.size());
    Pi_.resize(N, N);
    for (int i=0; i<pi_flat.size(); ++i)
      Pi_(i/N, i%N) = pi_flat[i];

    // 3. Initialize Jump‐Markov filter
    jmf_.setModelParams(params_);
    //jmf_.setTransitionMatrix(Pi_);

    KinematicModel::StateVec x0 = KinematicModel::StateVec::Zero();
    KinematicModel::StateMat P0 = KinematicModel::StateMat::Identity()*0.1;
    Eigen::VectorXd mode0 = Eigen::VectorXd::Ones(N) / double(N);
    jmf_.init(x0, P0, mode0, Pi_);

    // 4. Subscribers & publishers
    imu_sub_     = nh.subscribe("imu/data_raw", 10,
                                &JmfNode::imuCallback, this);
    wheel_sub_   = nh.subscribe("wheel/odom",    10,
                                &JmfNode::wheelCallback, this);
    fused_pub_   = nh.advertise<nav_msgs::Odometry>(
                                "sup_jmf/odom", 10);
    modes_pub_   = nh.advertise<std_msgs::Float64MultiArray>(
                                "sup_jmf/mode_prob", 10);

    last_stamp_ = ros::Time::now();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double t  = msg->header.stamp.toSec();
    double dt = (msg->header.stamp - last_stamp_).toSec();
    last_stamp_ = msg->header.stamp;

    // 1. Predict step for all modes
    ModelInput u;
    u.a_meas_x = msg->linear_acceleration.x;
    u.a_meas_y = msg->linear_acceleration.y;
    u.delta    = latest_delta_;
    u.dt       = dt;
    jmf_.setInput(u);
    // after (you already computed t):
    jmf_.predict(t, dt);

    // 2. IMU‐based measurement update across modes
    auto h_imu = [&](auto const& x){
      Eigen::Vector2d h;
      h << x(0) + x(4),
           std::atan(x(2));
      return h;
    };
    auto H_imu = [&](auto const& x){
      Eigen::Matrix<double,2,6> H = Eigen::Matrix<double,2,6>::Zero();
      H(0,0)=1; H(0,4)=1;
      H(1,2)=1.0/(1 + x(2)*x(2));
      return H;
    };
    Eigen::Matrix2d R;
    R.setIdentity();
    R *= 0.2;
    Eigen::Vector2d z;
    z << msg->linear_acceleration.x,
         std::atan(latest_delta_);
    jmf_.update(z, h_imu, H_imu, R);


    publish();
  }

  void wheelCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // store steering or wheel velocity for next IMU predict
    latest_delta_ = 0.0;
  }

private:
  void publish() {
    // 1. Fused odometry
    auto xf = jmf_.fusedState();
    ros::Time now = ros::Time::now();
    nav_msgs::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(xf(2));
    odom.twist.twist.linear.x  = xf(0);
    odom.twist.twist.linear.y  = xf(1);
    odom.twist.twist.angular.z = xf(2);
    fused_pub_.publish(odom);

    // 2. Mode probabilities
    auto modes = jmf_.modeProb();
    std_msgs::Float64MultiArray msg;
    msg.data.assign(modes.data(), modes.data() + modes.size());
    modes_pub_.publish(msg);

    // 3. Broadcast TF
    static tf::TransformBroadcaster br;
    tf::Transform tf_msg;
    tf_msg.setOrigin({0,0,0});
    tf_msg.setRotation({odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w});
    br.sendTransform(
      tf::StampedTransform(tf_msg, now, "odom", "base_link")
    );
  }

  ros::Subscriber          imu_sub_, wheel_sub_;
  ros::Publisher           fused_pub_, modes_pub_;
  ros::Time                last_stamp_;
  double                   latest_delta_{0.0};

  JumpMarkovFilter         jmf_;
  ModelParams              params_;
  Eigen::MatrixXd          Pi_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sup_jmf_node");
  ros::NodeHandle nh("~");
  JmfNode node(nh);
  ros::spin();
  return 0;
}

