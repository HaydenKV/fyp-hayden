#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_supervisor/JMF.h"
#include "qcar_supervisor/transition_model.h"

using namespace qcar_nav;

class JmfNode {
public:
  JmfNode(ros::NodeHandle& nh)
    : nh_(nh), filter_(8) { // Max 8 components
    imu_sub_ = nh_.subscribe("/imu", 10, &JmfNode::imuCallback, this);
    vel_sub_ = nh_.subscribe("/qcar/ekf/odom", 10, &JmfNode::velCallback, this);
    state_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/jmf/state", 10);
    mode_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/jmf/mode_probs", 10);

    setupTransitions();
    initializeFilter();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_, vel_sub_;
  ros::Publisher state_pub_, mode_pub_;

  JumpMarkovFilter filter_;
  TransitionModel T_acc_, T_vel_;

  double last_time_ = -1.0;
  double vel_meas_ = 0.0;
  double gyro_meas_ = 0.0;
  KinematicModel::ModelInput input_;
  KinematicModel::StateVec dxdt_;
  KinematicModel::StateMat ddxdtdx_;

  void setupTransitions() {
    Eigen::Matrix2d T_ref;
    T_ref << 0.99, 0.01,
             0.05, 0.95;
    T_acc_.setReferenceMatrix(T_ref, 1.0);
    T_vel_.setReferenceMatrix(T_ref, 1.0);
  }

  void initializeFilter() {
    JmfComponent init;
    init.mean.setZero();
    init.cov = KinematicModel::StateMat::Identity() * 0.1;
    init.status = {1, 1}; // Healthy
    init.log_weight = std::log(1.0);
    filter_.initialize({init});
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double t = msg->header.stamp.toSec();
    if (last_time_ < 0.0) {
      last_time_ = t;
      return;
    }

    double dt = t - last_time_;
    last_time_ = t;

    input_.a_meas_x = msg->linear_acceleration.x;
    input_.a_meas_y = msg->linear_acceleration.y;
    input_.dt = dt;
    gyro_meas_ = msg->angular_velocity.z;

    // Dummy dynamics for now (can be replaced with actual model)
    dxdt_.setZero();
    ddxdtdx_.setZero();

    filter_.step(dt, input_, dxdt_, ddxdtdx_, 9.81, vel_meas_, gyro_meas_, T_acc_, T_vel_);

    publishState(t);
    publishModes();
  }

  void velCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    vel_meas_ = msg->twist.twist.linear.x;
  }

  void publishState(double t) {
    KinematicModel::StateVec x = filter_.getMAPEstimate();
    geometry_msgs::TwistStamped out;
    out.header.stamp = ros::Time(t);
    out.header.frame_id = "base_link";
    out.twist.linear.x = x(0); // vx
    out.twist.linear.y = x(1); // vy
    out.twist.angular.z = x(2); // r
    state_pub_.publish(out);
  }

  void publishModes() {
    std_msgs::Float64MultiArray msg;
    auto acc_probs = filter_.getAccStatusMarginals();
    auto vel_probs = filter_.getVelStatusMarginals();
    msg.data.insert(msg.data.end(), acc_probs.begin(), acc_probs.end());
    msg.data.insert(msg.data.end(), vel_probs.begin(), vel_probs.end());
    mode_pub_.publish(msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sup_jmf_node");
  ros::NodeHandle nh;
  JmfNode node(nh);
  ros::spin();
  return 0;
}
