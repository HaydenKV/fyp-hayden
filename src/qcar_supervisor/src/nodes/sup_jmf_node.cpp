#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_supervisor/JMF.h"
#include "qcar_supervisor/transition_model.h"
#include "qcar_supervisor/measurement_accel/measurement_accel.h"


using namespace qcar_nav;

class JmfNode {
public:
  JmfNode(ros::NodeHandle& nh)
    : nh_(nh), filter_(8) { // Max 8 components
    imu_sub_ = nh_.subscribe("/imu", 10, &JmfNode::imuCallback, this);
    state_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/jmf/state", 10);
    mode_pub_  = nh_.advertise<std_msgs::Float64MultiArray>("/jmf/mode_probs", 10);

    setupTransitions();
    initializeFilter();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber imu_sub_;
  ros::Publisher state_pub_, mode_pub_;

  JumpMarkovFilter filter_;
  TransitionModel T_acc_;

  double last_time_ = -1.0;
  KinematicModel::ModelInput input_;
  KinematicModel::StateVec dxdt_;
  KinematicModel::StateMat ddxdtdx_;

  void setupTransitions() {
    Eigen::Matrix2d T_ref;
    T_ref << 0.90, 0.20,
             0.10, 0.80;
    T_acc_.setReferenceMatrix(T_ref, 1.0);  // 1.0s reference interval
  }

  void initializeFilter() {
    JmfComponent init;
    init.mean.setZero();
    init.cov = KinematicModel::StateMat::Identity() * 0.1;
    init.status.acc = ACC_HEALTHY;  // Healthy accelerometer
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
    input_.dt       = dt;

    // ROS_INFO_STREAM_THROTTLE(1.0,
    //   "[JMF Node] t=" << t
    //   << " dt=" << dt
    //   << " a_meas=(" << input_.a_meas_x << ", " << input_.a_meas_y << ")");

    // Placeholder dynamics (can be replaced with actual model)
    dxdt_.setZero();
    ddxdtdx_.setZero();

    filter_.step(dt, input_, dxdt_, ddxdtdx_, 9.81, T_acc_);

    // // Inspect marginals
    // const auto acc_probs = filter_.getAccStatusMarginals();
    // if (acc_probs.size() == 2) {
    //   ROS_INFO_STREAM_THROTTLE(0.5,
    //     "[JMF Node] acc marginals: healthy=" << acc_probs[0]
    //     << " faulty=" << acc_probs[1]);
    // }

    publishState(t);
    publishModes();
  }

  void publishState(double t) {
    const auto x = filter_.getMAPEstimate();
    geometry_msgs::TwistStamped out;
    out.header.stamp    = ros::Time(t);
    out.header.frame_id = "base_link";
    out.twist.linear.x  = x(0);  // vx
    out.twist.linear.y  = x(1);  // vy
    out.twist.angular.z = x(2);  // r
    state_pub_.publish(out);
  }

  void publishModes() {
    std_msgs::Float64MultiArray msg;
    const auto probs = filter_.getAccStatusMarginals();  // we’ll make this return [H,F]
    msg.data.clear();
    msg.data.push_back(probs[ACC_HEALTHY]);  // index 0
    msg.data.push_back(probs[ACC_FAULTY]);   // index 1
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
