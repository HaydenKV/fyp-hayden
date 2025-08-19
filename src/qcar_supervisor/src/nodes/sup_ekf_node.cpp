#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "qcar_supervisor/ekf.h"
#include "qcar_supervisor/model/kinematic_model.h"

using qcar_supervisor::ExtendedKalmanFilter;
using qcar_supervisor::KinematicModel;
using ModelInput  = KinematicModel::ModelInput;
using ModelParams = KinematicModel::ModelParams;


class EkfNode {
public:
  EkfNode(ros::NodeHandle& nh) {
    ExtendedKalmanFilter ekf_;
    ModelParams          params_;
    Eigen::Matrix2d      R_imu_;
    // 1. Load parameters from ROS parameter server
    nh.param("model_length", params_.L, 2.5);
    std::vector<double> q_vec;
    std::vector<double> r_imu_vec;
    nh.getParam("Q", q_vec);       // size 6
    nh.getParam("R_imu", r_imu_vec); // size 2

    for (int i = 0; i < 6; ++i)      params_.q(i)  = q_vec[i];
    R_imu_.setZero();
    R_imu_(0,0) = r_imu_vec[0];
    R_imu_(1,1) = r_imu_vec[1];

    ekf_.setModelParams(params_);

    // 2. Initialize filter state & covariance
    KinematicModel::StateVec x0 = KinematicModel::StateVec::Zero();
    KinematicModel::StateMat P0 = 
      KinematicModel::StateMat::Identity() * 0.1;
    ekf_.init(x0, P0);

    // 3. Subscribers & Publishers
    imu_sub_  = nh.subscribe("imu/data_raw",  10,
                             &EkfNode::imuCallback, this);
    odom_sub_ = nh.subscribe("wheel/odom",   10,
                             &EkfNode::odomCallback, this);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(
                             "sup_ekf/odom", 10);

    last_stamp_ = ros::Time::now();
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    double t       = msg->header.stamp.toSec();
    double dt      = (msg->header.stamp - last_stamp_).toSec();
    last_stamp_    = msg->header.stamp;

    // Set control input from IMU accel + (if available) steering in odomCallback
    ModelInput u;
    u.a_meas_x = msg->linear_acceleration.x;
    u.a_meas_y = msg->linear_acceleration.y;
    u.delta    = latest_delta_;
    u.dt       = dt;
    ekf_.setInput(u);

    // Predict
    ekf_.predict(t, dt);

    // Measurement update using IMU: h = [vx + bax; atan(r)]
    auto x = ekf_.state(); // grab current state
    Eigen::Vector2d z;
    z << msg->linear_acceleration.x + x(4),  // or whatever your measurement really is
         std::atan(x(2));
    ekf_.update(z, 
                [&](auto const& x){
                  Eigen::Vector2d h;
                  h << x(0) + x(4), std::atan(x(2));
                  return h;
                },
                [&](auto const& x){
                  Eigen::Matrix<double,2,6> H = Eigen::Matrix<double,2,6>::Zero();
                  H(0,0)=1; H(0,4)=1;
                  H(1,2)=1.0/(1 + x(2)*x(2));
                  return H;
                },
                R_imu_);


    publishOdom(msg->header.stamp);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract steering angle from your odometry message if encoded,
    // or simply store delta for the next predict step.
    latest_delta_ = 0.0;  // e.g. msg->twist.twist.angular.z mapped to steering
  }

private:
  void publishOdom(const ros::Time& stamp) {
    auto x = ekf_.state();
    nav_msgs::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(x(2));

    odom.twist.twist.linear.x  = x(0);
    odom.twist.twist.linear.y  = x(1);
    odom.twist.twist.angular.z = x(2);

    odom_pub_.publish(odom);

    // broadcast TF
    static tf::TransformBroadcaster br;
    tf::Transform tf_msg;
    tf_msg.setOrigin({0, 0, 0});
    tf_msg.setRotation({odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w});
    br.sendTransform(
      tf::StampedTransform(tf_msg, stamp, "odom", "base_link")
    );
  }

  ros::Subscriber         imu_sub_, odom_sub_;
  ros::Publisher          odom_pub_;
  ros::Time               last_stamp_;
  double                  latest_delta_{0.0};

  ExtendedKalmanFilter    ekf_;
  ModelParams             params_;
  Eigen::Matrix2d         R_imu_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "sup_ekf_node");
  ros::NodeHandle nh("~");
  EkfNode node(nh);
  ros::spin();
  return 0;
}

