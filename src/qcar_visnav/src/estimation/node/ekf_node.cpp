#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
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

namespace qcar_nav {

class EkfNode {
public:
  EkfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
  {
    // ---- params (all from YAML) ----
    pnh_.param<std::string>("imu_topic",          imu_topic_,          std::string("/imu"));
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, std::string("/qcar/joint_states"));
    pnh_.param<std::string>("odom_topic",         odom_topic_,         std::string("/qcar/ekf/odom"));

    pnh_.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    pnh_.param<std::string>("base_frame", base_frame_, std::string("base_link"));
    pnh_.param("publish_tf", publish_tf_, true);

    pnh_.param("gyro_std",  gyro_std_,  0.05);   // rad/s
    pnh_.param("speed_std", speed_std_, 0.10);   // m/s

    pnh_.param("rw", rw_, 0.033);                // wheel radius [m]
    pnh_.param("dtMaxEst", dtMaxEst_, 0.01);     // dt clamp [s]

    pnh_.param<std::string>("gravity_mode", gravity_mode_, std::string("startup_avg"));
    pnh_.param("startup_calib_time", startup_calib_time_, 2.5);
    pnh_.param("imu_lpf_hz", imu_lpf_hz_, 15.0);

    // initial pose
    double init_x=0.0, init_y=0.0, init_yaw=0.0;
    pnh_.param("init_x", init_x, 0.0);
    pnh_.param("init_y", init_y, 0.0);
    pnh_.param("init_yaw", init_yaw, 0.0);

    // drive wheel joint names
    {
      XmlRpc::XmlRpcValue names;
      if (pnh_.getParam("drive_wheel_joints", names) &&
          names.getType() == XmlRpc::XmlRpcValue::TypeArray && names.size() >= 1) {
        for (int i = 0; i < names.size(); ++i)
          drive_joints_.push_back(static_cast<std::string>(names[i]));
      } else {
        // sensible default for your model
        drive_joints_ = {"wheelfl_motor","wheelfr_motor"};
      }
    }

    // process noise q for [X,Y,psi,v,r,bg,ba] (continuous-time variances)
    {
      XmlRpc::XmlRpcValue q_list;
      if (pnh_.getParam("q", q_list) &&
          q_list.getType() == XmlRpc::XmlRpcValue::TypeArray && q_list.size() == 7) {
        for (int i=0; i<7; ++i) model_params_.q(i) = static_cast<double>(q_list[i]);
      } else {
        model_params_.q << 1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0;
      }
    }

    // ---- pubs/subs ----
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    imu_sub_  = nh_.subscribe(imu_topic_, 100, &EkfNode::imuCb, this);
    js_sub_   = nh_.subscribe(joint_states_topic_, 50, &EkfNode::jointStatesCb, this);

    // ---- EKF init (from YAML) ----
    SREKF::Vec mu0 = SREKF::Vec::Zero();
    mu0(0) = init_x;
    mu0(1) = init_y;
    mu0(2) = init_yaw;

    // Choose desired initial *P* on each state, then set S0 = chol(P0):
    // P0 diag target: [x,   y,   psi,  v,    r,    bg,    ba]
    const double P0_x   = 1e-2;
    const double P0_y   = 1e-2;
    const double P0_psi = 1e-2;
    const double P0_v   = 0.25;   // <-- big enough so wheels can pull v
    const double P0_r   = 0.09;   // e.g. (0.3 rad/s)^2
    const double P0_bg  = 1e-2;
    const double P0_ba  = 1e-2;

    SREKF::Mat S0 = SREKF::Mat::Zero();
    S0(0,0) = std::sqrt(P0_x);
    S0(1,1) = std::sqrt(P0_y);
    S0(2,2) = std::sqrt(P0_psi);
    S0(3,3) = std::sqrt(P0_v);
    S0(4,4) = std::sqrt(P0_r);
    S0(5,5) = std::sqrt(P0_bg);
    S0(6,6) = std::sqrt(P0_ba);

    // IMPORTANT: call setInitial exactly once.
    ekf_.setInitial(mu0, S0);

    // Then params for the dynamics/noise
    model_.setParams(model_params_);


    // runtime vars
    last_imu_stamp_ = ros::Time(0);
    a_meas_x_lp_ = 0.0;
    g_x_ = 0.0;
    calib_done_ = (gravity_mode_ != "startup_avg");

    ROS_INFO("[EKF] imu=%s  joints=%s  odom_out=%s  frames=%s->%s  TF=%s  init=(%.3f,%.3f,%.3f)  wheels=%zu",
             imu_topic_.c_str(), joint_states_topic_.c_str(), odom_topic_.c_str(),
             odom_frame_.c_str(), base_frame_.c_str(), publish_tf_ ? "on":"off",
             init_x, init_y, init_yaw, drive_joints_.size());
  }

private:
  // ===================== IMU =====================
  void imuCb(const sensor_msgs::Imu::ConstPtr& msg)
  {
    const ros::Time stamp = msg->header.stamp;
    double dt = 0.0;

    if (last_imu_stamp_.isZero()) {
      last_imu_stamp_ = stamp;
      return; // need two IMU stamps to get dt
    } else {
      dt = (stamp - last_imu_stamp_).toSec();
      if (dt <= 0.0 || dt > 0.1) dt = dtMaxEst_;
    }

    // gravity calibration (startup_avg)
    if (!calib_done_ && gravity_mode_ == "startup_avg") {
      if (calib_count_ == 0) calib_start_ = stamp;
      calib_sum_ax_ += msg->linear_acceleration.x;
      ++calib_count_;
      const double elapsed = (stamp - calib_start_).toSec();
      if (elapsed >= startup_calib_time_) {
        g_x_ = (calib_count_>0) ? (calib_sum_ax_ / calib_count_) : 0.0;
        calib_done_ = true;
        ROS_INFO("[EKF] gravity X calibrated = %.5f m/s^2", g_x_);
      }
    }

    // accel low-pass
    const double wc = 2.0 * M_PI * std::max(1e-3, imu_lpf_hz_);
    const double alpha = std::exp(-wc * dt);
    const double ax_raw = msg->linear_acceleration.x;
    a_meas_x_lp_ = alpha * a_meas_x_lp_ + (1.0 - alpha) * ax_raw;

    // model input
    ModelInput u;
    u.a_meas_x = a_meas_x_lp_;
    u.g_x      = g_x_;
    u.dt       = dt;

    // if disabled, don't let accel drive v
    if (gravity_mode_ == "off") {
      u.a_meas_x = 0.0;
      g_x_ = 0.0;
    }
    model_.setInput(u);

    // predict
    ekf_.predict(model_, stamp.toSec(), dt);

    // gyro Z update (r + bg)
    {
      GyroMeas gyro;
      GyroMeas::ZVec z, h;
      GyroMeas::HVec H;
      gyro.predict(ekf_.mu(), model_params_, h, H);

      double wz = msg->angular_velocity.z;   // rad/s
      z(0,0) = wz;


      Eigen::Matrix<double,1,1> sqrtR;
      sqrtR(0,0) = gyro_std_;
      ekf_.update(z, h, H, sqrtR);
    }

    // publish
    publishOdom(stamp);
    last_imu_stamp_ = stamp;
  }

  // ===================== Wheels =====================
  void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg)
  {
    double w_sum_abs = 0.0; 
    int cnt = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& j = msg->name[i];
      if (j == "wheelfl_motor" || j == "wheelfr_motor") {     // exact names
        if (i < msg->velocity.size()) {
          w_sum_abs += std::fabs(msg->velocity[i]);           // <-- use ABS
          ++cnt;
        }
      }
    }
    if (cnt < 1) return;

    const double w_avg = w_sum_abs / static_cast<double>(cnt); // rad/s (magnitude)
    const double v_meas = w_avg * rw_;                         // m/s
    // Build meas
    SpeedMeas spd; SpeedMeas::ZVec z, h; SpeedMeas::HVec H;
    spd.predict(ekf_.mu(), model_params_, h, H);
    z(0,0) = v_meas;

    // DEBUG — before update
    double v_before = ekf_.mu()(3);
    double innov    = (z - h)(0,0);

    Eigen::Matrix<double,1,1> sqrtR; sqrtR(0,0) = speed_std_;
    ekf_.update(z, h, H, sqrtR);

    // DEBUG — after update
    double v_after = ekf_.mu()(3);
    ROS_INFO_THROTTLE(0.5,
      "[EKF] speed meas: z=%.3f h=%.3f innov=%.3f  v: before=%.3f after=%.3f  R=%.4f",
      v_meas, h(0,0), innov, v_before, v_after, speed_std_);

    Eigen::Matrix<double,7,7> P = ekf_.S().transpose() * ekf_.S();
    ROS_INFO_THROTTLE(0.5, "[EKF] Pvv=%.6f  (should be ~1e-2 .. 1e+0 initially)", P(3,3));

  }




  // ===================== Odom out =====================
  void publishOdom(const ros::Time& stamp)
  {
    const auto& x = ekf_.mu();
    const double X=x(0), Y=x(1), psi=x(2), v=x(3), r=x(4);

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;

    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(psi);
    odom.pose.pose.orientation = q;

    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = r;

    // diag covariances from P
    Eigen::Matrix<double,7,7> P = ekf_.S().transpose() * ekf_.S();
    odom.pose.covariance[0]  = P(0,0);
    odom.pose.covariance[7]  = P(1,1);
    odom.pose.covariance[35] = P(2,2);
    odom.twist.covariance[0]  = P(3,3);
    odom.twist.covariance[35] = P(4,4);

    odom_pub_.publish(odom);

    if (publish_tf_) {
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = stamp;
      tf_msg.header.frame_id = odom_frame_;
      tf_msg.child_frame_id  = base_frame_;
      tf_msg.transform.translation.x = X;
      tf_msg.transform.translation.y = Y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = q;
      tf_broadcaster_.sendTransform(tf_msg);
    }
  }

  // -------- members --------
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber imu_sub_, js_sub_;
  ros::Publisher  odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // params
  std::string imu_topic_, joint_states_topic_, odom_topic_;
  std::string odom_frame_, base_frame_, gravity_mode_;
  std::vector<std::string> drive_joints_;
  bool publish_tf_{true};
  double gyro_std_{0.05}, speed_std_{0.10};
  double rw_{0.033}, dtMaxEst_{0.01};
  double startup_calib_time_{2.5}, imu_lpf_hz_{15.0};

  // model / ekf
  ModelParams model_params_;
  KinematicModel model_;
  SREKF ekf_;

  // runtime
  ros::Time last_imu_stamp_;
  bool   calib_done_{false};
  ros::Time calib_start_;
  double calib_sum_ax_{0.0};
  int    calib_count_{0};
  double g_x_{0.0};
  double a_meas_x_lp_{0.0};
};

} // namespace qcar_nav

int main(int argc, char** argv)
{
  ros::init(argc, argv, "qcar_ekf");
  ros::NodeHandle nh, pnh("~");
  qcar_nav::EkfNode node(nh, pnh);
  ros::spin();
  return 0;
}
