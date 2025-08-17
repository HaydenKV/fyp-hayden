#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>

#include "qcar_visnav/estimation/model/kinematic_model.h"
#include "qcar_visnav/estimation/ekf/ekf_core.h"
#include "qcar_visnav/estimation/sensors/gyro.h"
#include "qcar_visnav/estimation/sensors/speed.h"
#include "qcar_visnav/estimation/utils/odometry_buffer.h"

namespace qcar_nav {

class EkfNode {
public:
  // Always 6-state: [vx, vy, r, bg, bax, bay]
  static constexpr int STATE_SIZE = 6;
  using SREKF = EkfCore<STATE_SIZE>;

  EkfNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // Load parameters
    pnh_.param<std::string>("imu_topic", imu_topic_, "/imu");
    pnh_.param<std::string>("joint_states_topic", joint_states_topic_, "/qcar/joint_states");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/qcar/ekf/odom");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");

    pnh_.param("gyro_std", gyro_std_, 1.0);
    pnh_.param("speed_std", speed_std_, 0.1);
    pnh_.param("rw", rw_, 0.033);
    pnh_.param("wheelbase", wheelbase_, 0.258);
    pnh_.param("dtMaxEst", dtMaxEst_, 0.01);

    // Process noise for [vx, vy, r, bg, bax, bay] (continuous-time intensities)
    std::vector<double> q_vals;
    pnh_.param("q", q_vals, std::vector<double>{2.0, 2.0, 1.0, 0.001, 0.01, 0.01});
    for (int i = 0; i < 6 && i < static_cast<int>(q_vals.size()); ++i) {
      model_params_.q(i) = q_vals[i];
    }

    // Drive wheel joints
    drive_joints_ = {"wheelfl_motor", "wheelfr_motor"};
    
    // Publishers/Subscribers
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &EkfNode::imuCb, this);
    js_sub_ = nh_.subscribe(joint_states_topic_, 50, &EkfNode::jointStatesCb, this);
    truth_sub_ = nh_.subscribe("/odom", 1, &EkfNode::truthCb, this);

    //-------------------------------------------------
    // --- Initial conditions from YAML ---
    std::vector<double> init_x, init_Pdiag;
    if (!pnh_.getParam("initial_state", init_x) || init_x.size() != STATE_SIZE) {
      ROS_WARN("[EKF] 'initial_state' missing/size!=6; defaulting to zeros");
      init_x.assign(STATE_SIZE, 0.0);
    }
    if (!pnh_.getParam("initial_cov_diag", init_Pdiag) || init_Pdiag.size() != STATE_SIZE) {
      ROS_WARN("[EKF] 'initial_cov_diag' missing/size!=6; defaulting to 0.1 diag");
      init_Pdiag.assign(STATE_SIZE, 0.1);
    }

    // EKF Initialization (from YAML)
    SREKF::Vec mu0 = SREKF::Vec::Zero(); // Start at zero velocity
    for (int i = 0; i < STATE_SIZE; ++i) mu0(i) = init_x[i]; // This replaces zeros to YAML values

    SREKF::Mat P0 = SREKF::Mat::Zero();
    for (int i = 0; i < STATE_SIZE; ++i) P0(i,i) = init_Pdiag[i]; // This replaces zeros to YAML values
    ekf_.setInitial(mu0, P0); 

    
    //-------------------------------------------------
    // Model setup
    model_params_.L = wheelbase_;
    model_params_.dtMaxEst = dtMaxEst_;
    model_.setParams(model_params_);

    // Runtime variables
    last_imu_stamp_ = ros::Time(0);
    odom_buffer_ = std::make_unique<OdometryBuffer>(200);

    ROS_INFO("[EKF] 6-state velocity EKF initialized: [vx, vy, r, bg, bax, bay]");
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher odom_pub_;
  ros::Subscriber imu_sub_, js_sub_, truth_sub_;

  std::string imu_topic_, joint_states_topic_, odom_topic_;
  std::string odom_frame_, base_frame_;
  std::vector<std::string> drive_joints_;

  double gyro_std_, speed_std_, rw_, wheelbase_, dtMaxEst_;

  SREKF ekf_;
  KinematicModel model_;
  KinematicModel::ModelParams model_params_;
  std::unique_ptr<OdometryBuffer> odom_buffer_;

  ros::Time last_imu_stamp_;
  nav_msgs::Odometry latest_truth_;
  bool has_truth_ = false;

  void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
    const ros::Time stamp = msg->header.stamp;
    
    if (last_imu_stamp_.isZero()) {
      last_imu_stamp_ = stamp;
      return;
    }

    double dt = (stamp - last_imu_stamp_).toSec();
    if (dt <= 0.0 || dt > dtMaxEst_) dt = dtMaxEst_;

    // Simple model input (no acceleration for now to avoid complexity)
    // Set inputs here ----------------------------------------------------------- add accel when working
    KinematicModel::ModelInput u;
    u.a_meas_x = 0.0;  // Disabled for simplicity
    u.a_meas_y = 0.0;
    u.delta = 0.0;     // No steering input for now
    u.dt = dt;
    model_.setInput(u);

    // Predict
    ekf_.predict(model_, stamp.toSec(), dt);

    // Gyro update
    GyroMeas gyro;
    GyroMeas::ZVec z, h;
    GyroMeas::HVec H;
    gyro.predict(ekf_.mu(), model_params_, h, H);

    z(0) = msg->angular_velocity.z;
    Eigen::Matrix<double,1,1> sqrtR;
    sqrtR(0) = gyro_std_; // std -> squared inside update()
    ekf_.update(z, h, H, sqrtR);

    publishOdom(stamp);
    last_imu_stamp_ = stamp;
  }

  void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg) {
    double w_sum = 0.0;
    int cnt = 0;

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& name = msg->name[i];
      if (std::find(drive_joints_.begin(), drive_joints_.end(), name) != drive_joints_.end()) {
        if (i < msg->velocity.size()) {
          w_sum += std::abs(msg->velocity[i]);
          cnt++;
        }
      }
    }

    if (cnt < 1) return;

    double v_meas = (w_sum / cnt) * rw_;

    // Speed update
    SpeedMeas speed;
    SpeedMeas::ZVec z, h;
    SpeedMeas::HVec H;
    speed.predict(ekf_.mu(), model_params_, h, H);

    z(0) = v_meas;
    Eigen::Matrix<double,1,1> sqrtR;
    sqrtR(0) = speed_std_; // std -> squared inside update()
    ekf_.update(z, h, H, sqrtR);
  }

  void truthCb(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_truth_ = *msg;
    has_truth_ = true;
  }

  void publishOdom(const ros::Time& stamp) {
    const auto& x = ekf_.mu();

    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    // No position (velocity-only EKF)
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    // Body-frame velocities
    odom.twist.twist.linear.x = x(0);   // vx
    odom.twist.twist.linear.y = x(1);   // vy
    odom.twist.twist.angular.z = x(2);  // r

    // Simple covariance
    auto P = ekf_.getCovariance();
    odom.twist.covariance[0] = P(0,0);   // vx variance
    odom.twist.covariance[7] = P(1,1);   // vy variance
    odom.twist.covariance[35] = P(2,2);  // r variance

    odom_pub_.publish(odom);
    odom_buffer_->addEntry(odom);

    // Log comparison
    if (has_truth_) {
      double true_vx = latest_truth_.twist.twist.linear.x;
      double true_r = latest_truth_.twist.twist.angular.z;
      double err_vx = x(0) - true_vx;
      double err_r = x(2) - true_r;
      (void)err_vx; (void)err_r; (void)true_vx; (void)true_r;
      // ROS_INFO_THROTTLE(1.0, 
      //       "[EKF] TRUE: vx=%.2f r=%.2f | EST: vx=%.2f r=%.2f | ERR: vx=%.3f r=%.1f° | bias: bg=%.3f",
      //       true_vx,     true_r,           // Ground truth
      //       x(0),        x(2),             // EKF estimates  
      //       err_vx,      err_r*180/M_PI,   // Errors (r converted to degrees)
      //       x(3));                         // Gyro bias estimate
    }
  }
};

} // namespace qcar_nav

int main(int argc, char** argv) {
  ros::init(argc, argv, "velocity_ekf_node");
  ros::NodeHandle nh, pnh("~");
  
  qcar_nav::EkfNode ekf_node(nh, pnh);
  
  ROS_INFO("[EKF] Simple velocity EKF node started");
  ros::spin();
  
  return 0;
}