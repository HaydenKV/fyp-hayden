// smooth_imu_node.cpp
// A standalone ROS node that applies an exponential‐moving‐average
// filter to raw IMU accelerations and angular velocities.
//
// Subscribes to: /imu_raw  (sensor_msgs/Imu)
// Publishes to:   /imu_smoothed

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuSmoother
{
public:
  ImuSmoother(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : alpha_(0.5)
    , initialized_(false)
  {
    pnh.param("alpha", alpha_, 0.5);
    alpha_ = std::clamp(alpha_, 0.0, 1.0);

    // SUBSCRIBE raw /imu
    imu_sub_ = nh.subscribe("/imu", 10,
                 &ImuSmoother::imuCallback, this);

    // PUBLISH smoothed /imu_smoothed
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("/imu_smoothed", 10);
  }

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
    if (!initialized_) {
      // First message: seed the filter with raw values
      prev_accel_       = msg->linear_acceleration;
      prev_angular_vel_ = msg->angular_velocity;
      initialized_ = true;
    }
    // Exponential moving average:
    sensor_msgs::Imu out = *msg;  // copy header, orientation, etc.

    // Smooth linear acceleration
    out.linear_acceleration.x =
      alpha_ * msg->linear_acceleration.x
      + (1 - alpha_) * prev_accel_.x;
    out.linear_acceleration.y =
      alpha_ * msg->linear_acceleration.y
      + (1 - alpha_) * prev_accel_.y;
    out.linear_acceleration.z =
      alpha_ * msg->linear_acceleration.z
      + (1 - alpha_) * prev_accel_.z;

    // Smooth angular velocity
    out.angular_velocity.x =
      alpha_ * msg->angular_velocity.x
      + (1 - alpha_) * prev_angular_vel_.x;
    out.angular_velocity.y =
      alpha_ * msg->angular_velocity.y
      + (1 - alpha_) * prev_angular_vel_.y;
    out.angular_velocity.z =
      alpha_ * msg->angular_velocity.z
      + (1 - alpha_) * prev_angular_vel_.z;

    // Publish the smoothed IMU message
    imu_pub_.publish(out);

    // Update previous values
    prev_accel_       = out.linear_acceleration;
    prev_angular_vel_ = out.angular_velocity;
  }

  ros::Subscriber       imu_sub_;
  ros::Publisher        imu_pub_;
  double                alpha_;
  bool                  initialized_;
  geometry_msgs::Vector3 prev_accel_;
  geometry_msgs::Vector3 prev_angular_vel_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smooth_imu");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ImuSmoother smoother(nh, pnh);
  ros::spin();
  return 0;
}
