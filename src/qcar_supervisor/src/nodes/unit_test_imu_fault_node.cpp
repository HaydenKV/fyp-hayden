#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class TimedFaultInjector {
public:
  TimedFaultInjector(ros::NodeHandle& nh)
    : sub_(nh.subscribe("/imu", 10, &TimedFaultInjector::callback, this)),
      pub_(nh.advertise<sensor_msgs::Imu>("/imu_faulty", 10))  // override only when needed
  {}

private:
  ros::Subscriber sub_;
  ros::Publisher pub_;

  void callback(const sensor_msgs::Imu::ConstPtr& msg) {
      // Extract simulation time from the IMU message header
      double t = msg->header.stamp.toSec();

      // Define fault windows: time intervals during which faults are injected
      // You can extend or modify these windows for different test scenarios
      bool fault_active = (t >= 80.0 && t < 100.0) || (t >= 120.0 && t < 135.0);

      // Copy the incoming IMU message so we can modify it safely
      sensor_msgs::Imu modified = *msg;

      // If we're inside a fault window, inject the fault by zeroing out the accelerometer
      // This simulates a sensor failure or dropout
      if (fault_active) {
        modified.linear_acceleration.x = 0.0;
        modified.linear_acceleration.y = 0.0;
        modified.linear_acceleration.z = 0.0;
      }

      // Publish the (possibly modified) IMU message to /imu_faulty
      // This ensures downstream nodes receive a continuous stream, whether faulty or healthy
      pub_.publish(modified);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "timed_faulty_imu_node");
  ros::NodeHandle nh;

  while (ros::Time::now().toSec() == 0.0) {
    ROS_INFO_THROTTLE(1.0, "Waiting for simulated time (/clock)...");
    ros::Duration(0.1).sleep();
  }

  TimedFaultInjector injector(nh);
  ros::spin();
  return 0;
}
