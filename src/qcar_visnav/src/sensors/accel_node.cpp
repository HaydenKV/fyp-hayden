#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "estimation/ekf.hpp"
#include "measurement/accel_measurement.hpp"

std::shared_ptr<EKF> ekf_ptr;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Extract linear acceleration (x, y only)
    Eigen::Vector2d accel_data;
    accel_data << msg->linear_acceleration.x,
                  msg->linear_acceleration.y;

    // Create measurement object
    AccelMeasurement meas(accel_data);

    // Update EKF
    ekf_ptr->update(meas);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "accel_node");
    ros::NodeHandle nh;

    ekf_ptr = std::make_shared<EKF>();
    ekf_ptr->initialize(); // assuming initialize method exists

    ros::Subscriber imu_sub = nh.subscribe("/qcar/imu", 10, imuCallback);

    ros::spin();
    return 0;
}
