// File location: qcar_visnav/src/sensors/accel_debug_printer.cpp

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include "measurement/accel_measurement.hpp"
#include "system_qcar.hpp"

Eigen::VectorXd ekf_state = Eigen::VectorXd::Zero(10);  // Shared dummy state for now

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    Eigen::Vector2d z;
    z << msg->linear_acceleration.x, msg->linear_acceleration.y;

    AccelMeasurement meas(z);
    SystemQCAR sys;

    Eigen::VectorXd h = meas.h(ekf_state, sys);
    Eigen::VectorXd y = z - h;

    ROS_INFO_STREAM("Accelerometer measurement z:    [" << z.transpose() << "]");
    ROS_INFO_STREAM("Predicted measurement h(x):     [" << h.transpose() << "]");
    ROS_INFO_STREAM("Innovation (residual) y = z-h:  [" << y.transpose() << "]\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "accel_debug_printer");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/qcar/imu", 10, imuCallback);

    ros::spin();
    return 0;
}
