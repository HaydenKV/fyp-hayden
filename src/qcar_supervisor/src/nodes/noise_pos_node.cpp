#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include "qcar_supervisor/noise_pos/noise_pos.h"

ros::Publisher noisy_pub;
qcar_supervisor::PositionNoiser pos_noiser(0.0001, 0.0001); // 5cm (0.05) stddev

// Set your desired print frequency (Hz) here:
const double print_hz = 10.0; // Change this value as needed

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Vector2d pos;
    pos[0] = msg->pose.pose.position.x;
    pos[1] = msg->pose.pose.position.y;

    Eigen::Vector2d noisy_pos = pos_noiser.addNoise(pos);

    double error_x = noisy_pos[0] - pos[0];
    double error_y = noisy_pos[1] - pos[1];

    // Throttle print to 10 Hz
    // static ros::Time last_print = ros::Time(0);
    // if ((msg->header.stamp - last_print).toSec() >= 1.0 / print_hz) {
    //     ROS_INFO_STREAM("Noisy position error: dx=" << error_x << " dy=" << error_y);
    //     last_print = msg->header.stamp;
    // }
    
    nav_msgs::Odometry noisy_msg = *msg;
    noisy_msg.pose.pose.position.x = noisy_pos[0];
    noisy_msg.pose.pose.position.y = noisy_pos[1];
    noisy_pub.publish(noisy_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "noisy_pos_node");
    ros::NodeHandle nh;
    noisy_pub = nh.advertise<nav_msgs::Odometry>("noisy_odom", 1);
    ros::Subscriber sub = nh.subscribe("/odom", 1, odomCallback);
    ros::spin();
    return 0;
}