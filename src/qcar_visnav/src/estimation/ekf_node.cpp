#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>

#include "qcar_visnav/estimation/ekf_estimator.h"
#include "qcar_visnav/msg/StateEstimate.h"  // Your custom message for state output

class EKFNode
{
public:
    EKFNode(ros::NodeHandle& nh)
    : nh_(nh)
    {
        // Load parameters if needed
        double dt;
        nh_.param("ekf/dt", dt, 0.01);
        estimator_.initialize(dt);

        // Subscribers
        imu_sub_ = nh_.subscribe("/qcar/imu", 10, &EKFNode::imuCallback, this);
        encoder_sub_ = nh_.subscribe("/qcar/encoder", 10, &EKFNode::encoderCallback, this);
        gps_sub_ = nh_.subscribe("/qcar/gps", 10, &EKFNode::gpsCallback, this);

        // Publisher
        state_pub_ = nh_.advertise<qcar_visnav::StateEstimate>("qcar/state_estimate", 10);

        // Timer to periodically publish state
        pub_timer_ = nh_.createTimer(ros::Duration(dt), &EKFNode::publishState, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, encoder_sub_, gps_sub_;
    ros::Publisher state_pub_;
    ros::Timer pub_timer_;

    EKFEstimator estimator_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        Eigen::VectorXd z(3);
        z << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->angular_velocity.z;
        estimator_.updateIMU(z);
    }

    void encoderCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        Eigen::VectorXd z(2);
        z << msg->twist.linear.x, msg->twist.angular.z;
        estimator_.updateEncoders(z);
    }

    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        Eigen::VectorXd z(3);
        z << msg->pose.position.x, msg->pose.position.y, 0.0;  // yaw not observed directly
        estimator_.updateGPS(z);
    }

    void publishState(const ros::TimerEvent&)
    {
        estimator_.predict();

        const Eigen::VectorXd& x = estimator_.getState();

        qcar_visnav::StateEstimate msg;
        msg.header.stamp = ros::Time::now();
        msg.u = x(0);
        msg.v = x(1);
        msg.r = x(2);
        msg.omega_f = x(3);
        msg.omega_r = x(4);
        msg.delta = x(5);
        msg.north = x(6);
        msg.east = x(7);
        msg.psi = x(8);
        msg.gyro_bias = x(9);

        state_pub_.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;

    EKFNode node(nh);
    ros::spin();

    return 0;
}
