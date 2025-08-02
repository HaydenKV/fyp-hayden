#include <ros/ros.h>
#include "qcar_visnav/estimator/estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimator_node");
    
    try {
        ROS_INFO("Starting QCar Estimator Node...");
        qcar_visnav::EstimatorNode estimator;
        estimator.spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Estimator node failed: %s", e.what());
        return 1;
    }
    catch (...) {
        ROS_ERROR("Estimator node failed with unknown exception");
        return 1;
    }
    
    ROS_INFO("Estimator node shutting down");
    return 0;
}