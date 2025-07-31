// =====================================================================
// FILE: src/nodes/imu_main.cpp
// =====================================================================
#include <ros/ros.h>
#include "qcar_visnav/nodes/imu_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_processing_node");
    
    try {
        qcar_visnav::AccelerometerNode node;
        node.spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("IMU node failed: %s", e.what());
        return 1;
    }
    
    return 0;
}