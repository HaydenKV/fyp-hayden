#include <ros/ros.h>
#include "qcar_visnav/slam/fastslam2.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastslam2");
  ros::NodeHandle nh, pnh("~");
  qcar_visnav::slam::FastSLAM2 node(nh, pnh);
  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    node.spinOnce();
    r.sleep();
  }
  return 0;
}
