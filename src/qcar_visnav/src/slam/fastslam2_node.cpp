#include <ros/ros.h>
#include "qcar_visnav/slam/fastslam2_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fastslam2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  qcar_visnav::slam::FastSLAM2 fs2(nh, pnh);

  ROS_INFO("[fastslam2_node] up. Listening to ~tracked_topic and ~odom_topic, publishing particles, map markers, and weights.");
  ros::Rate r(50.0);
  while (ros::ok()) {
    fs2.spinOnce();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
