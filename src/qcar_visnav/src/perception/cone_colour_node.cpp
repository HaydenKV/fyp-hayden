// src/perception/cone_colour_node.cpp
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_colour");
  ros::NodeHandle nh, pnh("~");

  ROS_INFO("[cone_colour] stub node running (no functionality yet)");
  // Leave a spin here so the node stays alive while you develop
  ros::spin();

  return 0;
}
