#pragma once
#include <string>
#include <tf2_ros/buffer.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace qcar_visnav { namespace slam {

bool transformConeToOdom(tf2_ros::Buffer& tfbuf,
                         const std_msgs::Header& header,
                         double range, double bearing,
                         const std::string& odom_frame,
                         geometry_msgs::Point& out);

}} // namespace
