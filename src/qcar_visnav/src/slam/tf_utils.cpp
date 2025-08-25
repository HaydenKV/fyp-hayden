#include "qcar_visnav/slam/tf_utils.h"
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <cmath>

namespace qcar_visnav { namespace slam {

bool transformConeToOdom(tf2_ros::Buffer& tfbuf,
                         const std_msgs::Header& header,
                         double range, double bearing,
                         const std::string& odom_frame,
                         geometry_msgs::Point& out)
{
  geometry_msgs::PointStamped pl, po;
  pl.header = header;
  // LiDAR polar -> LiDAR Cartesian
  pl.point.x = range * std::cos(bearing);
  pl.point.y = range * std::sin(bearing);
  pl.point.z = 0.0;

  try {
    tfbuf.transform(pl, po, odom_frame, ros::Duration(0.05));
  } catch (const tf2::TransformException& e) {
    ROS_WARN_THROTTLE(1.0, "[tf_utils] TF %s->%s at t=%.3f failed: %s",
                      header.frame_id.c_str(), odom_frame.c_str(),
                      header.stamp.toSec(), e.what());
    return false;
  }
  out = po.point;
  return true;
}

}} // namespace
