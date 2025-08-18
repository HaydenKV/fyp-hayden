#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <qcar_visnav/ConeArray.h>

struct Params {
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";
  std::string odom_topic = "/ekf/odom";
  std::string cones_topic = "/cones";
  std::string slam_odom_topic = "/slam/odom";
  std::string particles_topic = "/slam/particles";
  std::string map_markers_topic = "/map_markers";
};

static void odomCb(const nav_msgs::Odometry::ConstPtr& /*odom*/) {
  // no-op in stub
}

static void conesCb(const qcar_visnav::ConeArray::ConstPtr& /*cones*/) {
  // no-op in stub
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastslam2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Params P;
  pnh.param("map_frame", P.map_frame, P.map_frame);
  pnh.param("odom_frame", P.odom_frame, P.odom_frame);
  pnh.param("base_frame", P.base_frame, P.base_frame);
  pnh.param("odom_topic", P.odom_topic, P.odom_topic);
  pnh.param("cones_topic", P.cones_topic, P.cones_topic);
  pnh.param("slam_odom_topic", P.slam_odom_topic, P.slam_odom_topic);
  pnh.param("particles_topic", P.particles_topic, P.particles_topic);
  pnh.param("map_markers_topic", P.map_markers_topic, P.map_markers_topic);

  // Publishers
  ros::Publisher odom_pub      = nh.advertise<nav_msgs::Odometry>(P.slam_odom_topic, 1, false);
  ros::Publisher particles_pub = nh.advertise<geometry_msgs::PoseArray>(P.particles_topic, 1, false);
  ros::Publisher markers_pub   = nh.advertise<visualization_msgs::MarkerArray>(P.map_markers_topic, 1, false);

  // Subscribers (explicit message types + function pointers)
  ros::Subscriber odom_sub  = nh.subscribe<nav_msgs::Odometry>(P.odom_topic, 10, &odomCb);
  ros::Subscriber cones_sub = nh.subscribe<qcar_visnav::ConeArray>(P.cones_topic, 10, &conesCb);

  ros::Rate r(2.0); // 2 Hz stub publisher
  while (ros::ok()) {
    // Publish a dummy odom in map frame (0 pose). Real node will compute RBPF estimate.
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = P.map_frame;     // pose in map
    odom.child_frame_id  = P.base_frame;    // of base_link
    odom.pose.pose.orientation.w = 1.0;     // identity quaternion
    odom_pub.publish(odom);

    // Empty particles & markers in stub
    geometry_msgs::PoseArray pa;
    pa.header = odom.header;
    particles_pub.publish(pa);

    visualization_msgs::MarkerArray ma;
    markers_pub.publish(ma);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
