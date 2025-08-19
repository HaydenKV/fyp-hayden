// cones_vis_node.cpp
#include <ros/ros.h>
#include <qcar_visnav/ConeArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string>

struct VParams {
  std::string cones_topic{"/cones"};
  std::string markers_topic{"/cones_markers"};
  double sphere_diam{0.25};   // visual size of cone center
  double text_scale{0.18};
  double lifetime{0.0};       // seconds; 0=forever (easier to debug)
  bool   show_text{true};
} PV;

static ros::Publisher g_pub;

static void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;
  arr.markers.reserve(msg->cones.size() * (PV.show_text ? 2 : 1));

  const std::string ns_pts  = "cones_centers";
  const std::string ns_text = "cones_labels";
  int id = 0;

  // Wipe previous markers in both namespaces (helps when #cones decreases)
  {
    visualization_msgs::Marker wipe;
    wipe.header = msg->header;
    wipe.action = visualization_msgs::Marker::DELETEALL;

    wipe.ns = ns_pts;
    wipe.id = 0;
    arr.markers.push_back(wipe);

    wipe.ns = ns_text;
    wipe.id = 0;
    arr.markers.push_back(wipe);
  }

  for (const auto& c : msg->cones)
  {
    // polar (range,bearing) -> lidar XY (to match LaserScan convention)
    const double x = c.range * std::cos(c.bearing);
    const double y = c.range * std::sin(c.bearing);

    // --- sphere marker ---
    visualization_msgs::Marker m;
    m.header = msg->header;             // e.g., frame_id="lidar"
    m.ns = ns_pts;
    m.id = ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;

    m.scale.x = PV.sphere_diam;
    m.scale.y = PV.sphere_diam;
    m.scale.z = PV.sphere_diam;

    // color by cone.color: 0 unknown, 1 blue, 2 yellow, 3 orange
    float r=0.85f,g=0.85f,b=0.85f;
    switch (c.color) {
      case 1: r=0.15f; g=0.35f; b=1.0f;  break;  // blue
      case 2: r=1.0f;  g=0.85f; b=0.0f;  break;  // yellow
      case 3: r=1.0f;  g=0.4f;  b=0.0f;  break;  // orange
      default: break;                               // grey
    }
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95f;
    m.lifetime = ros::Duration(PV.lifetime);
    arr.markers.push_back(m);

    if (PV.show_text) {
      visualization_msgs::Marker t;
      t.header = msg->header;
      t.ns = ns_text;
      t.id = ++id;
      t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::Marker::ADD;

      t.pose.position.x = x;
      t.pose.position.y = y;
      t.pose.position.z = 0.05;   // just above ground

      t.scale.z = PV.text_scale;
      t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 0.9f;

      // quick diagnostics
      const double sig_r = std::sqrt(std::max(0.0, c.r_var));
      auto r_str  = std::to_string(c.range);   r_str  = r_str.substr(0, 4);
      auto sr_str = std::to_string(sig_r);     sr_str = sr_str.substr(0, 4);
      t.text = "r=" + r_str + "  σr=" + sr_str;

      t.lifetime = ros::Duration(PV.lifetime);
      arr.markers.push_back(t);
    }
  }

  g_pub.publish(arr);

  ROS_INFO_STREAM_THROTTLE(1.0, "[cones_vis] cones=" << msg->cones.size()
                           << " markers_pub=" << arr.markers.size());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cones_vis");
  ros::NodeHandle nh, pnh("~");

  pnh.param("cones_topic",   PV.cones_topic,   PV.cones_topic);
  pnh.param("markers_topic", PV.markers_topic, PV.markers_topic);
  pnh.param("sphere_diam",   PV.sphere_diam,   PV.sphere_diam);  // NOTE: sphere_diam
  pnh.param("text_scale",    PV.text_scale,    PV.text_scale);
  pnh.param("lifetime",      PV.lifetime,      PV.lifetime);
  pnh.param("show_text",     PV.show_text,     PV.show_text);

  g_pub = nh.advertise<visualization_msgs::MarkerArray>(PV.markers_topic, 1, false);
  auto sub = nh.subscribe(PV.cones_topic, 1, &conesCb);

  ROS_INFO_STREAM("[cones_vis] Subscribing to " << PV.cones_topic
                  << " -> publishing MarkerArray on " << PV.markers_topic);
  ros::spin();
  return 0;
}
