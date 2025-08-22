// cones_vis_node.cpp
#include <ros/ros.h>
#include <qcar_visnav/ConeArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>

struct VParams {
  std::string cones_topic{"/cones"};
  std::string markers_topic{"/cones_markers"};
  double sphere_diam{0.25};
  double text_scale{0.18};
  double lifetime{0.0};     // seconds; 0 -> forever
  bool   show_text{true};

  // Handle TF timing for viz
  bool   use_latest_tf{true};   // stamp markers at time 0 so RViz uses latest TF
  double stamp_offset{0.0};     // optional offset (secs) if not using latest

  // Styling
  bool   override_color{false};
  float  cr{0.10f}, cg{0.80f}, cb{0.20f}, ca{1.0f}; // default green-ish
  std::string label_mode{"stats"}; // "stats" or "id"

  // Layering (lift to draw on top of others)
  double z_offset{0.0};         // NEW: small lift for this instance
} PV;

static ros::Publisher g_pub;

static void parseRGBA(const std::string& s, float &r, float &g, float &b, float &a) {
  std::istringstream iss(s);
  float rr=1.f, gg=1.f, bb=1.f, aa=1.f;
  iss >> rr >> gg >> bb;
  if (iss) iss >> aa;
  r = rr; g = gg; b = bb; a = aa;
}

static void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;
  arr.markers.reserve(msg->cones.size() * (PV.show_text ? 2 : 1));

  const std::string ns_pts  = "cones_centers";
  const std::string ns_text = "cones_labels";
  int id = 0;

  // Use latest TF (stamp=0) to avoid extrapolation in RViz
  const ros::Time viz_stamp = PV.use_latest_tf
      ? ros::Time(0)
      : (msg->header.stamp + ros::Duration(PV.stamp_offset));

  // Wipe existing markers in both namespaces
  {
    visualization_msgs::Marker wipe;
    wipe.header.frame_id = msg->header.frame_id;
    wipe.header.stamp    = viz_stamp;
    wipe.action = visualization_msgs::Marker::DELETEALL;

    wipe.ns = ns_pts;  wipe.id = 0; arr.markers.push_back(wipe);
    wipe.ns = ns_text; wipe.id = 0; arr.markers.push_back(wipe);
  }

  for (const auto& c : msg->cones)
  {
    // polar -> XY in the same frame as the message (lidar)
    const double x = c.range * std::cos(c.bearing);
    const double y = c.range * std::sin(c.bearing);

    // ---------- sphere ----------
    visualization_msgs::Marker m;
    m.header.frame_id = msg->header.frame_id;
    m.header.stamp    = viz_stamp;
    m.ns   = ns_pts;
    m.id   = ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;

    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = PV.z_offset;       // <— lift this layer if desired
    m.pose.orientation.w = 1.0;

    m.scale.x = PV.sphere_diam;
    m.scale.y = PV.sphere_diam;
    m.scale.z = PV.sphere_diam;

    if (PV.override_color) {
      m.color.r = PV.cr; m.color.g = PV.cg; m.color.b = PV.cb; m.color.a = PV.ca;
    } else {
      // color by cone.color: 0=grey, 1=blue, 2=yellow, 3=orange
      float r=0.85f, g=0.85f, b=0.85f, a=0.95f;
      switch (c.color) {
        case 1: r=0.15f; g=0.35f; b=1.00f; break; // blue
        case 2: r=1.00f; g=0.85f; b=0.00f; break; // yellow
        case 3: r=1.00f; g=0.40f; b=0.00f; break; // orange
        default: break;                           // grey
      }
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    }
    m.lifetime = ros::Duration(PV.lifetime);
    arr.markers.push_back(m);

    // ---------- text ----------
    if (PV.show_text) {
      visualization_msgs::Marker t;
      t.header.frame_id = msg->header.frame_id;
      t.header.stamp    = viz_stamp;
      t.ns   = ns_text;
      t.id   = ++id;
      t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      t.action = visualization_msgs::Marker::ADD;

      t.pose.position.x = x;
      t.pose.position.y = y;
      t.pose.position.z = PV.z_offset + 0.05;   // keep text above the sphere

      t.scale.z = PV.text_scale;
      t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 0.9f;

      if (PV.label_mode == "id") {
        t.text = "#" + std::to_string(c.id);
      } else { // "stats"
        const double sig_r = std::sqrt(std::max(0.0, c.r_var));
        auto r_str  = std::to_string(c.range);  r_str  = r_str.substr(0, 4);
        auto sr_str = std::to_string(sig_r);    sr_str = sr_str.substr(0, 4);
        t.text = "r=" + r_str + "  σr=" + sr_str;
      }

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
  pnh.param("sphere_diam",   PV.sphere_diam,   PV.sphere_diam);
  pnh.param("text_scale",    PV.text_scale,    PV.text_scale);
  pnh.param("lifetime",      PV.lifetime,      PV.lifetime);
  pnh.param("show_text",     PV.show_text,     PV.show_text);
  pnh.param("use_latest_tf", PV.use_latest_tf, PV.use_latest_tf);
  pnh.param("stamp_offset",  PV.stamp_offset,  PV.stamp_offset);

  pnh.param("override_color", PV.override_color, PV.override_color);
  std::string rgba = "0.10 0.80 0.20 1.0";
  pnh.param("rgba", rgba, rgba);
  parseRGBA(rgba, PV.cr, PV.cg, PV.cb, PV.ca);
  pnh.param("label_mode", PV.label_mode, PV.label_mode); // "stats" or "id"

  pnh.param("z_offset", PV.z_offset, PV.z_offset);       // NEW

  g_pub = nh.advertise<visualization_msgs::MarkerArray>(PV.markers_topic, 1, false);
  auto sub = nh.subscribe(PV.cones_topic, 1, &conesCb);

  ROS_INFO_STREAM("[cones_vis] Subscribing to " << PV.cones_topic
                  << " -> publishing MarkerArray on " << PV.markers_topic
                  << " (override_color=" << (PV.override_color?"true":"false")
                  << ", label_mode=" << PV.label_mode << ", use_latest_tf="
                  << (PV.use_latest_tf?"true":"false") << ", z_offset=" << PV.z_offset << ")");
  ros::spin();
  return 0;
}
