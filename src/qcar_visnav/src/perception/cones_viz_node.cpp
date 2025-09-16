// cones_viz_node.cpp
//
// Visualise raw and tracked cones with distinct colouring rules.
//  - RAW  (/cones):        always grey
//  - TRACKED (/cones_colored by default):
//        color==0 -> green (unknown yet)
//        color==1 -> blue
//        color==2 -> yellow
//        color==3 -> orange
//
// Publishes two MarkerArray topics:
//   /cones_markers_raw
//   /cones_markers_tracked

#include <ros/ros.h>
#include <qcar_visnav/ConeArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <string>

struct VParams {
  // I/O topics
  std::string raw_cones_topic{"/cones"};
  std::string tracked_cones_topic{"/cones_colored"};   // default updated
  std::string markers_topic_raw{"/cones_markers_raw"};
  std::string markers_topic_tracked{"/cones_markers_tracked"};

  // Styling
  double sphere_diam_raw{0.25};
  double sphere_diam_tracked{0.22};
  double text_scale{0.18};
  double lifetime{0.0};
  bool   show_text{true};
  std::string label_mode{"id"}; // "stats" or "id"

  // TF stamping strategy
  bool   use_latest_tf{true};
  double stamp_offset{0.0};

  // Vertical offsets so layers don't z-fight in RViz
  double z_offset_raw{0.00};
  double z_offset_tracked{0.10};
} PV;

static ros::Publisher g_pub_raw;
static ros::Publisher g_pub_trk;

static inline void setGrey(visualization_msgs::Marker& m)   { m.color.r=0.85; m.color.g=0.85; m.color.b=0.85; m.color.a=0.95; }
static inline void setGreen(visualization_msgs::Marker& m)  { m.color.r=0.10; m.color.g=0.90; m.color.b=0.20; m.color.a=0.95; }
static inline void setBlue(visualization_msgs::Marker& m)   { m.color.r=0.15; m.color.g=0.35; m.color.b=1.00; m.color.a=0.95; }
static inline void setYellow(visualization_msgs::Marker& m) { m.color.r=1.00; m.color.g=0.85; m.color.b=0.00; m.color.a=0.95; }
static inline void setOrange(visualization_msgs::Marker& m) { m.color.r=1.00; m.color.g=0.40; m.color.b=0.00; m.color.a=0.95; }

static void fillText(visualization_msgs::Marker& t, const qcar_visnav::Cone& c)
{
  if (PV.label_mode == "stats") {
    const double sig_r = std::sqrt(std::max(0.0, c.r_var));
    std::stringstream ss; ss.setf(std::ios::fixed); ss.precision(2);
    ss << "r=" << c.range << "  σr=" << sig_r;
    t.text = ss.str();
  } else {
    t.text = "#" + std::to_string(c.id);
  }
}

static void buildMarkers(const qcar_visnav::ConeArray::ConstPtr& msg,
                         bool is_tracked,
                         visualization_msgs::MarkerArray& out)
{
  out.markers.clear();

  const std::string ns_pts  = is_tracked ? "cones_tracked_centers" : "cones_raw_centers";
  const std::string ns_text = is_tracked ? "cones_tracked_labels"  : "cones_raw_labels";

  const ros::Time viz_stamp = PV.use_latest_tf
      ? ros::Time(0)
      : (msg->header.stamp + ros::Duration(PV.stamp_offset));

  // delete-all per namespace
  {
    visualization_msgs::Marker wipe;
    wipe.header.frame_id = msg->header.frame_id;
    wipe.header.stamp    = viz_stamp;
    wipe.action          = visualization_msgs::Marker::DELETEALL;
    wipe.ns = ns_pts;  wipe.id = 0; out.markers.push_back(wipe);
    wipe.ns = ns_text; wipe.id = 0; out.markers.push_back(wipe);
  }

  int id = 0;
  for (const auto& c : msg->cones)
  {
    const double x = c.range * std::cos(c.bearing);
    const double y = c.range * std::sin(c.bearing);

    // sphere
    visualization_msgs::Marker m;
    m.header.frame_id = msg->header.frame_id;
    m.header.stamp    = viz_stamp;
    m.ns   = ns_pts;
    m.id   = ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = is_tracked ? PV.z_offset_tracked : PV.z_offset_raw;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = is_tracked ? PV.sphere_diam_tracked : PV.sphere_diam_raw;

    if (!is_tracked) {
      setGrey(m);
    } else {
      switch (c.color) {
        case 1: setBlue(m);   break;
        case 2: setYellow(m); break;
        case 3: setOrange(m); break;
        case 0:
        default: setGreen(m); break;
      }
    }
    m.lifetime = ros::Duration(PV.lifetime);
    out.markers.push_back(m);

    // text
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
      t.pose.position.z = (is_tracked ? PV.z_offset_tracked : PV.z_offset_raw) + 0.05;
      t.scale.z = PV.text_scale;
      t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 0.9;
      fillText(t, c);
      t.lifetime = ros::Duration(PV.lifetime);
      out.markers.push_back(t);
    }
  }
}

static void rawCb(const qcar_visnav::ConeArray::ConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;
  buildMarkers(msg, /*is_tracked=*/false, arr);
  g_pub_raw.publish(arr);
}

static void trackedCb(const qcar_visnav::ConeArray::ConstPtr& msg)
{
  visualization_msgs::MarkerArray arr;
  buildMarkers(msg, /*is_tracked=*/true, arr);
  g_pub_trk.publish(arr);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cones_viz");
  ros::NodeHandle nh, pnh("~");

  pnh.param("raw_cones_topic",       PV.raw_cones_topic,       PV.raw_cones_topic);
  pnh.param("tracked_cones_topic",   PV.tracked_cones_topic,   PV.tracked_cones_topic);
  pnh.param("markers_topic_raw",     PV.markers_topic_raw,     PV.markers_topic_raw);
  pnh.param("markers_topic_tracked", PV.markers_topic_tracked, PV.markers_topic_tracked);

  pnh.param("sphere_diam_raw",     PV.sphere_diam_raw,     PV.sphere_diam_raw);
  pnh.param("sphere_diam_tracked", PV.sphere_diam_tracked, PV.sphere_diam_tracked);
  pnh.param("text_scale",          PV.text_scale,          PV.text_scale);
  pnh.param("lifetime",            PV.lifetime,            PV.lifetime);
  pnh.param("show_text",           PV.show_text,           PV.show_text);
  pnh.param("label_mode",          PV.label_mode,          PV.label_mode);

  pnh.param("use_latest_tf", PV.use_latest_tf, PV.use_latest_tf);
  pnh.param("stamp_offset",  PV.stamp_offset,  PV.stamp_offset);

  pnh.param("z_offset_raw",     PV.z_offset_raw,     PV.z_offset_raw);
  pnh.param("z_offset_tracked", PV.z_offset_tracked, PV.z_offset_tracked);

  g_pub_raw = nh.advertise<visualization_msgs::MarkerArray>(PV.markers_topic_raw, 1, false);
  g_pub_trk = nh.advertise<visualization_msgs::MarkerArray>(PV.markers_topic_tracked, 1, false);

  auto sub_raw = nh.subscribe(PV.raw_cones_topic, 1, &rawCb);
  auto sub_trk = nh.subscribe(PV.tracked_cones_topic, 1, &trackedCb);

  ros::spin();
  return 0;
}
