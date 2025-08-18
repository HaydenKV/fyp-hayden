#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <qcar_visnav/ConeArray.h>

struct Params {
  std::string cones_topic{"/cones"};
  std::string markers_topic{"/cones_markers"};
  double sphere_diameter{0.22};   // ~ cone footprint in RViz (m)
};

static Params P;
static ros::Publisher pub;

static void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
  visualization_msgs::MarkerArray ma;

  // Clear previous markers
  {
    visualization_msgs::Marker clear;
    clear.header = msg->header;
    clear.ns = "cones";
    clear.id = 0;
    clear.action = visualization_msgs::Marker::DELETEALL;
    ma.markers.push_back(clear);
  }

  int id = 1;
  for (const auto& c : msg->cones) {
    // convert polar (r,theta) -> XY in lidar frame
    const double x = c.range * std::cos(c.bearing);
    const double y = c.range * std::sin(c.bearing);

    // color pick
    float r=0.6f, g=0.6f, b=0.6f; // unknown=gray
    if (c.color == 1) { r=0.2f; g=0.4f; b=1.0f; }     // blue
    else if (c.color == 2) { r=1.0f; g=1.0f; b=0.2f;} // yellow
    else if (c.color == 3) { r=1.0f; g=0.5f; b=0.1f;} // orange

    // sphere
    visualization_msgs::Marker m;
    m.header         = msg->header;
    m.ns             = "cones";
    m.id             = id++;
    m.type           = visualization_msgs::Marker::SPHERE;
    m.action         = visualization_msgs::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.x = P.sphere_diameter;
    m.scale.y = P.sphere_diameter;
    m.scale.z = P.sphere_diameter * 0.5; // shorter
    m.color.a = 0.9;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.lifetime = ros::Duration(0.25); // refresh at ~10 Hz
    ma.markers.push_back(m);
  }

  pub.publish(ma);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cone_markers");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("cones_topic",   P.cones_topic,   P.cones_topic);
  pnh.param("markers_topic", P.markers_topic, P.markers_topic);
  pnh.param("sphere_diameter", P.sphere_diameter, P.sphere_diameter);

  pub = nh.advertise<visualization_msgs::MarkerArray>(P.markers_topic, 1, false);
  ros::Subscriber sub = nh.subscribe<qcar_visnav::ConeArray>(P.cones_topic, 1, &conesCb);

  ros::spin();
  return 0;
}
