#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <qcar_visnav/ConeArray.h>
#include <qcar_visnav/Cone.h>

#include <Eigen/Core>  // Eigen::Vector2d

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>

struct Params {
  // lidar
  double min_range{0.3};
  double max_range{20.0};
  double eps{0.2};             // clustering radius [m]
  int    min_cluster_size{3};
  int    max_cluster_size{50};
  // noise (per-beam, prior to any averaging)
  double sigma_r{0.07};        // [m]
  double sigma_theta_deg{0.8}; // [deg]
  // frames/topics
  std::string lidar_topic{"/scan"};
  std::string cones_topic{"/cones"};
  std::string lidar_frame{"lidar"};
  // runtime
  int debug_print{0};
};

static Params P;
static ros::Publisher cones_pub;

// Cluster the lidar points into cones
// Simple O(N^2) radius clustering in XY (sufficient for moderate beam counts)
static std::vector<std::vector<int>> clusterIndices(
    const std::vector<Eigen::Vector2d>& pts,
    double eps, int min_sz, int max_sz) {

  const double eps2 = eps * eps;
  const int N = static_cast<int>(pts.size());
  std::vector<int> label(N, -1);
  int cluster_id = 0;
  std::vector<std::vector<int>> clusters;

  for (int i = 0; i < N; ++i) {
    if (label[i] != -1) continue;

    // seed cluster with i
    std::vector<int> stack{ i };
    label[i] = cluster_id;
    std::vector<int> members{ i };

    while (!stack.empty()) {
      const int a = stack.back(); stack.pop_back();
      for (int j = 0; j < N; ++j) {
        if (label[j] != -1) continue;
        const double dx = pts[a].x() - pts[j].x();
        const double dy = pts[a].y() - pts[j].y();
        if (dx*dx + dy*dy <= eps2) {
          label[j] = cluster_id;
          stack.push_back(j);
          members.push_back(j);
        }
      }
    }

    const int cs = static_cast<int>(members.size());
    if (cs >= min_sz && cs <= max_sz) {
      clusters.push_back(std::move(members));
      cluster_id++;
    } else {
      // mark noise (optional, not used later)
      for (int idx : members) label[idx] = -2;
    }
  }
  return clusters;
}

// Convert polar (range, bearing) to XY in lidar frame
static void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {
  // Precompute sigma_theta in rad
  const double sigma_theta = P.sigma_theta_deg * M_PI / 180.0;

  // Gather valid points in XY (laser frame)
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(scan->ranges.size());

  const double a0 = scan->angle_min;
  const double da = scan->angle_increment;

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const float r = scan->ranges[i];
    if (!std::isfinite(r)) continue;
    if (r < P.min_range || r > P.max_range) continue;

    const double th = a0 + da * static_cast<double>(i);
    pts.emplace_back(r * std::cos(th), r * std::sin(th));
  }

  // Cluster points
  const auto clusters = pts.empty()
      ? std::vector<std::vector<int>>{}
      : clusterIndices(pts, P.eps, P.min_cluster_size, P.max_cluster_size);

  // Build ConeArray message
  qcar_visnav::ConeArray out;
  out.header.stamp = scan->header.stamp;
  out.header.frame_id = P.lidar_frame; // keep consistent with TF

  // For each cluster: centroid -> (range, bearing). Covariance ~ per-beam noise / sqrt(n)
  for (const auto& c : clusters) {
    if (c.empty()) continue;

    // centroid (XY)
    Eigen::Vector2d mu(0.0, 0.0);
    for (int idx : c) mu += pts[idx];
    mu /= static_cast<double>(c.size());

    // to polar
    const double range   = mu.norm();
    const double bearing = std::atan2(mu.y(), mu.x());

    // variance scaling with n
    const double n = std::max(1, static_cast<int>(c.size()));
    const double r_var  = (P.sigma_r * P.sigma_r) / n;
    const double th_var = (sigma_theta * sigma_theta) / n;

    qcar_visnav::Cone cone;
    cone.range        = range;
    cone.bearing      = bearing;
    cone.r_var        = r_var;
    cone.bearing_var  = th_var;
    cone.color        = 0;    // unknown here (color node optional)
    cone.color_conf   = 0.0;

    out.cones.push_back(cone);
  }

  if (P.debug_print) {
    ROS_INFO_THROTTLE(1.0, "[cone_detector] points=%zu clusters=%zu",
                      pts.size(), clusters.size());
  }

  cones_pub.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "cone_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Load params (with defaults)
  pnh.param("cones_topic", P.cones_topic, P.cones_topic);
  pnh.param("lidar_topic", P.lidar_topic, P.lidar_topic);
  pnh.param("lidar_frame", P.lidar_frame, P.lidar_frame);

  pnh.param("lidar/min_range", P.min_range, P.min_range);
  pnh.param("lidar/max_range", P.max_range, P.max_range);
  pnh.param("lidar/cluster_eps", P.eps, P.eps);
  pnh.param("lidar/min_cluster_size", P.min_cluster_size, P.min_cluster_size);
  pnh.param("lidar/max_cluster_size", P.max_cluster_size, P.max_cluster_size);
  pnh.param("lidar/sigma_r", P.sigma_r, P.sigma_r);
  pnh.param("lidar/sigma_theta_deg", P.sigma_theta_deg, P.sigma_theta_deg);

  pnh.param("debug_print", P.debug_print, P.debug_print);

  cones_pub = nh.advertise<qcar_visnav::ConeArray>(P.cones_topic, 1, false);
  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(P.lidar_topic, 1, &scanCb);

  ros::spin();
  return 0;
}
