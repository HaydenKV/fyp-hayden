// ============================================================================
// cone_detector_node.cpp
//
// QCar LiDAR-only cone detector (clustering front-end).
// - Subscribes:  /scan (sensor_msgs/LaserScan)
// - Publishes:   /cones (qcar_visnav/ConeArray) in the LiDAR frame
//
// Pipeline (mode="clustering"):
//   1) Range gating on LaserScan beams
//   2) Scan-order segmentation using a range-jump threshold tau(r)=tau0+alpha*r
//   3) Local Euclidean clustering per segment (small neighborhoods => fast)
//   4) Cluster centroid -> detection (range,bearing) + covariance
//
// Notes:
// - We publish in the LiDAR frame (header.frame_id) and keep detections in polar
//   coordinates as expected by the rest of your stack.
// - Covariance comes from per-beam noise divided by sqrt(N) within the cluster.
// - Everything is tuned by params in your YAML; no hard-coded topics here.
//
// Author: you (+ light comment polish)
// ============================================================================

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <qcar_visnav/ConeArray.h>
#include <qcar_visnav/Cone.h>

#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>

struct Params {
  // detection mode: "clustering" or "circular_hough" (mutually exclusive)
  std::string detection_mode{"clustering"};

  // lidar topics/frames
  std::string lidar_topic{"/scan"};
  std::string cones_topic{"/cones"};
  std::string lidar_frame{"lidar"}; // if empty, will use scan->header.frame_id

  // gating
  double min_range{0.3};
  double max_range{15.0};

  // scan-order segmentation (range jump threshold tau(r)=tau0 + alpha*r)
  double seg_tau0{0.12};      // [m]
  double seg_tau_alpha{0.03}; // [m/m]

  // local clustering (inside each segment)
  double eps{0.20};           // [m]
  int    min_cluster_size{3};
  int    max_cluster_size{50};

  // noise (per-beam)
  double sigma_r{0.07};         // [m]
  double sigma_theta_deg{0.8};  // [deg]

  int debug_print{0};
};

static Params P;
static ros::Publisher cones_pub;

// ---------------------------------------------------------------------------
// Simple segment-local clustering (Euclidean, single-link style).
// Input: 2D points in segment order
// Output: cluster index lists
// ---------------------------------------------------------------------------
static std::vector<std::vector<int>> clusterIndices(
    const std::vector<Eigen::Vector2d>& pts,
    double eps, int min_sz, int max_sz)
{
  const double eps2 = eps * eps;
  const int N = static_cast<int>(pts.size());
  std::vector<int> label(N, -1);
  int cluster_id = 0;
  std::vector<std::vector<int>> clusters;

  for (int i = 0; i < N; ++i) {
    if (label[i] != -1) continue;

    // start a new component from i
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
      // mark discarded points (noise) to avoid reseeding
      for (int idx : members) label[idx] = -2;
    }
  }
  return clusters;
}

// ---------------------------------------------------------------------------
// Build contiguous scan-order segments using a range-jump threshold.
// We split when |r_i - r_{i-1}| > tau(r) or beam indices are not consecutive.
// ---------------------------------------------------------------------------
static std::vector<std::vector<int>> buildSegments(
    const sensor_msgs::LaserScan& scan,
    const std::vector<int>& kept_scan_indices)
{
  std::vector<std::vector<int>> segments;
  if (kept_scan_indices.empty()) return segments;

  const auto& ranges = scan.ranges;
  auto start_new_segment = [&](int i_prev, int i_cur) -> bool {
    if (!std::isfinite(ranges[i_prev]) || !std::isfinite(ranges[i_cur])) return true;
    const double r_prev = ranges[i_prev];
    const double r_cur  = ranges[i_cur];
    const double tau = P.seg_tau0 + P.seg_tau_alpha * std::max(r_prev, r_cur);
    return std::fabs(r_cur - r_prev) > tau;
  };

  std::vector<int> current;
  current.push_back(kept_scan_indices.front());

  for (size_t k = 1; k < kept_scan_indices.size(); ++k) {
    int i_prev = kept_scan_indices[k-1];
    int i_cur  = kept_scan_indices[k];
    const bool index_gap = (i_cur != i_prev + 1);

    if (index_gap || start_new_segment(i_prev, i_cur)) {
      if (current.size() >= static_cast<size_t>(P.min_cluster_size)) {
        segments.push_back(current);
      }
      current.clear();
    }
    current.push_back(i_cur);
  }

  if (current.size() >= static_cast<size_t>(P.min_cluster_size)) {
    segments.push_back(current);
  }

  return segments;
}

static void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  if (P.detection_mode != "clustering") {
    if (P.debug_print) {
      ROS_WARN_THROTTLE(2.0,
        "[cone_detector] detection.mode='%s' not implemented here. No output.",
        P.detection_mode.c_str());
    }
    return;
  }

  const double sigma_theta = P.sigma_theta_deg * M_PI / 180.0;
  const double a0 = scan->angle_min;
  const double da = scan->angle_increment;

  // 1) Keep valid beams within range
  std::vector<int> kept_scan_indices;
  kept_scan_indices.reserve(scan->ranges.size());

  for (int i = 0; i < static_cast<int>(scan->ranges.size()); ++i) {
    const float r = scan->ranges[i];
    if (!std::isfinite(r)) continue;
    if (r < P.min_range || r > P.max_range) continue;
    kept_scan_indices.push_back(i);
  }

  // 2) Build segments with range-jump rule
  const auto segments = buildSegments(*scan, kept_scan_indices);

  // 3) Local clustering per segment and emit ConeArray
  qcar_visnav::ConeArray out;
  out.header.stamp = scan->header.stamp;
  out.header.frame_id = P.lidar_frame.empty() ? scan->header.frame_id : P.lidar_frame;

  size_t total_pts = 0, total_clusters = 0;

  for (const auto& seg : segments) {
    // Project this segment to XY (LiDAR frame)
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(seg.size());
    for (int i : seg) {
      const double r  = scan->ranges[i];
      const double th = a0 + da * static_cast<double>(i);
      pts.emplace_back(r * std::cos(th), r * std::sin(th));
    }
    total_pts += pts.size();

    // Euclidean clustering (local)
    const auto clusters = clusterIndices(pts, P.eps, P.min_cluster_size, P.max_cluster_size);

    // Convert each cluster to Cone (polar + covariance)
    for (const auto& c : clusters) {
      if (c.empty()) continue;
      Eigen::Vector2d mu(0.0, 0.0);
      for (int idx : c) mu += pts[idx];
      mu /= static_cast<double>(c.size());

      const double range   = mu.norm();
      const double bearing = std::atan2(mu.y(), mu.x());

      const double n = std::max(1, static_cast<int>(c.size()));
      const double r_var  = (P.sigma_r * P.sigma_r) / n;
      const double th_var = (sigma_theta * sigma_theta) / n;

      qcar_visnav::Cone cone;
      cone.range        = range; // meters
      cone.bearing      = bearing; // radians
      cone.r_var        = r_var; // m^2
      cone.bearing_var  = th_var; // radians^2
      cone.color        = 0;    // unknown (LiDAR-only)
      cone.color_conf   = 0.0;

      out.cones.push_back(cone);
      total_clusters++;
    }
  }

  if (P.debug_print) {
    ROS_INFO_THROTTLE(1.0, "[cone_detector] mode=%s segs=%zu points=%zu clusters=%zu",
                      P.detection_mode.c_str(), segments.size(), total_pts, total_clusters);
  }

  cones_pub.publish(out); // Publish to /cones topic
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // mode
  pnh.param("detection/mode", P.detection_mode, P.detection_mode);

  // topics/frames
  pnh.param("cones_topic", P.cones_topic, P.cones_topic);
  pnh.param("lidar_topic", P.lidar_topic, P.lidar_topic);
  pnh.param("lidar_frame", P.lidar_frame, P.lidar_frame);

  // gating
  pnh.param("lidar/min_range", P.min_range, P.min_range);
  pnh.param("lidar/max_range", P.max_range, P.max_range);

  // segmentation
  pnh.param("seg/tau0",        P.seg_tau0, P.seg_tau0);
  pnh.param("seg/tau_alpha",   P.seg_tau_alpha, P.seg_tau_alpha);

  // clustering
  pnh.param("cluster/eps",              P.eps, P.eps);
  pnh.param("cluster/min_cluster_size", P.min_cluster_size, P.min_cluster_size);
  pnh.param("cluster/max_cluster_size", P.max_cluster_size, P.max_cluster_size);

  // noise
  pnh.param("noise/sigma_r",          P.sigma_r, P.sigma_r);
  pnh.param("noise/sigma_theta_deg",  P.sigma_theta_deg, P.sigma_theta_deg);

  pnh.param("debug_print", P.debug_print, P.debug_print);

  cones_pub = nh.advertise<qcar_visnav::ConeArray>(P.cones_topic, 1, false);
  ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>(P.lidar_topic, 1, &scanCb);

  ros::spin();
  return 0;
}
