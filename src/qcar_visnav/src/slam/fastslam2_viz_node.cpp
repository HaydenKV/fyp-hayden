#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <vector>

struct LayerCfg {
  std::string ns;            // namespace on the combined topic (/slam/map_markers)
  double sphere_diam{0.18};
  double z_offset{0.05};
  float  r{1.f}, g{1.f}, b{1.f}, a{1.f};
};

struct VizCfg {
  std::string topic{"/slam/map_markers"};  // combined topic (one per frame)
  LayerCfg all, confirmed, locked, unmatched;
};

// ---- helpers ----------------------------------------------------------------
static void parseRGBA(const std::vector<double>& v, float& r, float& g, float& b, float& a) {
  r = (v.size()>0)?(float)v[0]:1.f;
  g = (v.size()>1)?(float)v[1]:1.f;
  b = (v.size()>2)?(float)v[2]:1.f;
  a = (v.size()>3)?(float)v[3]:1.f;
}

static visualization_msgs::Marker makeSphereList(
    int id, const std::string& ns, const std::string& frame,
    double diam, float r, float g, float b, float a)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame.empty() ? "odom" : frame;
  m.header.stamp    = ros::Time(0);               // let RViz use latest TF
  m.ns = ns;
  m.id = id;                                      // stable per layer
  m.type = visualization_msgs::Marker::SPHERE_LIST; // type=7 (efficient)
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = diam; m.scale.y = diam; m.scale.z = diam;
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
  m.lifetime = ros::Duration(0.0);
  return m;
}

static visualization_msgs::Marker makeDelete(
    int id, const std::string& ns, const std::string& frame)
{
  visualization_msgs::Marker m;
  m.header.frame_id = frame.empty() ? "odom" : frame;
  m.header.stamp    = ros::Time(0);
  m.ns = ns;
  m.id = id;
  m.action = visualization_msgs::Marker::DELETE;
  return m;
}

static void posesToPoints(const geometry_msgs::PoseArray::ConstPtr& src,
                          double z_offset,
                          std::vector<geometry_msgs::Point>& out)
{
  out.clear();
  if (!src) return;
  out.reserve(src->poses.size());
  for (const auto& pose : src->poses) {
    geometry_msgs::Point p;
    p.x = pose.position.x;
    p.y = pose.position.y;
    p.z = z_offset;
    out.push_back(p);
  }
}

// ---- node -------------------------------------------------------------------
class FastSLAM2Viz {
public:
  FastSLAM2Viz(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    loadParams();
    pub_ = nh_.advertise<visualization_msgs::MarkerArray>(cfg_.topic, 1, false);

    // Subscribe to SLAM’s PoseArrays (unchanged topics)
    sub_all_  = nh_.subscribe("/slam/landmarks_pose",           1, &FastSLAM2Viz::cbAll,  this);
    sub_conf_ = nh_.subscribe("/slam/landmarks_pose_confirmed", 1, &FastSLAM2Viz::cbConf, this);
    sub_lock_ = nh_.subscribe("/slam/landmarks_pose_locked",    1, &FastSLAM2Viz::cbLock, this);
    sub_unm_  = nh_.subscribe("/slam/unmatched_pose",           1, &FastSLAM2Viz::cbUnm,  this);

    ROS_INFO("[fastslam2_viz] publishing combined layered markers on %s (ns: %s, %s, %s, %s)",
      cfg_.topic.c_str(),
      cfg_.all.ns.c_str(), cfg_.confirmed.ns.c_str(),
      cfg_.locked.ns.c_str(), cfg_.unmatched.ns.c_str());
  }

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_all_, sub_conf_, sub_lock_, sub_unm_;

  geometry_msgs::PoseArray::ConstPtr last_all_, last_conf_, last_lock_, last_unm_;
  VizCfg cfg_;

  void loadParams() {
    pnh_.param("visualization/topic", cfg_.topic, cfg_.topic);

    auto read_layer = [&](const std::string& base, LayerCfg& L,
                          const char* def_ns, double def_d, double def_z,
                          float dr, float dg, float db, float da) {
      L.ns = def_ns; pnh_.param(base + "/ns", L.ns, L.ns);
      L.sphere_diam = def_d; pnh_.param(base + "/sphere_diam", L.sphere_diam, L.sphere_diam);
      L.z_offset    = def_z; pnh_.param(base + "/z_offset",    L.z_offset,    L.z_offset);
      std::vector<double> rgba;
      if (pnh_.getParam(base + "/rgba", rgba)) parseRGBA(rgba, L.r, L.g, L.b, L.a);
      else { L.r = dr; L.g = dg; L.b = db; L.a = da; }
    };

    // Defaults = your requested colors
    read_layer("visualization/all",       cfg_.all,       "landmarks_all",       0.18, 0.05, 1.f, 0.5f, 0.f, 1.f); // orange
    read_layer("visualization/confirmed", cfg_.confirmed, "landmarks_confirmed", 0.18, 0.05, 0.f, 0.f, 1.f, 1.f);  // blue
    read_layer("visualization/locked",    cfg_.locked,    "landmarks_locked",    0.18, 0.05, 0.f, 1.f, 1.f, 1.f);  // cyan
    read_layer("visualization/unmatched", cfg_.unmatched, "landmarks_unmatched", 0.16, 0.05, 1.f, 0.f, 0.f, 1.f);  // red
  }

  // Pick a usable frame (prefer non-empty sources)
  std::string chooseFrame() const {
    if (last_conf_ && !last_conf_->poses.empty()) return last_conf_->header.frame_id;
    if (last_all_  && !last_all_->poses.empty())  return last_all_->header.frame_id;
    if (last_lock_ && !last_lock_->poses.empty()) return last_lock_->header.frame_id;
    if (last_unm_  && !last_unm_->poses.empty())  return last_unm_->header.frame_id;
    return "odom";
  }

  void publishCombined() {
    const std::string frame = chooseFrame();
    visualization_msgs::MarkerArray arr;
    std::vector<geometry_msgs::Point> pts;

    // ALL
    if (last_all_ && !last_all_->poses.empty()) {
      auto m = makeSphereList(0, cfg_.all.ns, frame, cfg_.all.sphere_diam, cfg_.all.r, cfg_.all.g, cfg_.all.b, cfg_.all.a);
      posesToPoints(last_all_, cfg_.all.z_offset, pts); m.points.swap(pts);
      arr.markers.push_back(m);
    } else {
      arr.markers.push_back(makeDelete(0, cfg_.all.ns, frame));
    }

    // CONFIRMED
    if (last_conf_ && !last_conf_->poses.empty()) {
      auto m = makeSphereList(1, cfg_.confirmed.ns, frame, cfg_.confirmed.sphere_diam, cfg_.confirmed.r, cfg_.confirmed.g, cfg_.confirmed.b, cfg_.confirmed.a);
      posesToPoints(last_conf_, cfg_.confirmed.z_offset, pts); m.points.swap(pts);
      arr.markers.push_back(m);
    } else {
      arr.markers.push_back(makeDelete(1, cfg_.confirmed.ns, frame));
    }

    // LOCKED
    if (last_lock_ && !last_lock_->poses.empty()) {
      auto m = makeSphereList(2, cfg_.locked.ns, frame, cfg_.locked.sphere_diam, cfg_.locked.r, cfg_.locked.g, cfg_.locked.b, cfg_.locked.a);
      posesToPoints(last_lock_, cfg_.locked.z_offset, pts); m.points.swap(pts);
      arr.markers.push_back(m);
    } else {
      arr.markers.push_back(makeDelete(2, cfg_.locked.ns, frame));
    }

    // UNMATCHED
    if (last_unm_ && !last_unm_->poses.empty()) {
      auto m = makeSphereList(3, cfg_.unmatched.ns, frame, cfg_.unmatched.sphere_diam, cfg_.unmatched.r, cfg_.unmatched.g, cfg_.unmatched.b, cfg_.unmatched.a);
      posesToPoints(last_unm_, cfg_.unmatched.z_offset, pts); m.points.swap(pts);
      arr.markers.push_back(m);
    } else {
      arr.markers.push_back(makeDelete(3, cfg_.unmatched.ns, frame));
    }

    pub_.publish(arr);
  }

  // Callbacks: store & publish
  void cbAll (const geometry_msgs::PoseArray::ConstPtr& msg) { last_all_  = msg; publishCombined(); }
  void cbConf(const geometry_msgs::PoseArray::ConstPtr& msg) { last_conf_ = msg; publishCombined(); }
  void cbLock(const geometry_msgs::PoseArray::ConstPtr& msg) { last_lock_ = msg; publishCombined(); }
  void cbUnm (const geometry_msgs::PoseArray::ConstPtr& msg) { last_unm_  = msg; publishCombined(); }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastslam2_viz");
  ros::NodeHandle nh, pnh("~");
  FastSLAM2Viz node(nh, pnh);
  ros::spin();
  return 0;
}
