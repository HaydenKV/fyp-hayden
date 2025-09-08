#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

struct VizCfg {
  // Inputs
  std::string particles_pose_topic{"/slam/particles_pose"};
  std::string landmarks_pose_confirmed_topic{"/slam/landmarks_pose_confirmed"};
  std::string landmarks_pose_all_topic{"/slam/landmarks_pose"};
  std::string unmatched_pose_topic{"/slam/unmatched_pose"};
  std::string births_pose_topic{"/slam/births_pose"}; // optional / not used by SLAM now

  // Outputs (RViz topics)
  std::string particles_marker_topic{"/slam/particles"};
  std::string landmarks_marker_topic{"/slam/map_markers"}; // final topic used by RViz

  // Behavior
  bool show_confirmed_only{true};
  bool include_births_if_empty{true};
  bool use_latest_tf{true};

  // Styling
  double particle_len{0.30};
  double particle_width{0.05};
  double particle_height{0.05};
  double landmark_scale{0.18};
  double unmatched_scale{0.16};
  double z_offset{0.05};
  double particle_alpha{0.9};
  double landmark_alpha{1.0};
  double unmatched_alpha{1.0};
} Cfg;

class FastSLAM2Viz {
public:
  FastSLAM2Viz(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // Params
    pnh_.param("particles_pose_topic", Cfg.particles_pose_topic, Cfg.particles_pose_topic);
    pnh_.param("landmarks_pose_confirmed_topic", Cfg.landmarks_pose_confirmed_topic, Cfg.landmarks_pose_confirmed_topic);
    pnh_.param("landmarks_pose_all_topic", Cfg.landmarks_pose_all_topic, Cfg.landmarks_pose_all_topic);
    pnh_.param("unmatched_pose_topic", Cfg.unmatched_pose_topic, Cfg.unmatched_pose_topic);
    pnh_.param("births_pose_topic", Cfg.births_pose_topic, Cfg.births_pose_topic);
    pnh_.param("particles_marker_topic", Cfg.particles_marker_topic, Cfg.particles_marker_topic);
    pnh_.param("landmarks_marker_topic", Cfg.landmarks_marker_topic, Cfg.landmarks_marker_topic);

    pnh_.param("show_confirmed_only", Cfg.show_confirmed_only, Cfg.show_confirmed_only);
    pnh_.param("include_births_if_empty", Cfg.include_births_if_empty, Cfg.include_births_if_empty);
    pnh_.param("use_latest_tf", Cfg.use_latest_tf, Cfg.use_latest_tf);

    pnh_.param("particle_len", Cfg.particle_len, Cfg.particle_len);
    pnh_.param("particle_width", Cfg.particle_width, Cfg.particle_width);
    pnh_.param("particle_height", Cfg.particle_height, Cfg.particle_height);
    pnh_.param("landmark_scale", Cfg.landmark_scale, Cfg.landmark_scale);
    pnh_.param("unmatched_scale", Cfg.unmatched_scale, Cfg.unmatched_scale);
    pnh_.param("z_offset", Cfg.z_offset, Cfg.z_offset);
    pnh_.param("particle_alpha", Cfg.particle_alpha, Cfg.particle_alpha);
    pnh_.param("landmark_alpha", Cfg.landmark_alpha, Cfg.landmark_alpha);
    pnh_.param("unmatched_alpha", Cfg.unmatched_alpha, Cfg.unmatched_alpha);

    pub_particles_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(Cfg.particles_marker_topic, 1);
    pub_landmarks_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(Cfg.landmarks_marker_topic, 1);

    sub_particles_ = nh_.subscribe(Cfg.particles_pose_topic, 1, &FastSLAM2Viz::cbParticles, this);
    sub_lms_conf_  = nh_.subscribe(Cfg.landmarks_pose_confirmed_topic, 1, &FastSLAM2Viz::cbLandmarksConfirmed, this);
    sub_lms_all_   = nh_.subscribe(Cfg.landmarks_pose_all_topic, 1, &FastSLAM2Viz::cbLandmarksAll, this);
    sub_unmatched_ = nh_.subscribe(Cfg.unmatched_pose_topic, 1, &FastSLAM2Viz::cbUnmatched, this);
    sub_births_    = nh_.subscribe(Cfg.births_pose_topic, 1, &FastSLAM2Viz::cbBirths, this);

    ROS_INFO("[FastSLAM2Viz] Inputs: %s, %s, %s, %s | Outputs: %s, %s",
      Cfg.particles_pose_topic.c_str(),
      Cfg.landmarks_pose_confirmed_topic.c_str(),
      Cfg.landmarks_pose_all_topic.c_str(),
      Cfg.unmatched_pose_topic.c_str(),
      Cfg.particles_marker_topic.c_str(),
      Cfg.landmarks_marker_topic.c_str());
  }

private:
  // cache last received arrays
  geometry_msgs::PoseArray::ConstPtr last_conf_;
  geometry_msgs::PoseArray::ConstPtr last_all_;
  geometry_msgs::PoseArray::ConstPtr last_unmatched_;
  geometry_msgs::PoseArray::ConstPtr last_births_;

  void cbParticles(const geometry_msgs::PoseArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray arr;

    if (msg->poses.empty()) {
      // Publish a wipe to clear particle arrows
      visualization_msgs::Marker wipe;
      wipe.header.frame_id = msg->header.frame_id;
      wipe.header.stamp    = Cfg.use_latest_tf ? ros::Time(0) : msg->header.stamp;
      wipe.ns = "particles";
      wipe.id = 0;
      wipe.action = visualization_msgs::Marker::DELETEALL;
      arr.markers.push_back(wipe);
      pub_particles_markers_.publish(arr);
      return;
    }

    // Wipe existing
    wipeNs(arr, "particles", msg->header.frame_id, msg->header.stamp);

    int id = 0;
    for (const auto& pose : msg->poses) {
      visualization_msgs::Marker m;
      fillCommon(m, "particles", id++, msg->header.frame_id, msg->header.stamp);
      m.type = visualization_msgs::Marker::ARROW;
      m.pose = pose;
      m.pose.position.z = Cfg.z_offset;
      m.scale.x = Cfg.particle_len;
      m.scale.y = Cfg.particle_width;
      m.scale.z = Cfg.particle_height;
      m.color.a = Cfg.particle_alpha;
      m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0;
      arr.markers.push_back(m);
    }
    pub_particles_markers_.publish(arr);
  }

  void cbLandmarksConfirmed(const geometry_msgs::PoseArray::ConstPtr& msg) {
    last_conf_ = msg;
    publishLandmarks();
  }
  void cbLandmarksAll(const geometry_msgs::PoseArray::ConstPtr& msg) {
    last_all_ = msg;
    publishLandmarks();
  }
  void cbUnmatched(const geometry_msgs::PoseArray::ConstPtr& msg) {
    last_unmatched_ = msg;
    publishLandmarks();
  }
  void cbBirths(const geometry_msgs::PoseArray::ConstPtr& msg) {
    last_births_ = msg;
    publishLandmarks();
  }

  void publishLandmarks() {
    // Choose source: prefer confirmed only if there are any; otherwise fall back to all
    const bool have_conf = (last_conf_ && !last_conf_->poses.empty());
    const bool have_all  = (last_all_  && !last_all_->poses.empty());
    const auto& src = (Cfg.show_confirmed_only && have_conf) ? last_conf_
                  : (have_all ? last_all_ : last_conf_);

    const bool have_unmatched = (last_unmatched_ && !last_unmatched_->poses.empty());
    const bool have_births    = (last_births_ && !last_births_->poses.empty());

    if ((!src || src->poses.empty()) && !have_unmatched && !have_births) {
      // Nothing to draw — avoid sending a DELETEALL-only frame
      return;
    }

    // Use source header for timing; if none, prefer unmatched header
    std_msgs::Header hdr;
    if (src && !src->poses.empty()) hdr = src->header;
    else if (have_unmatched)        hdr = last_unmatched_->header;
    else                            hdr = last_births_->header;

    visualization_msgs::MarkerArray arr;
    wipeNs(arr, "landmarks", hdr.frame_id, hdr.stamp);

    // Landmarks (blue)
    if (src && !src->poses.empty()) {
      visualization_msgs::Marker m;
      fillCommon(m, "landmarks", 0, hdr.frame_id, hdr.stamp);
      m.type = visualization_msgs::Marker::SPHERE_LIST;
      m.scale.x = Cfg.landmark_scale;
      m.scale.y = Cfg.landmark_scale;
      m.scale.z = Cfg.landmark_scale;
      m.color.a = Cfg.landmark_alpha;
      m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;

      m.points.reserve(src->poses.size());
      for (const auto& pose : src->poses) {
        geometry_msgs::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = Cfg.z_offset;
        m.points.push_back(p);
      }
      arr.markers.push_back(m);
    }

    // Unmatched (red)
    if (have_unmatched) {
      visualization_msgs::Marker u;
      fillCommon(u, "landmarks_unmatched", 1, hdr.frame_id, hdr.stamp);
      u.type = visualization_msgs::Marker::SPHERE_LIST;
      u.scale.x = Cfg.unmatched_scale;
      u.scale.y = Cfg.unmatched_scale;
      u.scale.z = Cfg.unmatched_scale;
      u.color.a = Cfg.unmatched_alpha;
      u.color.r = 1.0; u.color.g = 0.0; u.color.b = 0.0;

      u.points.reserve(last_unmatched_->poses.size());
      for (const auto& pose : last_unmatched_->poses) {
        geometry_msgs::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = Cfg.z_offset;
        u.points.push_back(p);
      }
      arr.markers.push_back(u);
    }

    // Optional: births (lighter blue) if we ever repopulate that topic
    if (Cfg.include_births_if_empty && (!src || src->poses.empty()) && have_births) {
      visualization_msgs::Marker b;
      fillCommon(b, "births", 2, hdr.frame_id, hdr.stamp);
      b.type = visualization_msgs::Marker::SPHERE_LIST;
      b.scale.x = Cfg.landmark_scale * 0.85;
      b.scale.y = Cfg.landmark_scale * 0.85;
      b.scale.z = Cfg.landmark_scale * 0.85;
      b.color.a = 0.6;
      b.color.r = 0.2; b.color.g = 0.2; b.color.b = 1.0;
      for (const auto& pose : last_births_->poses) {
        geometry_msgs::Point p;
        p.x = pose.position.x;
        p.y = pose.position.y;
        p.z = Cfg.z_offset;
        b.points.push_back(p);
      }
      arr.markers.push_back(b);
    }

    pub_landmarks_markers_.publish(arr);
  }

  void wipeNs(visualization_msgs::MarkerArray& arr,
              const std::string& ns,
              const std::string& frame_id,
              const ros::Time& src_stamp)
  {
    visualization_msgs::Marker wipe;
    wipe.header.frame_id = frame_id;
    wipe.header.stamp = Cfg.use_latest_tf ? ros::Time(0) : src_stamp;
    wipe.ns = ns;
    wipe.id = 0;
    wipe.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(wipe);
  }

  void fillCommon(visualization_msgs::Marker& m,
                  const std::string& ns,
                  int id,
                  const std::string& frame_id,
                  const ros::Time& src_stamp)
  {
    m.header.frame_id = frame_id;
    m.header.stamp    = Cfg.use_latest_tf ? ros::Time(0) : src_stamp;
    m.ns = ns;
    m.id = id;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
  }

private:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber sub_particles_;
  ros::Subscriber sub_lms_conf_;
  ros::Subscriber sub_lms_all_;
  ros::Subscriber sub_unmatched_;
  ros::Subscriber sub_births_;

  ros::Publisher pub_particles_markers_;
  ros::Publisher pub_landmarks_markers_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastslam2_viz_node");
  ros::NodeHandle nh, pnh("~");
  FastSLAM2Viz node(nh, pnh);
  ros::spin();
  return 0;
}
