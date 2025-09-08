#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

struct VizCfg {
  // Inputs
  std::string particles_pose_topic{"/slam/particles_pose"};
  std::string landmarks_pose_confirmed_topic{"/slam/landmarks_pose_confirmed"};
  std::string landmarks_pose_all_topic{"/slam/landmarks_pose"};
  std::string births_pose_topic{"/slam/births_pose"};

  // Outputs (legacy RViz topics)
  std::string particles_marker_topic{"/slam/particles"};
  std::string landmarks_marker_topic{"/slam/map_markers"};

  // Behavior
  bool show_confirmed_only{true};  // like the old node
  bool include_births_if_empty{true}; // helps early frames
  bool use_latest_tf{true};        // stamp markers at time(0)

  // Styling
  double particle_len{0.30};
  double particle_width{0.05};
  double particle_height{0.05};
  double landmark_scale{0.18};
  double z_offset{0.05};
  double particle_alpha{0.9};
  double landmark_alpha{1.0};
} Cfg;

class FastSLAM2Viz {
public:
  FastSLAM2Viz(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // Params
    pnh_.param("particles_pose_topic", Cfg.particles_pose_topic, Cfg.particles_pose_topic);
    pnh_.param("landmarks_pose_confirmed_topic", Cfg.landmarks_pose_confirmed_topic, Cfg.landmarks_pose_confirmed_topic);
    pnh_.param("landmarks_pose_all_topic", Cfg.landmarks_pose_all_topic, Cfg.landmarks_pose_all_topic);
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
    pnh_.param("z_offset", Cfg.z_offset, Cfg.z_offset);
    pnh_.param("particle_alpha", Cfg.particle_alpha, Cfg.particle_alpha);
    pnh_.param("landmark_alpha", Cfg.landmark_alpha, Cfg.landmark_alpha);

    pub_particles_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(Cfg.particles_marker_topic, 1);
    pub_landmarks_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(Cfg.landmarks_marker_topic, 1);

    sub_particles_ = nh_.subscribe(Cfg.particles_pose_topic, 1, &FastSLAM2Viz::cbParticles, this);
    sub_lms_conf_  = nh_.subscribe(Cfg.landmarks_pose_confirmed_topic, 1, &FastSLAM2Viz::cbLandmarksConfirmed, this);
    sub_lms_all_   = nh_.subscribe(Cfg.landmarks_pose_all_topic, 1, &FastSLAM2Viz::cbLandmarksAll, this);
    sub_births_    = nh_.subscribe(Cfg.births_pose_topic, 1, &FastSLAM2Viz::cbBirths, this);

    ROS_INFO("[FastSLAM2Viz] Inputs: %s, %s, %s, %s | Outputs: %s, %s",
      Cfg.particles_pose_topic.c_str(),
      Cfg.landmarks_pose_confirmed_topic.c_str(),
      Cfg.landmarks_pose_all_topic.c_str(),
      Cfg.births_pose_topic.c_str(),
      Cfg.particles_marker_topic.c_str(),
      Cfg.landmarks_marker_topic.c_str());
  }

private:
  // cache last received arrays
  geometry_msgs::PoseArray::ConstPtr last_conf_;
  geometry_msgs::PoseArray::ConstPtr last_all_;
  geometry_msgs::PoseArray::ConstPtr last_births_;

  void cbParticles(const geometry_msgs::PoseArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray arr;
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
  void cbBirths(const geometry_msgs::PoseArray::ConstPtr& msg) {
    last_births_ = msg;
    publishLandmarks();
  }

  void publishLandmarks() {
    const auto& src = (Cfg.show_confirmed_only && last_conf_) ? last_conf_ : last_all_;

    if (!src) return; // nothing yet

    visualization_msgs::MarkerArray arr;
    wipeNs(arr, "landmarks", src->header.frame_id, src->header.stamp);

    int id = 0;
    // Draw landmarks (blue)
    for (const auto& pose : src->poses) {
      visualization_msgs::Marker m;
      fillCommon(m, "landmarks", id++, src->header.frame_id, src->header.stamp);
      m.type = visualization_msgs::Marker::SPHERE;
      m.pose = pose;
      m.pose.position.z = Cfg.z_offset;
      m.scale.x = Cfg.landmark_scale;
      m.scale.y = Cfg.landmark_scale;
      m.scale.z = Cfg.landmark_scale;
      m.color.a = Cfg.landmark_alpha;
      m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0;
      arr.markers.push_back(m);
    }

    // Optionally, if confirmed/all empty but births exist, draw births (lighter blue)
    if (Cfg.include_births_if_empty && arr.markers.empty() && last_births_) {
      int idb = 0;
      for (const auto& pose : last_births_->poses) {
        visualization_msgs::Marker m;
        fillCommon(m, "landmarks", idb++, last_births_->header.frame_id, last_births_->header.stamp);
        m.type = visualization_msgs::Marker::SPHERE;
        m.pose = pose;
        m.pose.position.z = Cfg.z_offset;
        m.scale.x = Cfg.landmark_scale * 0.85;
        m.scale.y = Cfg.landmark_scale * 0.85;
        m.scale.z = Cfg.landmark_scale * 0.85;
        m.color.a = 0.6; // more transparent
        m.color.r = 0.2; m.color.g = 0.2; m.color.b = 1.0;
        arr.markers.push_back(m);
      }
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
