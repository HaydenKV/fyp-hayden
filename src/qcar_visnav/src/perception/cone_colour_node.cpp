// cone_colour_node.cpp
//
// Adds color labels (blue/yellow) + confidence to LiDAR-detected cones by
// projecting each cone into one of the CSI RGB images (front/left/right),
// classifying an ROI in HSV, and picking the best camera per cone.
// Publishes:
//   • /cones_colored  (qcar_visnav/ConeArray)  — same frame/range/bearing, with color + conf
//   • /cone_colour/debug_image (sensor_msgs/Image) — annotated FRONT RGB image (as before)
//   • /cone_colour/debug_front  (sensor_msgs/Image) — front camera annotated
//   • /cone_colour/debug_left   (sensor_msgs/Image) — left  camera annotated
//   • /cone_colour/debug_right  (sensor_msgs/Image) — right camera annotated
//
// Subscribes:
//   • /tracked_cones (qcar_visnav/ConeArray)   [or /cones — configurable]
//   • /csi_front/image_raw (sensor_msgs/Image)
//   • /csi_left/image_raw  (sensor_msgs/Image)
//   • /csi_right/image_raw (sensor_msgs/Image)
//   • /csi_*/camera_info   (sensor_msgs/CameraInfo)
//
// TF required (one per camera):
//   lidar_frame -> csi_front_optical
//   lidar_frame -> csi_left_optical
//   lidar_frame -> csi_right_optical
//
// Params (private ns ~cone_colour):
//   detections_topic      [string]  default "/tracked_cones"
//   output_topic          [string]  default "/cones_colored"
//   rgb_topic_front       [string]  default "/csi_front/image_raw"
//   rgb_topic_left        [string]  default "/csi_left/image_raw"
//   rgb_topic_right       [string]  default "/csi_right/image_raw"
//   cam_info_front        [string]  default "/csi_front/camera_info"
//   cam_info_left         [string]  default "/csi_left/camera_info"
//   cam_info_right        [string]  default "/csi_right/camera_info"
//   camera_frame_front    [string]  default "csi_front_optical"
//   camera_frame_left     [string]  default "csi_left_optical"
//   camera_frame_right    [string]  default "csi_right_optical"
//   max_image_age_s       [double]  default 0.25
//   cone_width_m          [double]  default 0.25
//   min_roi_px            [int]     default 16
//   max_roi_px            [int]     default 96
//   min_pixels_for_conf   [int]     default 120
//   conf_floor            [double]  default 0.10
//   hsv_* thresholds      [ints]    see defaults below
//   debug                 [int]     default 1
//   debug_publish_image   [bool]    default true
//
// Color ids: 0=unknown, 1=blue, 2=yellow

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <qcar_visnav/ConeArray.h>
#include <qcar_visnav/Cone.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <cmath>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct Params {
  // I/O
  std::string detections_topic{"/tracked_cones"};
  std::string output_topic{"/cones_colored"};

  // Camera topics/frames
  std::string rgb_topic_front{"/csi_front/image_raw"};
  std::string rgb_topic_left{"/csi_left/image_raw"};
  std::string rgb_topic_right{"/csi_right/image_raw"};
  std::string cam_info_front{"/csi_front/camera_info"};
  std::string cam_info_left{"/csi_left/camera_info"};
  std::string cam_info_right{"/csi_right/camera_info"};
  std::string camera_frame_front{"csi_front_optical"};
  std::string camera_frame_left{"csi_left_optical"};
  std::string camera_frame_right{"csi_right_optical"};

  // Constraints
  double max_image_age_s{0.25};
  double cone_width_m{0.25};
  int    min_roi_px{16};
  int    max_roi_px{96};
  int    min_pixels_for_conf{120};
  double conf_floor{0.10};

  // HSV thresholds
  cv::Scalar hsv_blue_low{100,150,50};
  cv::Scalar hsv_blue_high{130,255,255};
  cv::Scalar hsv_yellow_low{18,80,80};
  cv::Scalar hsv_yellow_high{35,255,255};

  // Debug
  int  debug{1};
  bool debug_publish_image{true};
} P;

struct CamState {
  // Params
  std::string name;          // "front" / "left" / "right"
  std::string rgb_topic;
  std::string cam_info_topic;
  std::string camera_frame;

  // Intrinsics
  bool   have_K{false};
  double fx{0}, fy{0}, cx{0}, cy{0};
  int    img_w{0}, img_h{0};

  // Latest image
  cv::Mat rgb;
  ros::Time stamp_rgb;
  bool have_rgb{false};

  // For debug overlays
  cv::Mat annotated;
  ros::Publisher pub_dbg;
};

class ConeColourNode {
public:
  ConeColourNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_buffer_(ros::Duration(10.0)),
      tf_listener_(tf_buffer_) {

    // Load params
    pnh.param("detections_topic", P.detections_topic, P.detections_topic);
    pnh.param("output_topic",     P.output_topic,     P.output_topic);

    pnh.param("rgb_topic_front",  P.rgb_topic_front,  P.rgb_topic_front);
    pnh.param("rgb_topic_left",   P.rgb_topic_left,   P.rgb_topic_left);
    pnh.param("rgb_topic_right",  P.rgb_topic_right,  P.rgb_topic_right);

    pnh.param("cam_info_front",   P.cam_info_front,   P.cam_info_front);
    pnh.param("cam_info_left",    P.cam_info_left,    P.cam_info_left);
    pnh.param("cam_info_right",   P.cam_info_right,   P.cam_info_right);

    pnh.param("camera_frame_front", P.camera_frame_front, P.camera_frame_front);
    pnh.param("camera_frame_left",  P.camera_frame_left,  P.camera_frame_left);
    pnh.param("camera_frame_right", P.camera_frame_right, P.camera_frame_right);

    pnh.param("max_image_age_s",  P.max_image_age_s,  P.max_image_age_s);
    pnh.param("cone_width_m",     P.cone_width_m,     P.cone_width_m);
    pnh.param("min_roi_px",       P.min_roi_px,       P.min_roi_px);
    pnh.param("max_roi_px",       P.max_roi_px,       P.max_roi_px);
    pnh.param("min_pixels_for_conf", P.min_pixels_for_conf, P.min_pixels_for_conf);
    pnh.param("conf_floor",       P.conf_floor,       P.conf_floor);
    pnh.param("debug",            P.debug,            P.debug);
    pnh.param("debug_publish_image", P.debug_publish_image, P.debug_publish_image);

    // HSV overrides
    std::vector<int> t;
    if (pnh.getParam("hsv_blue_low", t)    && t.size()==3) P.hsv_blue_low    = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_blue_high", t)   && t.size()==3) P.hsv_blue_high   = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_yellow_low", t)  && t.size()==3) P.hsv_yellow_low  = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_yellow_high", t) && t.size()==3) P.hsv_yellow_high = cv::Scalar(t[0],t[1],t[2]);

    // Build camera table
    cams_.resize(3);
    cams_[0].name = "front"; cams_[0].rgb_topic = P.rgb_topic_front; cams_[0].cam_info_topic = P.cam_info_front; cams_[0].camera_frame = P.camera_frame_front;
    cams_[1].name = "left";  cams_[1].rgb_topic = P.rgb_topic_left;  cams_[1].cam_info_topic = P.cam_info_left;  cams_[1].camera_frame = P.camera_frame_left;
    cams_[2].name = "right"; cams_[2].rgb_topic = P.rgb_topic_right; cams_[2].cam_info_topic = P.cam_info_right; cams_[2].camera_frame = P.camera_frame_right;

    // Subscriptions (one per camera)
    for (size_t i=0;i<cams_.size();++i) {
      sub_rgb_.push_back(nh.subscribe<sensor_msgs::Image>(cams_[i].rgb_topic, 1,
                        boost::bind(&ConeColourNode::rgbCb, this, _1, i)));
      sub_cinfo_.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cams_[i].cam_info_topic, 1,
                        boost::bind(&ConeColourNode::camInfoCb, this, _1, i)));
    }

    // Detections
    sub_cones_ = nh.subscribe(P.detections_topic, 1, &ConeColourNode::conesCb, this);

    // Output
    pub_out_   = nh.advertise<qcar_visnav::ConeArray>(P.output_topic, 1, false);

    // Debug image pubs
    if (P.debug_publish_image) {
      pub_dbg_main_ = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_image", 1, false); // legacy main (front)
      cams_[0].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_front", 1, false);
      cams_[1].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_left",  1, false);
      cams_[2].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_right", 1, false);
    }

    ROS_INFO("[cone_colour] subs: det='%s' cams: F('%s','%s'), L('%s','%s'), R('%s','%s') -> out='%s'%s",
             P.detections_topic.c_str(),
             cams_[0].rgb_topic.c_str(), cams_[0].cam_info_topic.c_str(),
             cams_[1].rgb_topic.c_str(), cams_[1].cam_info_topic.c_str(),
             cams_[2].rgb_topic.c_str(), cams_[2].cam_info_topic.c_str(),
             P.output_topic.c_str(),
             P.debug_publish_image ? " + debug topics" : "");
  }

private:
  // State
  std::mutex mtx_;
  std::vector<CamState> cams_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS IO
  ros::Subscriber sub_cones_;
  std::vector<ros::Subscriber> sub_rgb_;
  std::vector<ros::Subscriber> sub_cinfo_;
  ros::Publisher  pub_out_;

  // Debug
  ros::Publisher  pub_dbg_main_;

  // --- Callbacks ---
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg, size_t i) {
    auto& C = cams_[i];
    C.fx = msg->K[0]; C.fy = msg->K[4];
    C.cx = msg->K[2]; C.cy = msg->K[5];
    C.img_w = msg->width; C.img_h = msg->height;
    C.have_K = (C.fx>0 && C.fy>0 && C.img_w>0 && C.img_h>0);
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg, size_t i) {
    try {
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
      std::lock_guard<std::mutex> lk(mtx_);
      cams_[i].rgb = cvp->image.clone();
      cams_[i].stamp_rgb = msg->header.stamp;
      cams_[i].have_rgb = true;
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] cv_bridge RGB (%s): %s", cams_[i].name.c_str(), e.what());
    }
  }

  void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
    // quick guard: need at least front K + image to proceed (others optional)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!(cams_[0].have_K && cams_[0].have_rgb)) {
        ROS_WARN_THROTTLE(1.0, "[cone_colour] waiting for FRONT camera and info...");
        return;
      }
    }

    // Copy current frames to local to minimize lock time
    std::vector<CamState> C;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      C = cams_; // shallow copy of cv::Mat refs is fine, but we cloned on receive; ok to copy
    }

    const ros::Time det_stamp = msg->header.stamp;

    // Age check (warn only, per camera)
    for (auto& cam : C) {
      if (!cam.have_rgb) continue;
      const double dt = (det_stamp - cam.stamp_rgb).toSec();
      if (std::fabs(dt) > P.max_image_age_s) {
        ROS_WARN_THROTTLE(1.0, "[cone_colour] RGB age Δt=%.3fs vs detections for %s.", dt, cam.name.c_str());
      }
      // Prepare/clear annotated buffers if someone is listening
      if (P.debug_publish_image && cam.pub_dbg && cam.pub_dbg.getNumSubscribers() > 0) {
        cam.annotated = cam.rgb.clone();
      } else {
        cam.annotated.release();
      }
    }

    // Also prep legacy main debug (front)
    cv::Mat annotated_main;
    if (P.debug_publish_image && pub_dbg_main_.getNumSubscribers() > 0 && C[0].have_rgb) {
      annotated_main = C[0].rgb.clone();
    }

    // Prepare output + fill in-place
    qcar_visnav::ConeArray out = *msg;

    for (auto& cone : out.cones) {
      classifyWithBestCamera(cone, msg->header.frame_id, det_stamp, C, annotated_main);
    }

    // Publish cones
    pub_out_.publish(out);

    // Publish per-camera debug images (only if annotated exists)
    if (P.debug_publish_image) {
      for (const auto& cam : C) {
        if (!cam.annotated.empty() && cam.pub_dbg) {
          std_msgs::Header hdr;
          hdr.stamp = cam.stamp_rgb;
          hdr.frame_id = cam.camera_frame;
          sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(hdr, "bgr8", cam.annotated).toImageMsg();
          cam.pub_dbg.publish(msg_img);
        }
      }
      // Legacy main (front)
      if (!annotated_main.empty()) {
        std_msgs::Header hdr;
        hdr.stamp = C[0].stamp_rgb;
        hdr.frame_id = C[0].camera_frame;
        sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(hdr, "bgr8", annotated_main).toImageMsg();
        pub_dbg_main_.publish(msg_img);
      }
    }

    // Push back any debug state into member cams_ (for next frame continuity)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      // Only need to keep images; annotated were per-frame locals
      for (size_t i=0;i<cams_.size();++i) {
        cams_[i].have_rgb = C[i].have_rgb;
        cams_[i].stamp_rgb = C[i].stamp_rgb;
        cams_[i].rgb = C[i].rgb; // already clone on input
      }
    }
  }

  // --- Helpers ---
  static Eigen::Matrix3d rotFromTF(const geometry_msgs::TransformStamped& tf) {
    const auto& q = tf.transform.rotation;
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    Eigen::Quaterniond qe(qq.getW(), qq.getX(), qq.getY(), qq.getZ());
    return qe.toRotationMatrix();
  }

  static void polarToXY(double r, double th, double& x, double& y) {
    x = r * std::cos(th);
    y = r * std::sin(th);
  }

  bool transformLidarToCam(const std::string& cam_frame,
                           const std::string& lidar_frame,
                           const ros::Time& stamp,
                           const Eigen::Vector3d& p_lidar,
                           Eigen::Vector3d& p_cam) {
    try {
      geometry_msgs::TransformStamped T =
        tf_buffer_.lookupTransform(cam_frame, lidar_frame, stamp, ros::Duration(0.05));
      Eigen::Matrix3d R = rotFromTF(T);
      Eigen::Vector3d t(T.transform.translation.x,
                        T.transform.translation.y,
                        T.transform.translation.z);
      p_cam = R * p_lidar + t;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] TF %s->%s: %s",
                        lidar_frame.c_str(), cam_frame.c_str(), ex.what());
      return false;
    }
  }

  static bool projectCam(const CamState& C, const Eigen::Vector3d& p_cam, int& u, int& v) {
    const double X = p_cam.x(), Y = p_cam.y(), Z = p_cam.z();
    if (!(Z > 0.05)) return false;
    u = static_cast<int>(C.fx * (X / Z) + C.cx);
    v = static_cast<int>(C.fy * (Y / Z) + C.cy);
    return (u >= 0 && u < C.img_w && v >= 0 && v < C.img_h);
  }

  int roiHalfSizePx(const CamState& C, double Z) const {
    if (Z <= 0.05) Z = 0.05;
    double w_px = C.fx * P.cone_width_m / Z;
    int s = static_cast<int>(0.6 * w_px * 0.5);
    s = std::max(P.min_roi_px, std::min(P.max_roi_px, s));
    return s;
  }

  static void clampRect(int& x0, int& y0, int& x1, int& y1, int W, int H) {
    x0 = std::max(0, std::min(W-1, x0));
    y0 = std::max(0, std::min(H-1, y0));
    x1 = std::max(0, std::min(W-1, x1));
    y1 = std::max(0, std::min(H-1, y1));
    if (x1 < x0) std::swap(x0, x1);
    if (y1 < y0) std::swap(y0, y1);
  }

  static double ratioNonZero(const cv::Mat& mask) {
    const int total = mask.rows * mask.cols;
    if (total <= 0) return 0.0;
    return static_cast<double>(cv::countNonZero(mask)) / static_cast<double>(total);
  }

  void drawAnno(cv::Mat& img, int u, int v, int x0, int y0, int x1, int y1,
                int color_id, double conf, double Z, const std::string& tag) {
    if (img.empty()) return;

    // BGR colours for boxes
    cv::Scalar col(200,200,200);            // unknown -> grey
    if (color_id == 1) col = cv::Scalar(255, 0,   0  ); // blue
    else if (color_id == 2) col = cv::Scalar(0,   255,255); // yellow

    cv::rectangle(img, cv::Rect(cv::Point(x0,y0), cv::Point(x1,y1)), col, 2);
    cv::circle(img, cv::Point(u,v), 4, col, -1);

    char text[96];
    if (color_id == 1)  std::snprintf(text, sizeof(text), "[%s] B %.2f  Z=%.2f", tag.c_str(), conf, Z);
    else if (color_id == 2) std::snprintf(text, sizeof(text), "[%s] Y %.2f  Z=%.2f", tag.c_str(), conf, Z);
    else                 std::snprintf(text, sizeof(text), "[%s] ? 0.00  Z=%.2f", tag.c_str(), Z);

    int baseline=0;
    cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int tx = std::max(0, std::min(img.cols - ts.width - 2, u + 6));
    int ty = std::max(ts.height + 2, std::min(img.rows - 2, v - 6));
    cv::putText(img, text, cv::Point(tx,ty), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 2);
  }

  struct CamResult {
    int cam_idx{-1};
    int color{0};
    double conf{0.0};
    int u{0}, v{0}, x0{0}, y0{0}, x1{0}, y1{0};
    double Z{0.0};
  };

  // Classify with all cameras (that have data) and pick the best result.
  void classifyWithBestCamera(qcar_visnav::Cone& c,
                              const std::string& lidar_frame,
                              const ros::Time& stamp,
                              std::vector<CamState>& C,
                              cv::Mat& annotated_main_front)
  {
    double xL=0, yL=0; polarToXY(c.range, c.bearing, xL, yL);
    Eigen::Vector3d p_lidar(xL, yL, 0.0);

    CamResult best;

    for (int i=0; i<(int)C.size(); ++i) {
      auto& cam = C[i];
      if (!cam.have_K || !cam.have_rgb) continue;

      // LiDAR -> this camera frame
      Eigen::Vector3d p_cam;
      if (!transformLidarToCam(cam.camera_frame, lidar_frame, stamp, p_lidar, p_cam)) {
        continue;
      }

      // project
      int u=0, v=0;
      if (!projectCam(cam, p_cam, u, v)) continue;

      // ROI
      const double Z = p_cam.z();
      const int s = roiHalfSizePx(cam, Z);
      int x0 = u - s, y0 = v - s, x1 = u + s, y1 = v + s;
      clampRect(x0, y0, x1, y1, cam.img_w, cam.img_h);
      const int W = x1 - x0 + 1, H = y1 - y0 + 1;
      if (W*H < P.min_pixels_for_conf) continue;

      // HSV classify
      cv::Mat roi_bgr = cam.rgb(cv::Rect(x0, y0, W, H));
      cv::Mat roi_hsv; cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

      cv::Mat mask_blue, mask_yel;
      static const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3));
      cv::inRange(roi_hsv, P.hsv_blue_low,   P.hsv_blue_high,   mask_blue);
      cv::inRange(roi_hsv, P.hsv_yellow_low, P.hsv_yellow_high, mask_yel);
      cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask_yel,  mask_yel,  cv::MORPH_OPEN, kernel);

      const double r_blue = ratioNonZero(mask_blue);
      const double r_yel  = ratioNonZero(mask_yel);

      int color_id = 0; double conf = 0.0;
      if (r_blue > r_yel) { color_id = 1; conf = r_blue; }
      else                { color_id = 2; conf = r_yel;  }
      if (conf < P.conf_floor) { color_id = 0; conf = 0.0; }

      // score = confidence × ROI area (mild bias to bigger, better-sampled ROIs)
      const double score = conf * (double)(W*H);

      if (score > (best.conf * (double)((best.x1-best.x0+1)*(best.y1-best.y0+1)))) {
        best.cam_idx = i;
        best.color = color_id;
        best.conf = conf;
        best.u=u; best.v=v; best.x0=x0; best.y0=y0; best.x1=x1; best.y1=y1;
        best.Z = Z;
      }
    }

    // Fill output + draw on appropriate debug images
    if (best.cam_idx >= 0) {
      c.color = best.color;
      c.color_conf = best.conf;

      // Per-camera overlay (only on the camera that "won" this cone)
      auto& cam = C[best.cam_idx];
      if (!cam.annotated.empty()) {
        drawAnno(cam.annotated, best.u, best.v, best.x0, best.y0, best.x1, best.y1,
                 best.color, best.conf, best.Z, cam.name);
      }

      // Legacy main (front) overlay — keep behavior stable: only draw if winner is FRONT
      if (!annotated_main_front.empty() && best.cam_idx == 0) {
        drawAnno(annotated_main_front, best.u, best.v, best.x0, best.y0, best.x1, best.y1,
                 best.color, best.conf, best.Z, "front");
      }

      if (P.debug) {
        ROS_INFO_THROTTLE(0.5,
          "[cone_colour] cam=%s  r=%.2f brg=%.2f -> (u,v)=(%d,%d) Z=%.2f ROI=%dx%d -> color=%d conf=%.2f",
          C[best.cam_idx].name.c_str(), c.range, c.bearing, best.u, best.v, best.Z,
          best.x1-best.x0+1, best.y1-best.y0+1, c.color, c.color_conf);
      }
    } else {
      c.color = 0; c.color_conf = 0.0;
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "cone_colour");
  ros::NodeHandle nh, pnh("~");
  ConeColourNode node(nh, pnh);
  ros::spin();
  return 0;
}
