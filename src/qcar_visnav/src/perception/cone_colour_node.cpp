// cone_colour_node.cpp
//
// Purpose:
//   Adds color labels (blue/yellow) + confidence to LiDAR-tracked cones by
//   projecting each cone into THREE CSI RGB images (front/left/right),
//   classifying a small ROI in HSV in each camera, and picking the best camera
//   result per cone.
//
// Outputs:
//   • /cones_colored  (qcar_visnav/ConeArray) — same pose/range/bearing as input,
//                     with fields color (0=unknown,1=blue,2=yellow) + color_conf [0..1]
//   • /cone_colour/debug_image   (sensor_msgs/Image) — legacy FRONT annotated image
//   • /cone_colour/debug_front   (sensor_msgs/Image) — front camera annotated
//   • /cone_colour/debug_left    (sensor_msgs/Image) — left  camera annotated
//   • /cone_colour/debug_right   (sensor_msgs/Image) — right camera annotated
//
// Inputs:
//   • /tracked_cones (qcar_visnav/ConeArray)  — from the tracker
//   • /csi_front/image_raw, /csi_left/image_raw, /csi_right/image_raw
//   • /csi_*/camera_info — intrinsics K and image size
//
// TF requirements (per camera):
//   lidar_frame -> csi_front_optical
//   lidar_frame -> csi_left_optical
//   lidar_frame -> csi_right_optical
//
// Parameters (namespace ~cone_colour):
//   detections_topic        [string] default "/tracked_cones"
//   output_topic            [string] default "/cones_colored"
//   rgb_topic_*             [string] image topics
//   cam_info_*              [string] CameraInfo topics
//   camera_frame_*          [string] optical frame ids
//   max_image_age_s         [double] warn if image older than this vs detection stamp (default 0.25)
//   cone_width_m            [double] used to size ROI from range (default 0.25)
//   roi_scale               [double] 0..1 factor on ROI half-size (default 0.6)
//   min_roi_px, max_roi_px  [int] ROI half-size clamps (default 16/96)
//   min_pixels_for_conf     [int] ROI area guard (default 120)
//   conf_floor              [double] min coverage to accept color (default 0.10)
//   hsv_* thresholds        [ints]   HSV ranges for blue/yellow
//   hsv_open_ksize          [int]    morph open kernel size (odd, default 3)
//   min_Z, max_Z            [double] reject too-close/too-far Z (default 0.05/30.0)
//   prefer_center_weight    [double] [0..1] bonus for central ROIs (default 0.15)
//   tf_use_latest           [bool]   use TF at time 0 (latest) instead of exact stamp (default true)
//   lidar_frame_override    [string] if non-empty, use this as lidar frame id
//   debug                   [int]    >0 enables logs
//   debug_publish_image     [bool]   publish annotated debug images
//
// Notes:
//   - We annotate per-camera frames only if there are subscribers, to save CPU.

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
#include <algorithm>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct Params {
  // I/O topics
  std::string detections_topic{"/tracked_cones"};
  std::string output_topic{"/cones_colored"};

  // Per-camera topics + frames
  std::string rgb_topic_front{"/csi_front/image_raw"};
  std::string rgb_topic_left{ "/csi_left/image_raw"};
  std::string rgb_topic_right{"/csi_right/image_raw"};
  std::string cam_info_front{ "/csi_front/camera_info"};
  std::string cam_info_left{  "/csi_left/camera_info"};
  std::string cam_info_right{ "/csi_right/camera_info"};
  std::string camera_frame_front{"csi_front_optical"};
  std::string camera_frame_left{ "csi_left_optical"};
  std::string camera_frame_right{"csi_right_optical"};

  // Classifier sizing & gating
  double max_image_age_s{0.25};
  double cone_width_m{0.25};
  double roi_scale{0.6};           // NEW
  int    min_roi_px{16};
  int    max_roi_px{96};
  int    min_pixels_for_conf{120};
  double conf_floor{0.10};
  double min_Z{0.05};              // NEW
  double max_Z{30.0};              // NEW

  // HSV ranges (OpenCV HSV: H ∈ [0,179])
  cv::Scalar hsv_blue_low{100,150,50};
  cv::Scalar hsv_blue_high{130,255,255};
  cv::Scalar hsv_yellow_low{18,80,80};
  cv::Scalar hsv_yellow_high{35,255,255};
  int hsv_open_ksize{3};           // NEW

  // Camera arbitration
  double prefer_center_weight{0.15}; // NEW: [0..1] bonus toward center

  // TF behavior
  bool tf_use_latest{true};           // NEW: latest vs exact-time TF
  std::string lidar_frame_override{""}; // NEW

  // Debugging
  int  debug{1};
  bool debug_publish_image{true};
} P;

// Per-camera runtime state
struct CamState {
  // Identity
  std::string name;          // "front" / "left" / "right"
  std::string rgb_topic;
  std::string cam_info_topic;
  std::string camera_frame;

  // Intrinsics (from CameraInfo)
  bool   have_K{false};
  double fx{0}, fy{0}, cx{0}, cy{0};
  int    img_w{0}, img_h{0};

  // Latest image + stamp
  cv::Mat rgb;
  ros::Time stamp_rgb;
  bool have_rgb{false};

  // Debug overlays (published if subscribers exist)
  cv::Mat annotated;
  ros::Publisher pub_dbg;
};

class ConeColourNode {
public:
  ConeColourNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_buffer_(ros::Duration(10.0)),
      tf_listener_(tf_buffer_) {

    // --- Load config parameters (with defaults) ---
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
    pnh.param("roi_scale",        P.roi_scale,        P.roi_scale);
    pnh.param("min_roi_px",       P.min_roi_px,       P.min_roi_px);
    pnh.param("max_roi_px",       P.max_roi_px,       P.max_roi_px);
    pnh.param("min_pixels_for_conf", P.min_pixels_for_conf, P.min_pixels_for_conf);
    pnh.param("conf_floor",       P.conf_floor,       P.conf_floor);
    pnh.param("min_Z",            P.min_Z,            P.min_Z);
    pnh.param("max_Z",            P.max_Z,            P.max_Z);

    pnh.param("hsv_open_ksize",   P.hsv_open_ksize,   P.hsv_open_ksize);
    pnh.param("prefer_center_weight", P.prefer_center_weight, P.prefer_center_weight);

    pnh.param("tf_use_latest",    P.tf_use_latest,    P.tf_use_latest);
    pnh.param("lidar_frame_override", P.lidar_frame_override, P.lidar_frame_override);

    pnh.param("debug",            P.debug,            P.debug);
    pnh.param("debug_publish_image", P.debug_publish_image, P.debug_publish_image);

    // --- Optional HSV overrides from param server ---
    std::vector<int> t;
    if (pnh.getParam("hsv_blue_low", t)    && t.size()==3) P.hsv_blue_low    = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_blue_high", t)   && t.size()==3) P.hsv_blue_high   = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_yellow_low", t)  && t.size()==3) P.hsv_yellow_low  = cv::Scalar(t[0],t[1],t[2]);
    if (pnh.getParam("hsv_yellow_high", t) && t.size()==3) P.hsv_yellow_high = cv::Scalar(t[0],t[1],t[2]);

    // --- Build the camera table (fixed size: 3 cams) ---
    cams_.resize(3);
    cams_[0].name = "front"; cams_[0].rgb_topic = P.rgb_topic_front; cams_[0].cam_info_topic = P.cam_info_front; cams_[0].camera_frame = P.camera_frame_front;
    cams_[1].name = "left";  cams_[1].rgb_topic = P.rgb_topic_left;  cams_[1].cam_info_topic = P.cam_info_left;  cams_[1].camera_frame = P.camera_frame_left;
    cams_[2].name = "right"; cams_[2].rgb_topic = P.rgb_topic_right; cams_[2].cam_info_topic = P.cam_info_right; cams_[2].camera_frame = P.camera_frame_right;

    // --- Subscriptions: image + CameraInfo per camera ---
    for (size_t i=0;i<cams_.size();++i) {
      sub_rgb_.push_back(nh.subscribe<sensor_msgs::Image>(cams_[i].rgb_topic, 1,
                        boost::bind(&ConeColourNode::rgbCb, this, _1, i)));
      sub_cinfo_.push_back(nh.subscribe<sensor_msgs::CameraInfo>(cams_[i].cam_info_topic, 1,
                        boost::bind(&ConeColourNode::camInfoCb, this, _1, i)));
    }

    // Tracked cone stream from the tracker
    sub_cones_ = nh.subscribe(P.detections_topic, 1, &ConeColourNode::conesCb, this);

    // Output colored cones
    pub_out_   = nh.advertise<qcar_visnav::ConeArray>(P.output_topic, 1, false);

    // Optional per-camera debug images (published only if someone is subscribed)
    if (P.debug_publish_image) {
      pub_dbg_main_ = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_image", 1, false); // legacy: front
      cams_[0].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_front", 1, false);
      cams_[1].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_left",  1, false);
      cams_[2].pub_dbg = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_right", 1, false);
    }

    ROS_INFO("[cone_colour] subs: det='%s' cams: F('%s','%s'), L('%s','%s'), R('%s','%s') -> out='%s'%s (tf_use_latest=%s)",
             P.detections_topic.c_str(),
             cams_[0].rgb_topic.c_str(), cams_[0].cam_info_topic.c_str(),
             cams_[1].rgb_topic.c_str(), cams_[1].cam_info_topic.c_str(),
             cams_[2].rgb_topic.c_str(), cams_[2].cam_info_topic.c_str(),
             P.output_topic.c_str(),
             P.debug_publish_image ? " + debug topics" : "",
             P.tf_use_latest ? "true" : "false");
  }

private:
  // --- State & IO ---
  std::mutex mtx_;
  std::vector<CamState> cams_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_cones_;
  std::vector<ros::Subscriber> sub_rgb_;
  std::vector<ros::Subscriber> sub_cinfo_;
  ros::Publisher  pub_out_;

  // Debug (legacy front image)
  ros::Publisher  pub_dbg_main_;

  // --- Callbacks ---
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg, size_t i) {
    // Cache intrinsics + image size
    auto& C = cams_[i];
    C.fx = msg->K[0]; C.fy = msg->K[4];
    C.cx = msg->K[2]; C.cy = msg->K[5];
    C.img_w = msg->width; C.img_h = msg->height;
    C.have_K = (C.fx>0 && C.fy>0 && C.img_w>0 && C.img_h>0);
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg, size_t i) {
    // Cache latest BGR image + stamp
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
    // Require at least FRONT camera intrinsics + a frame before processing
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!(cams_[0].have_K && cams_[0].have_rgb)) {
        ROS_WARN_THROTTLE(1.0, "[cone_colour] waiting for FRONT camera and info...");
        return;
      }
    }

    // Take a snapshot of camera states (minimize lock time during heavy work)
    std::vector<CamState> C;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      C = cams_;
    }

    const ros::Time det_stamp = msg->header.stamp;
    const std::string lidar_frame = P.lidar_frame_override.empty() ? msg->header.frame_id
                                                                   : P.lidar_frame_override;

    // Prepare per-camera overlays if anyone is listening; also age-check
    for (auto& cam : C) {
      if (!cam.have_rgb) continue;
      const double dt = (det_stamp - cam.stamp_rgb).toSec();
      if (std::fabs(dt) > P.max_image_age_s) {
        ROS_WARN_THROTTLE(1.0, "[cone_colour] RGB age Δt=%.3fs vs detections for %s.", dt, cam.name.c_str());
      }
      if (P.debug_publish_image && cam.pub_dbg && cam.pub_dbg.getNumSubscribers() > 0) {
        cam.annotated = cam.rgb.clone();
      } else {
        cam.annotated.release();
      }
    }

    // Legacy main (front) overlay kept for compatibility with your RViz config
    cv::Mat annotated_main;
    if (P.debug_publish_image && pub_dbg_main_.getNumSubscribers() > 0 && C[0].have_rgb) {
      annotated_main = C[0].rgb.clone();
    }

    // Copy input cones, fill color/color_conf in-place
    qcar_visnav::ConeArray out = *msg;

    for (auto& cone : out.cones) {
      classifyWithBestCamera(cone, lidar_frame, det_stamp, C, annotated_main);
    }

    // Publish colored cone array
    pub_out_.publish(out);

    // Publish per-camera annotated images (only if overlays were created)
    if (P.debug_publish_image) {
      for (const auto& cam : C) {
        if (!cam.annotated.empty() && cam.pub_dbg) {
          std_msgs::Header hdr;
          hdr.stamp = cam.stamp_rgb;
          hdr.frame_id = cam.camera_frame; // e.g., csi_front_optical
          sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(hdr, "bgr8", cam.annotated).toImageMsg();
          cam.pub_dbg.publish(msg_img);
        }
      }
      // Legacy front debug image
      if (!annotated_main.empty()) {
        std_msgs::Header hdr;
        hdr.stamp = C[0].stamp_rgb;
        hdr.frame_id = C[0].camera_frame;
        sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(hdr, "bgr8", annotated_main).toImageMsg();
        pub_dbg_main_.publish(msg_img);
      }
    }

    // Update live cache (images already cloned on receipt)
    {
      std::lock_guard<std::mutex> lk(mtx_);
      for (size_t i=0;i<cams_.size();++i) {
        cams_[i].have_rgb  = C[i].have_rgb;
        cams_[i].stamp_rgb = C[i].stamp_rgb;
        cams_[i].rgb       = C[i].rgb;
      }
    }
  }

  // --- Small helpers ---
  static Eigen::Matrix3d rotFromTF(const geometry_msgs::TransformStamped& tf) {
    // Convert TF quaternion to Eigen rotation
    const auto& q = tf.transform.rotation;
    tf2::Quaternion qq(q.x, q.y, q.z, q.w);
    Eigen::Quaterniond qe(qq.getW(), qq.getX(), qq.getY(), qq.getZ());
    return qe.toRotationMatrix();
  }

  static void polarToXY(double r, double th, double& x, double& y) {
    // Polar (range,bearing) -> XY in LiDAR frame
    x = r * std::cos(th);
    y = r * std::sin(th);
  }

  // Transform point from LiDAR frame into a camera optical frame; exact-time or latest TF
  bool transformLidarToCam(const std::string& cam_frame,
                           const std::string& lidar_frame,
                           const ros::Time& stamp,
                           const Eigen::Vector3d& p_lidar,
                           Eigen::Vector3d& p_cam) {
    try {
      geometry_msgs::TransformStamped T;
      if (P.tf_use_latest) {
        T = tf_buffer_.lookupTransform(cam_frame, lidar_frame, ros::Time(0), ros::Duration(0.05));
      } else {
        T = tf_buffer_.lookupTransform(cam_frame, lidar_frame, stamp, ros::Duration(0.05));
      }
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

  // Project a 3D point in a given camera frame onto its image plane
  static bool projectCam(const CamState& C, const Eigen::Vector3d& p_cam, int& u, int& v, double minZ, double maxZ) {
    const double X = p_cam.x(), Y = p_cam.y(), Z = p_cam.z();
    if (!(Z > minZ && Z < maxZ)) return false; // behind/too close/too far
    u = static_cast<int>(C.fx * (X / Z) + C.cx);
    v = static_cast<int>(C.fy * (Y / Z) + C.cy);
    return (u >= 0 && u < C.img_w && v >= 0 && v < C.img_h);
  }

  // Estimate ROI half-size in pixels from range and cone width (with clamps)
  int roiHalfSizePx(const CamState& C, double Z) const {
    if (Z <= P.min_Z) Z = P.min_Z;
    double w_px = C.fx * P.cone_width_m / Z;      // apparent width in pixels
    int s = static_cast<int>(P.roi_scale * w_px * 0.5);
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
    // Fraction of mask pixels that are >0 (coverage)
    const int total = mask.rows * mask.cols;
    if (total <= 0) return 0.0;
    return static_cast<double>(cv::countNonZero(mask)) / static_cast<double>(total);
  }

  // Draw a labeled box and a small tag onto an annotated image
  void drawAnno(cv::Mat& img, int u, int v, int x0, int y0, int x1, int y1,
                int color_id, double conf, double Z, const std::string& tag) {
    if (img.empty()) return;

    // BGR box colors
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

  // Per-cone classifier: evaluate all cameras and keep the best result
  struct CamResult {
    int cam_idx{-1};
    int color{0};
    double conf{0.0};
    int u{0}, v{0}, x0{0}, y0{0}, x1{0}, y1{0};
    double Z{0.0};
    double score{0.0};
  };

  void classifyWithBestCamera(qcar_visnav::Cone& c,
                              const std::string& lidar_frame,
                              const ros::Time& stamp,
                              std::vector<CamState>& C,
                              cv::Mat& annotated_main_front)
  {
    // LiDAR polar -> LiDAR XY
    double xL=0, yL=0; polarToXY(c.range, c.bearing, xL, yL);
    Eigen::Vector3d p_lidar(xL, yL, 0.0);

    CamResult best;

    // Try each camera that has data
    for (int i=0; i<(int)C.size(); ++i) {
      auto& cam = C[i];
      if (!cam.have_K || !cam.have_rgb) continue;

      // LiDAR -> camera optical frame using TF
      Eigen::Vector3d p_cam;
      if (!transformLidarToCam(cam.camera_frame, lidar_frame, stamp, p_lidar, p_cam)) {
        continue;
      }

      // Project to pixel; require inside image and sane Z
      int u=0, v=0;
      if (!projectCam(cam, p_cam, u, v, P.min_Z, P.max_Z)) continue;

      // Build ROI sized from range (Z) and cone width
      const double Z = p_cam.z();
      const int s = roiHalfSizePx(cam, Z);
      int x0 = u - s, y0 = v - s, x1 = u + s, y1 = v + s;
      clampRect(x0, y0, x1, y1, cam.img_w, cam.img_h);
      const int W = x1 - x0 + 1, H = y1 - y0 + 1;
      if (W <= 1 || H <= 1) continue;
      if (W*H < P.min_pixels_for_conf) continue; // too small to trust

      // --- HSV classification (lightweight) ---
      cv::Mat roi_bgr = cam.rgb(cv::Rect(x0, y0, W, H));
      cv::Mat roi_hsv; cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

      cv::Mat mask_blue, mask_yel;
      const int k = std::max(1, P.hsv_open_ksize | 1); // force odd >=1
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k,k));
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

      // Arbitration score:
      //   prefer stronger conf, larger ROI (steadier), and proximity to center
      const double area = static_cast<double>(W) * static_cast<double>(H);
      double center_bonus = 1.0;
      if (P.prefer_center_weight > 1e-6) {
        const double dx = (static_cast<double>(u) - cam.cx) / std::max(1.0, (double)cam.img_w);
        const double dy = (static_cast<double>(v) - cam.cy) / std::max(1.0, (double)cam.img_h);
        const double r = std::min(1.0, std::sqrt(dx*dx + dy*dy) * std::sqrt(2.0)); // ~0 @ center, ~1 @ corners
        center_bonus = 1.0 + P.prefer_center_weight * (1.0 - r);
      }
      const double score = conf * area * center_bonus;

      if (score > best.score) {
        best.cam_idx = i;
        best.color = color_id;
        best.conf = conf;
        best.u=u; best.v=v; best.x0=x0; best.y0=y0; best.x1=x1; best.y1=y1;
        best.Z = Z;
        best.score = score;
      }
    }

    // Commit result for this cone + annotate winner
    if (best.cam_idx >= 0) {
      c.color = best.color;
      c.color_conf = best.conf;

      // Per-camera overlay (only the winning camera gets a box)
      auto& cam = C[best.cam_idx];
      if (!cam.annotated.empty()) {
        drawAnno(cam.annotated, best.u, best.v, best.x0, best.y0, best.x1, best.y1,
                 best.color, best.conf, best.Z, cam.name);
      }

      // Legacy front overlay (only if front won)
      if (!annotated_main_front.empty() && best.cam_idx == 0) {
        drawAnno(annotated_main_front, best.u, best.v, best.x0, best.y0, best.x1, best.y1,
                 best.color, best.conf, best.Z, "front");
      }

      if (P.debug) {
        ROS_INFO_THROTTLE(0.5,
          "[cone_colour] cam=%s  r=%.2f brg=%.2f -> (u,v)=(%d,%d) Z=%.2f ROI=%dx%d -> color=%d conf=%.2f score=%.1f",
          C[best.cam_idx].name.c_str(), c.range, c.bearing, best.u, best.v, best.Z,
          best.x1-best.x0+1, best.y1-best.y0+1, c.color, c.color_conf, best.score);
      }
    } else {
      // No usable projection/ROI on any camera
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
