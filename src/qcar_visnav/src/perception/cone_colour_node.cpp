// cone_colour_node.cpp
//
// Adds color labels (blue/yellow) + confidence to LiDAR-detected cones by
// projecting each cone into the RGB image and classifying an ROI in HSV.
// Publishes:
//   • /cones_colored  (qcar_visnav/ConeArray)  — same frame/range/bearing, with color + conf
//   • /cone_colour/debug_image (sensor_msgs/Image) — annotated RGB image for RViz debugging
//
// Subscribes:
//   • /tracked_cones (qcar_visnav/ConeArray)   [or /cones — configurable]
//   • /rgbd_front/image_raw (sensor_msgs/Image)
//   • /rgbd_front/depth/image_raw (sensor_msgs/Image)   [optional]
//   • /rgbd_front/camera_info (sensor_msgs/CameraInfo)
//
// TF required:
//   lidar_frame -> rgbd_front_optical
//
// Params (private ns ~cone_colour):
//   detections_topic      [string]  default "/tracked_cones"
//   output_topic          [string]  default "/cones_colored"
//   rgb_topic             [string]  default "/rgbd_front/image_raw"
//   depth_topic           [string]  default "/rgbd_front/depth/image_raw"
//   cam_info_topic        [string]  default "/rgbd_front/camera_info"
//   camera_frame          [string]  default "rgbd_front_optical"
//   use_depth_for_roi     [bool]    default true
//   max_image_age_s       [double]  default 0.25
//   cone_width_m          [double]  default 0.25
//   min_roi_px            [int]     default 14
//   max_roi_px            [int]     default 80
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

struct Params {
  std::string detections_topic{"/tracked_cones"};
  std::string output_topic{"/cones_colored"};
  std::string rgb_topic{"/rgbd_front/image_raw"};
  std::string depth_topic{"/rgbd_front/depth/image_raw"};
  std::string cam_info_topic{"/rgbd_front/camera_info"};
  std::string camera_frame{"rgbd_front_optical"};
  bool   use_depth_for_roi{true};
  double max_image_age_s{0.25};
  double cone_width_m{0.25};
  int    min_roi_px{14};
  int    max_roi_px{80};
  int    min_pixels_for_conf{120};
  double conf_floor{0.10};
  cv::Scalar hsv_blue_low{100,150,50};
  cv::Scalar hsv_blue_high{130,255,255};
  cv::Scalar hsv_yellow_low{18,80,80};
  cv::Scalar hsv_yellow_high{35,255,255};
  int    debug{1};
  bool   debug_publish_image{true};
} P;

class ConeColourNode {
public:
  ConeColourNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : tf_buffer_(ros::Duration(10.0)),
      tf_listener_(tf_buffer_) {

    // Load params
    pnh.param("detections_topic", P.detections_topic, P.detections_topic);
    pnh.param("output_topic",     P.output_topic,     P.output_topic);
    pnh.param("rgb_topic",        P.rgb_topic,        P.rgb_topic);
    pnh.param("depth_topic",      P.depth_topic,      P.depth_topic);
    pnh.param("cam_info_topic",   P.cam_info_topic,   P.cam_info_topic);
    pnh.param("camera_frame",     P.camera_frame,     P.camera_frame);
    pnh.param("use_depth_for_roi",P.use_depth_for_roi,P.use_depth_for_roi);
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

    // Subscriptions
    sub_cones_ = nh.subscribe(P.detections_topic, 1, &ConeColourNode::conesCb, this);
    sub_rgb_   = nh.subscribe(P.rgb_topic,        1, &ConeColourNode::rgbCb,   this);
    sub_depth_ = nh.subscribe(P.depth_topic,      1, &ConeColourNode::depthCb, this);
    sub_cinfo_ = nh.subscribe(P.cam_info_topic,   1, &ConeColourNode::camInfoCb, this);

    // Publications
    pub_out_   = nh.advertise<qcar_visnav::ConeArray>(P.output_topic, 1, false);
    if (P.debug_publish_image) {
      pub_dbg_img_ = nh.advertise<sensor_msgs::Image>("/cone_colour/debug_image", 1, false);
    }

    ROS_INFO("[cone_colour] subs: '%s','%s','%s','%s' -> pubs: '%s'%s  (camera_frame=%s)",
             P.detections_topic.c_str(), P.rgb_topic.c_str(), P.depth_topic.c_str(),
             P.cam_info_topic.c_str(), P.output_topic.c_str(),
             P.debug_publish_image ? ", '/cone_colour/debug_image'" : "",
             P.camera_frame.c_str());
  }

private:
  // Latest images
  std::mutex mtx_;
  cv::Mat last_rgb_;
  cv::Mat last_depth_m_; // CV_32FC1 meters
  ros::Time rgb_stamp_, depth_stamp_;
  bool have_rgb_{false}, have_depth_{false};

  // Camera intrinsics
  bool   have_K_{false};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  int    img_w_{0}, img_h_{0};

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ROS IO
  ros::Subscriber sub_cones_, sub_rgb_, sub_depth_, sub_cinfo_;
  ros::Publisher  pub_out_;
  ros::Publisher  pub_dbg_img_;

  // --- Callbacks ---
  void camInfoCb(const sensor_msgs::CameraInfoConstPtr& msg) {
    fx_ = msg->K[0]; fy_ = msg->K[4];
    cx_ = msg->K[2]; cy_ = msg->K[5];
    img_w_ = msg->width; img_h_ = msg->height;
    have_K_ = (fx_>0 && fy_>0 && img_w_>0 && img_h_>0);
  }

  void rgbCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, "bgr8");
      std::lock_guard<std::mutex> lk(mtx_);
      last_rgb_ = cvp->image.clone();
      rgb_stamp_ = msg->header.stamp;
      have_rgb_ = true;
    } catch (const cv_bridge::Exception& e) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] cv_bridge RGB: %s", e.what());
    }
  }

  void depthCb(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      std::lock_guard<std::mutex> lk(mtx_);
      last_depth_m_ = cvp->image.clone();
      depth_stamp_  = msg->header.stamp;
      have_depth_   = true;
    } catch (const cv_bridge::Exception& e) {
      // Try 16UC1 -> 32F meters
      try {
        cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        std::lock_guard<std::mutex> lk(mtx_);
        last_depth_m_ = cvp->image.clone();
        last_depth_m_.convertTo(last_depth_m_, CV_32F, 1.0/1000.0);
        depth_stamp_ = msg->header.stamp;
        have_depth_ = true;
      } catch (...) {
        ROS_WARN_THROTTLE(1.0, "[cone_colour] depth conversion failed (need 32FC1 or 16UC1).");
      }
    }
  }

  void conesCb(const qcar_visnav::ConeArray::ConstPtr& msg) {
    if (!have_K_) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] waiting for CameraInfo...");
      return;
    }

    // Grab latest images
    cv::Mat rgb, depth_m;
    ros::Time rgb_t, depth_t;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!have_rgb_) { ROS_WARN_THROTTLE(1.0, "[cone_colour] no RGB yet..."); return; }
      rgb = last_rgb_.clone();
      rgb_t = rgb_stamp_;
      if (P.use_depth_for_roi && have_depth_) {
        depth_m = last_depth_m_.clone();
        depth_t = depth_stamp_;
      }
    }

    // Check age vs detection stamp (warn only)
    const ros::Time stamp = msg->header.stamp;
    const double dt = (stamp - rgb_t).toSec();
    if (std::fabs(dt) > P.max_image_age_s) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] RGB age Δt=%.3fs vs detections.", dt);
    }

    // Prepare output + debug canvas
    qcar_visnav::ConeArray out = *msg;
    cv::Mat annotated;
    if (P.debug_publish_image) {
      annotated = rgb.clone();
    }

    for (auto& c : out.cones) {
      classifyOneCone(c, msg->header.frame_id, stamp, rgb, depth_m, annotated);
    }

    pub_out_.publish(out);

    // Publish annotated debug image
    if (P.debug_publish_image && !annotated.empty() && pub_dbg_img_) {
      std_msgs::Header hdr;
      hdr.stamp = rgb_t;                 // align with the RGB frame
      hdr.frame_id = P.camera_frame;     // e.g., "rgbd_front_optical"
      sensor_msgs::ImagePtr msg_img = cv_bridge::CvImage(hdr, "bgr8", annotated).toImageMsg();
      pub_dbg_img_.publish(msg_img);
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

  bool transformLidarToCam(const std::string& lidar_frame,
                           const ros::Time& stamp,
                           const Eigen::Vector3d& p_lidar,
                           Eigen::Vector3d& p_cam) {
    try {
      geometry_msgs::TransformStamped T =
        tf_buffer_.lookupTransform(P.camera_frame, lidar_frame, stamp, ros::Duration(0.05));
      Eigen::Matrix3d R = rotFromTF(T);
      Eigen::Vector3d t(T.transform.translation.x,
                        T.transform.translation.y,
                        T.transform.translation.z);
      p_cam = R * p_lidar + t;
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "[cone_colour] TF %s->%s: %s",
                        lidar_frame.c_str(), P.camera_frame.c_str(), ex.what());
      return false;
    }
  }

  bool projectCam(const Eigen::Vector3d& p_cam, int& u, int& v) const {
    const double X = p_cam.x(), Y = p_cam.y(), Z = p_cam.z();
    if (!(Z > 0.05)) return false;
    u = static_cast<int>(fx_ * (X / Z) + cx_);
    v = static_cast<int>(fy_ * (Y / Z) + cy_);
    return (u >= 0 && u < img_w_ && v >= 0 && v < img_h_);
  }

  int roiHalfSizePx(double Z, double cone_width_m) const {
    if (Z <= 0.05) Z = 0.05;
    double w_px = fx_ * cone_width_m / Z;
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

  // Remap an RGB pixel coordinate into the depth image resolution before sampling
  static float depthFromRgbPixel(const cv::Mat& depth_m, int u_rgb, int v_rgb, int rgb_w, int rgb_h) {
    if (depth_m.empty() || rgb_w <= 0 || rgb_h <= 0) return std::numeric_limits<float>::quiet_NaN();
    int x = static_cast<int>(std::lround((double)u_rgb * depth_m.cols / rgb_w));
    int y = static_cast<int>(std::lround((double)v_rgb * depth_m.rows / rgb_h));
    x = std::max(0, std::min(depth_m.cols - 1, x));
    y = std::max(0, std::min(depth_m.rows - 1, y));
    float z = depth_m.at<float>(y, x);
    if (!std::isfinite(z) || z <= 0.0f || z > 20.0f) return std::numeric_limits<float>::quiet_NaN();
    return z;
  }

  void drawAnno(cv::Mat& img, int u, int v, int x0, int y0, int x1, int y1,
                int color_id, double conf, double Z) {
    if (img.empty()) return;

    // BGR colours for boxes
    cv::Scalar col(200,200,200);            // unknown -> grey
    if (color_id == 1) col = cv::Scalar(255, 0,   0  ); // blue
    else if (color_id == 2) col = cv::Scalar(0,   255,255); // yellow

    cv::rectangle(img, cv::Rect(cv::Point(x0,y0), cv::Point(x1,y1)), col, 2);
    cv::circle(img, cv::Point(u,v), 4, col, -1);

    char text[64];
    if (color_id == 1)  std::snprintf(text, sizeof(text), "B %.2f  Z=%.2f", conf, Z);
    else if (color_id == 2) std::snprintf(text, sizeof(text), "Y %.2f  Z=%.2f", conf, Z);
    else                 std::snprintf(text, sizeof(text), "? 0.00  Z=%.2f", Z);

    int baseline=0;
    cv::Size ts = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
    int tx = std::max(0, std::min(img.cols - ts.width - 2, u + 6));
    int ty = std::max(ts.height + 2, std::min(img.rows - 2, v - 6));
    cv::putText(img, text, cv::Point(tx,ty), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 2);
  }

  void classifyOneCone(qcar_visnav::Cone& c,
                       const std::string& lidar_frame,
                       const ros::Time& stamp,
                       const cv::Mat& rgb,
                       const cv::Mat& depth_m,
                       cv::Mat& annotated_out) {
    // LiDAR polar -> LiDAR XY -> Cam 3D
    double xL=0, yL=0; polarToXY(c.range, c.bearing, xL, yL);
    Eigen::Vector3d p_lidar(xL, yL, 0.0), p_cam;
    if (!transformLidarToCam(lidar_frame, stamp, p_lidar, p_cam)) {
      c.color = 0; c.color_conf = 0.0; return;
    }

    // Project
    int u=0, v=0;
    if (!projectCam(p_cam, u, v)) {
      c.color = 0; c.color_conf = 0.0; return;
    }

    // ROI
    double Z = p_cam.z();
    if (P.use_depth_for_roi) {
      float z_meas = depthFromRgbPixel(depth_m, u, v, rgb.cols, rgb.rows);
      if (std::isfinite(z_meas) && z_meas > 0.05) Z = z_meas;
    }
    const int s = roiHalfSizePx(Z, P.cone_width_m);
    int x0 = u - s, y0 = v - s, x1 = u + s, y1 = v + s;
    clampRect(x0, y0, x1, y1, rgb.cols, rgb.rows);
    const int W = x1 - x0 + 1, H = y1 - y0 + 1;
    if (W*H < P.min_pixels_for_conf) { c.color=0; c.color_conf=0.0; return; }

    // HSV classify
    cv::Mat roi_bgr = rgb(cv::Rect(x0, y0, W, H));
    cv::Mat roi_hsv; cv::cvtColor(roi_bgr, roi_hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_blue, mask_yel;
    cv::inRange(roi_hsv, P.hsv_blue_low,   P.hsv_blue_high,   mask_blue);
    cv::inRange(roi_hsv, P.hsv_yellow_low, P.hsv_yellow_high, mask_yel);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3});
    cv::morphologyEx(mask_blue, mask_blue, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask_yel,  mask_yel,  cv::MORPH_OPEN, kernel);

    const double r_blue = ratioNonZero(mask_blue);
    const double r_yel  = ratioNonZero(mask_yel);

    int color_id = 0; double conf = 0.0;
    if (r_blue > r_yel) { color_id = 1; conf = r_blue; }
    else                { color_id = 2; conf = r_yel;  }
    if (conf < P.conf_floor) { color_id = 0; conf = 0.0; }

    c.color = color_id;
    c.color_conf = conf;

    if (P.debug_publish_image) {
      drawAnno(annotated_out, u, v, x0, y0, x1, y1, color_id, conf, Z);
    }

    if (P.debug) {
      ROS_INFO_THROTTLE(0.5,
        "[cone_colour] r=%.2f brg=%.2f -> (u,v)=(%d,%d) Z=%.2f ROI=%dx%d blue=%.2f yel=%.2f -> color=%d conf=%.2f",
        c.range, c.bearing, u, v, Z, W, H, r_blue, r_yel, c.color, c.color_conf);
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
