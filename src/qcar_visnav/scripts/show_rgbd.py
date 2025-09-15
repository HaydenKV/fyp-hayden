#!/usr/bin/env python3
# rgbd_front_viewer.py
# Display /rgbd_front RGB + Depth (colormap) side-by-side with FPS and sliders for depth range.

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import message_filters

class RGBDViewer:
    def __init__(self):
        # Params (override via rosparam if you like)
        self.color_topic = rospy.get_param("~color_topic", "/rgbd_front/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/rgbd_front/depth/image_raw")
        self.caminfo_topic = rospy.get_param("~camera_info_topic", "/rgbd_front/camera_info")
        self.window_name = rospy.get_param("~window", "RGBD Viewer")
        self.init_min_m = float(rospy.get_param("~min_depth_m", 0.2))
        self.init_max_m = float(rospy.get_param("~max_depth_m", 5.0))
        self.use_equalize = bool(rospy.get_param("~equalize_color", False))

        self.bridge = CvBridge()
        self.K = None
        self.last_time = None
        self.fps = 0.0

        # Prepare OpenCV window and trackbars (trackbars use integer values)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 1280, 540)
        # Trackbars in millimetres for finer control
        self.max_track_mm = 20000  # 20 m upper bound
        min_init_mm = int(max(0, min(self.init_min_m, self.init_max_m) * 1000))
        max_init_mm = int(min(self.max_track_mm, max(self.init_min_m, self.init_max_m) * 1000))
        cv2.createTrackbar("min_mm", self.window_name, min_init_mm, self.max_track_mm, lambda v: None)
        cv2.createTrackbar("max_mm", self.window_name, max_init_mm, self.max_track_mm, lambda v: None)

        # Subscribers (sync color+depth, separate CameraInfo)
        color_sub = message_filters.Subscriber(self.color_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.caminfo_sub = rospy.Subscriber(self.caminfo_topic, CameraInfo, self.caminfo_cb, queue_size=1)

        # Approximate sync (sim clocks can jitter)
        sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self.cb)

        rospy.loginfo("[rgbd_front_viewer] Subscribed to:\n  %s\n  %s\n  %s",
                      self.color_topic, self.depth_topic, self.caminfo_topic)

    def caminfo_cb(self, msg):
        # Store intrinsics if you want to print/verify; not required for display
        self.K = np.array(msg.K, dtype=np.float32).reshape(3, 3)

    @staticmethod
    def _depth_to_meters(depth_img, encoding):
        """
        Convert depth image to float32 meters depending on encoding.
        """
        if encoding.upper().startswith("32FC"):  # e.g., "32FC1"
            depth_m = depth_img.astype(np.float32)
        elif encoding.upper() in ("16UC1", "MONO16"):
            # Assume millimetres -> meters
            depth_m = depth_img.astype(np.float32) * 0.001
        else:
            # Fallback: try float
            depth_m = depth_img.astype(np.float32)
        return depth_m

    def _colormap_depth(self, depth_m, min_m, max_m):
        """
        Clip [min_m, max_m], normalize to [0,255], apply color map.
        """
        depth = np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0)
        # Prevent degenerate ranges
        if max_m <= min_m + 1e-6:
            max_m = min_m + 0.001
        clipped = np.clip(depth, min_m, max_m)
        norm = (clipped - min_m) / (max_m - min_m)
        norm_uint8 = (norm * 255.0).astype(np.uint8)
        depth_color = cv2.applyColorMap(norm_uint8, cv2.COLORMAP_JET)
        return depth_color

    def _draw_overlays(self, img_bgr, depth_color, depth_m, color_enc, depth_enc):
        """
        Compose side-by-side, write FPS/encodings, and center depth readout.
        """
        # Optionally equalize color display (visual aid only)
        if self.use_equalize:
            yuv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2YUV)
            yuv[:, :, 0] = cv2.equalizeHist(yuv[:, :, 0])
            img_bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)

        # Make heights match if needed
        h = max(img_bgr.shape[0], depth_color.shape[0])
        def pad_to_h(img):
            if img.shape[0] == h:
                return img
            pad = h - img.shape[0]
            top = pad // 2
            bottom = pad - top
            return cv2.copyMakeBorder(img, top, bottom, 0, 0, cv2.BORDER_CONSTANT, value=(0,0,0))

        imc = pad_to_h(img_bgr)
        dcm = pad_to_h(depth_color)
        combined = np.hstack((imc, dcm))

        # FPS
        cv2.putText(combined, f"FPS: {self.fps:.1f}", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2, cv2.LINE_AA)
        # Encodings
        cv2.putText(combined, f"Color enc: {color_enc}", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(combined, f"Depth enc: {depth_enc}", (10, 74),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)

        # Center-depth readout (from depth_m)
        h_c, w_c = imc.shape[:2]
        cx = w_c // 2
        cy = imc.shape[0] // 2
        # Map center of color image to depth panel x-offset
        depth_center = float(depth_m[min(cy, depth_m.shape[0]-1),
                                     min(cx, depth_m.shape[1]-1)])
        cv2.drawMarker(combined, (cx, cy), (0,255,0), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)
        cv2.putText(combined, f"Center depth: {depth_center:.3f} m", (10, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2, cv2.LINE_AA)

        # If intrinsics available, print fx, fy briefly
        if self.K is not None:
            fx, fy = self.K[0,0], self.K[1,1]
            cv2.putText(combined, f"fx={fx:.1f}, fy={fy:.1f}", (10, 126),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1, cv2.LINE_AA)

        return combined

    def cb(self, color_msg, depth_msg):
        # Compute FPS
        now = color_msg.header.stamp.to_sec() if color_msg.header.stamp else rospy.Time.now().to_sec()
        if self.last_time is not None:
            dt = max(1e-6, now - self.last_time)
            self.fps = 0.9*self.fps + 0.1*(1.0/dt)
        else:
            self.fps = 0.0
        self.last_time = now

        # Read trackbars (mm → m)
        min_mm = cv2.getTrackbarPos("min_mm", self.window_name)
        max_mm = cv2.getTrackbarPos("max_mm", self.window_name)
        if max_mm < min_mm:
            max_mm = min_mm + 1
        min_m = float(min_mm) / 1000.0
        max_m = float(max_mm) / 1000.0

        # Convert images
        try:
            color_bgr = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr_throttle(1.0, "RGB cv_bridge error: %s", str(e))
            return

        try:
            # For depth, pass through and convert based on encoding
            depth_raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_m = self._depth_to_meters(depth_raw, dep_
