#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Show RGB + Depth streams in OpenCV windows.
Params (private ~ns):
  ~rgb_topic:            default "/front_camera/image_raw"
  ~depth_topic:          default "/depth_camera/depth/image_raw"
  ~camera_info_topic:    default "/front_camera/camera_info" (optional)
  ~near_m:               default 0.2   (depth viz min)
  ~far_m:                default 5.0   (depth viz max)
  ~overlay_alpha:        default 0.5   (RGB-depth overlay)
Press 'q' to quit.
"""
import threading
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ShowRGBD:
    def __init__(self):
        pnh = rospy.get_param
        self.rgb_topic   = pnh("~rgb_topic",   "/front_camera/image_raw")
        self.depth_topic = pnh("~depth_topic", "/depth_camera/depth/image_raw")
        self.info_topic  = pnh("~camera_info_topic", "/front_camera/camera_info")
        self.near_m      = float(pnh("~near_m", 0.2))
        self.far_m       = float(pnh("~far_m", 5.0))
        self.alpha       = float(pnh("~overlay_alpha", 0.5))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_rgb = None
        self.last_depth_m = None
        self.last_rgb_stamp = None
        self.last_depth_stamp = None

        rospy.loginfo("[show_rgbd] Subscribing rgb='%s' depth='%s' info='%s'",
                      self.rgb_topic, self.depth_topic, self.info_topic)

        self.sub_rgb   = rospy.Subscriber(self.rgb_topic, Image, self.cb_rgb, queue_size=1)
        self.sub_depth = rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
        # CameraInfo is optional; we don't actually use it yet but good to confirm the stream exists.
        self.sub_info  = rospy.Subscriber(self.info_topic, CameraInfo, self.cb_info, queue_size=1)

    def cb_info(self, msg):
        # Only used as a heartbeat
        rospy.loginfo_once("[show_rgbd] CameraInfo is arriving.")

    def cb_rgb(self, msg):
        try:
            # Accept BGR8 or RGB8; convert to BGR for OpenCV
            enc = msg.encoding.lower()
            if enc == "bgr8":
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            rospy.logwarn_throttle(1.0, "[show_rgbd] RGB decode error: %s", str(e))
            return

        with self.lock:
            self.last_rgb = img
            self.last_rgb_stamp = msg.header.stamp

    def cb_depth(self, msg):
        try:
            # Normalize depth to meters as float32
            if msg.encoding in ("32FC1", "32FC1"):
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                depth_m = np.array(d, dtype=np.float32)
            elif msg.encoding in ("16UC1", "MONO16"):
                d = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
                depth_m = np.where(d > 0, d.astype(np.float32) * 0.001, np.nan)  # mm -> m
            else:
                # Try generic float
                d = self.bridge.imgmsg_to_cv2(msg)
                depth_m = d.astype(np.float32)
        except CvBridgeError as e:
            rospy.logwarn_throttle(1.0, "[show_rgbd] Depth decode error: %s", str(e))
            return

        with self.lock:
            self.last_depth_m = depth_m
            self.last_depth_stamp = msg.header.stamp

    def render_loop(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            rgb = None
            depth_m = None
            rgb_t = None
            dep_t = None
            with self.lock:
                if self.last_rgb is not None:
                    rgb = self.last_rgb.copy()
                    rgb_t = self.last_rgb_stamp
                if self.last_depth_m is not None:
                    depth_m = self.last_depth_m.copy()
                    dep_t = self.last_depth_stamp

            # Nothing to show yet
            if rgb is None and depth_m is None:
                rate.sleep()
                continue

            # Prepare depth visualization
            if depth_m is not None:
                # Clip & scale to 0..255
                d = depth_m.copy()
                d[np.isfinite(d) == False] = 0.0
                d = np.clip((d - self.near_m) / max(1e-6, (self.far_m - self.near_m)), 0.0, 1.0)
                d8 = (d * 255.0).astype(np.uint8)
                depth_vis = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
                # Put text
                txt = f"Depth [{self.near_m:.1f}..{self.far_m:.1f}] m"
                cv2.putText(depth_vis, txt, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                if dep_t is not None:
                    cv2.putText(depth_vis, f"t={dep_t.to_sec():.3f}", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("Depth (m)", depth_vis)

            # Show RGB (and overlay if both exist)
            if rgb is not None:
                show_rgb = rgb.copy()
                cv2.putText(show_rgb, "RGB", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                if rgb_t is not None:
                    cv2.putText(show_rgb, f"t={rgb_t.to_sec():.3f}", (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow("RGB", show_rgb)

                if depth_m is not None:
                    # Build a grayscale mask from depth visualization range for overlay
                    d = np.clip((depth_m - self.near_m) / max(1e-6, (self.far_m - self.near_m)), 0.0, 1.0)
                    d8 = (d * 255.0).astype(np.uint8)
                    depth_color = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
                    # Resize if shapes differ
                    if depth_color.shape[:2] != rgb.shape[:2]:
                        depth_color = cv2.resize(depth_color, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                    overlay = cv2.addWeighted(rgb, 1.0 - self.alpha, depth_color, self.alpha, 0.0)
                    cv2.putText(overlay, f"Overlay alpha={self.alpha:.2f}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1, cv2.LINE_AA)
                    cv2.imshow("RGB + Depth overlay", overlay)

            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                rospy.signal_shutdown("user quit")
                break
            rate.sleep()

def main():
    rospy.init_node("show_rgbd")
    node = ShowRGBD()
    rospy.loginfo("[show_rgbd] Ready. Press 'q' to quit the OpenCV windows.")
    node.render_loop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
