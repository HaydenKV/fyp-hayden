#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RGBDViewer:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        rospy.Subscriber("/front_camera/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.depth_callback)

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            rospy.logerr("RGB error: %s", str(e))

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
            depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0)
            clipped = np.clip(depth_image, 0.2, 5.0)
            depth_norm = cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX)
            depth_coloured = cv2.applyColorMap(depth_norm.astype(np.uint8), cv2.COLORMAP_JET)
            self.depth_image = depth_coloured
        except Exception as e:
            rospy.logerr("Depth error: %s", str(e))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.rgb_image is not None and self.depth_image is not None:
                combined = np.hstack((self.rgb_image, self.depth_image))
                cv2.imshow("RGB (left) + Depth (right)", combined)
                cv2.waitKey(1)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("rgbd_viewer", anonymous=True)
    viewer = RGBDViewer()
    viewer.run()

