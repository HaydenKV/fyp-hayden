#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ConeDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        # Rough camera intrinsics for QCar depth camera (adjust if known)
        self.fx = 554.25469
        self.fy = 554.25469
        self.cx = 320
        self.cy = 240

        rospy.Subscriber("/front_camera/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/depth_camera/depth/image_raw", Image, self.depth_callback)

    def rgb_callback(self, data):
        self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def depth_callback(self, data):
        self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")

    def detect_cones(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)

        masks = {
            "blue": cv2.inRange(hsv, (100, 150, 50), (130, 255, 255)),
            "yellow": cv2.inRange(hsv, (20, 100, 100), (30, 255, 255))
        }

        display = self.rgb_image.copy()

        for colour, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 200:  # Skip small noise
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                z = float(self.depth_image[cy, cx])
                if z == 0.0 or z > 10.0:
                    continue

                x = (cx - self.cx) * z / self.fx
                y = (cy - self.cy) * z / self.fy

                # Display on image
                cv2.circle(display, (cx, cy), 5, (0, 255, 0), -1)
                cv2.putText(display, f"{colour} ({x:.2f},{y:.2f},{z:.2f})", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

                rospy.loginfo(f"{colour} cone at x={x:.2f}, y={y:.2f}, z={z:.2f}")

        cv2.imshow("Cone Detection (RGB)", display)
        cv2.waitKey(1)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.detect_cones()
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("cone_detector_rgbd", anonymous=True)
    node = ConeDetector()
    node.run()

