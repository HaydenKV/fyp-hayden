#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import math

class LidarLivePlot:
    def __init__(self):
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        rospy.init_node("lidar_plot_node")
        rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1, buff_size=2**24)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.loop()

    def callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def loop(self):
        rate = rospy.Rate(2)  # 2 Hz update rate
        while not rospy.is_shutdown():
            self.ax.cla()
            self.ax.set_title("Live LiDAR Scan")
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            self.ax.axis("equal")
            self.ax.plot(0, 0, marker="o", markersize=5, markerfacecolor="black", label="QCar")

            angle = self.angle_min
            for r in self.ranges:
                if r > 0 and not math.isinf(r):
                    x = r * math.cos(angle + math.pi/2)
                    y = r * math.sin(angle + math.pi/2)
                    self.ax.plot(x, y, "g.", markersize=3)
                angle += self.angle_increment

            self.ax.legend()
            self.ax.grid(True)
            plt.pause(0.001)
            rate.sleep()

if __name__ == "__main__":
    LidarLivePlot()

