#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelState
import matplotlib.pyplot as plt
import math
import tf

class LidarMapper:
    def __init__(self):
        self.latest_points = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.car_x = 0.0
        self.car_y = 0.0

        rospy.init_node("lidar_map_node")

        # Subscribe to scan
        rospy.Subscriber("/scan", LaserScan, self.callback, queue_size=1, buff_size=2**24)

        # Gazebo pose service
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Plot setup
        plt.ion()
        self.fig, self.ax = plt.subplots()

        rospy.loginfo("Lidar live map with car pose initialized.")
        self.loop()

    def callback(self, msg):
        try:
            # Get car pose
            state = self.get_model_state("qcar", "")
            pos = state.pose.position
            ori = state.pose.orientation
            quat = (ori.x, ori.y, ori.z, ori.w)
            _, _, yaw = tf.transformations.euler_from_quaternion(quat)

            self.car_x = pos.x
            self.car_y = pos.y

            # Transform LiDAR scan to world frame
            self.latest_points = []
            angle = msg.angle_min
            for r in msg.ranges:
                if r > 0 and not math.isinf(r):
                    xr = r * math.cos(angle + math.pi / 2)
                    yr = r * math.sin(angle + math.pi / 2)
                    xw = self.car_x + math.cos(yaw) * xr - math.sin(yaw) * yr
                    yw = self.car_y + math.sin(yaw) * xr + math.cos(yaw) * yr
                    self.latest_points.append((xw, yw))
                angle += msg.angle_increment

        except Exception as e:
            rospy.logwarn(f"Error getting model state or processing scan: {e}")

    def loop(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.ax.cla()
            self.ax.set_title("Live LiDAR Scan (World Frame)")
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            self.ax.axis("equal")

            if self.latest_points:
                xs, ys = zip(*self.latest_points)
                self.ax.plot(xs, ys, "g.", markersize=2, label="LiDAR")

            # Plot current QCar position
            self.ax.plot(self.car_x, self.car_y, marker="o", markersize=6, color="black", label="QCar")

            self.ax.legend()
            self.ax.grid(True)
            plt.pause(0.001)
            rate.sleep()

if __name__ == "__main__":
    LidarMapper()

