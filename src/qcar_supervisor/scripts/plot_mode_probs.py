#!/usr/bin/env python
"""
Live plotter for Jump Markov Filter mode probabilities.

Subscribes to /sup_jmf/mode_prob (Float64MultiArray of length 5)
and updates a matplotlib figure showing:
  HEALTHY, MOTOR_FAULT, STEERING_FAULT, IMU_FAULT, ENCODER_FAULT
"""

import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class ModeProbPlotter:
    def __init__(self,
                 topic='/sup_jmf/mode_prob',
                 max_len=300,
                 mode_count=5,
                 colors=None):
        # buffers
        self.times = deque(maxlen=max_len)
        self.probs = [deque(maxlen=max_len) for _ in range(mode_count)]
        self.start_t = time.time()

        # set up figure
        plt.ion()
        self.fig, self.ax = plt.subplots()

        # custom labels for each mode
        mode_labels = [
            "HEALTHY",
            "MOTOR_FAULT",
            "STEERING_FAULT",
            "IMU_FAULT",
            "ENCODER_FAULT"
        ]

        if colors is None:
            colors = ['blue', 'green', 'red', 'purple', 'orange']

        self.lines = []
        for i in range(mode_count):
            line, = self.ax.plot(
                [], [],
                label=mode_labels[i],
                color=colors[i]
            )
            self.lines.append(line)

        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Probability')
        self.ax.set_ylim(0, 1.0)
        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()

        # subscribe to the ROS topic
        rospy.Subscriber(topic, Float64MultiArray, self._ros_cb)

    def _ros_cb(self, msg):
        t = time.time() - self.start_t
        self.times.append(t)
        for i, p in enumerate(msg.data):
            self.probs[i].append(p)

    def _update_plot(self):
        if not self.times:
            return

        t0, t1 = self.times[0], self.times[-1]
        self.ax.set_xlim(t0, t1 + 0.1)

        for line, buf in zip(self.lines, self.probs):
            line.set_data(self.times, buf)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def spin(self, rate_hz=20):
        # run ROS spin in a background thread
        t = threading.Thread(target=rospy.spin)
        t.daemon = True
        t.start()

        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                self._update_plot()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
        finally:
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    rospy.init_node('mode_probability_plotter', anonymous=True)
    plotter = ModeProbPlotter()
    plotter.spin()
