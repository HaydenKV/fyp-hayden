#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

class AccModePlotter:
    def __init__(self,
                 topic='/jmf/mode_probs',
                 window_seconds=30.0,
                 healthy_idx=0,
                 faulty_idx=1,
                 colors=None,
                 rate_hz=20):
        self.window_seconds = float(window_seconds)
        self.healthy_idx = healthy_idx
        self.faulty_idx  = faulty_idx
        self.rate_hz = rate_hz

        # keep full history (no maxlen)
        self.times = deque()
        self.p_healthy = deque()
        self.p_faulty  = deque()

        # optional: shaded fault windows in SIM time
        self.fault_windows = [(80.0, 100.0), (120.0, 135.0)]

        plt.ion()
        self.fig, self.ax = plt.subplots()

        if colors is None:
            colors = ['blue', 'red']
        self.line_healthy, = self.ax.plot([], [], label="ACC_HEALTHY", color=colors[0])
        self.line_faulty,  = self.ax.plot([], [], label="ACC_FAULTY",  color=colors[1])

        self.ax.set_xlabel('Simulation Time (s)')
        self.ax.set_ylabel('Probability')
        self.ax.set_ylim(-0.1, 1.1)
        self.ax.legend(loc='upper right')
        self.ax.grid(True)
        self.fig.canvas.draw()

        rospy.Subscriber(topic, Float64MultiArray, self._ros_cb)

    def _ros_cb(self, msg):
        # use SIM time so overlays align with injections
        t = rospy.Time.now().to_sec()
        if not msg.data or len(msg.data) < 2:
            return

        # append (NO PURGE)
        self.times.append(t)
        self.p_healthy.append(msg.data[self.healthy_idx])
        self.p_faulty.append(msg.data[self.faulty_idx])

    def _update_plot(self):
        if not self.times:
            return

        # show only the last window_seconds on x-axis, but keep full history in data
        t1 = self.times[-1]
        t0 = t1 - self.window_seconds
        self.ax.set_xlim(t0, t1)

        # plot full history; points outside xlim just won’t be visible
        self.line_healthy.set_data(self.times, self.p_healthy)
        self.line_faulty.set_data(self.times, self.p_faulty)

        # refresh fault overlays for current view
        for p in list(self.ax.patches):
            p.remove()
        for start, end in self.fault_windows:
            if end >= t0 and start <= t1:
                self.ax.axvspan(start, end, color='gray', alpha=0.3, label='_nolegend_')

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(1.0 / (5.0 * self.rate_hz))

    def spin(self):
        # wait for /clock
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() == 0.0:
            rospy.loginfo_throttle(1.0, "Waiting for simulated time (/clock)...")
            time.sleep(0.1)

        t = threading.Thread(target=rospy.spin)
        t.daemon = True
        t.start()

        rate = rospy.Rate(self.rate_hz)
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
    rospy.init_node('acc_mode_plotter', anonymous=True)
    plotter = AccModePlotter(healthy_idx=0, faulty_idx=1, window_seconds=30.0, rate_hz=20)
    plotter.spin()
