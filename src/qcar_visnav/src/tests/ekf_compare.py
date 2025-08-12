#!/usr/bin/env python3
import rospy
import math
import csv
import os
from collections import deque
from threading import Lock

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

try:
    import matplotlib.pyplot as plt
    HAVE_MPL = True
except Exception:
    HAVE_MPL = False

def yaw_from_quat(q):
    # q is geometry_msgs/Quaternion
    # yaw = atan2(2(wz+xy), 1 - 2(y^2 + z^2))
    s = 2.0*(q.w*q.z + q.x*q.y)
    c = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
    return math.atan2(s, c)

class EKFCompare:
    def __init__(self):
        # params (all can be overridden via rosparam)
        self.topic_ekf   = rospy.get_param("~ekf_odom_topic",   "/qcar/ekf/odom")
        self.topic_truth = rospy.get_param("~truth_odom_topic", "/odom")
        self.topic_imu   = rospy.get_param("~imu_topic",        "/imu")
        self.topic_js    = rospy.get_param("~joint_states_topic","/qcar/joint_states")

        self.print_hz    = float(rospy.get_param("~print_hz", 1.0))
        self.enable_plot = bool(rospy.get_param("~plot", False))
        self.save_csv    = bool(rospy.get_param("~save_csv", False))
        self.csv_path    = os.path.expanduser(rospy.get_param("~csv_path", "~/ekf_diag.csv"))
        self.window_secs = float(rospy.get_param("~plot_window_secs", 10.0))

        # data
        self.lock = Lock()
        self.last_truth = None
        self.last_ekf   = None
        self.last_imu   = None
        self.last_js    = None

        # rolling buffers for plotting
        self.buf_t = deque(maxlen=10000)
        self.buf_v_truth = deque(maxlen=10000)
        self.buf_v_ekf   = deque(maxlen=10000)
        self.buf_r_truth = deque(maxlen=10000)
        self.buf_r_ekf   = deque(maxlen=10000)

        # CSV
        self.csv_file = None
        self.csv_writer = None
        if self.save_csv:
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "t", "x_truth", "y_truth", "psi_truth", "v_truth", "r_truth",
                "x_ekf", "y_ekf", "psi_ekf", "v_ekf", "r_ekf",
                "v_err", "r_err"
            ])

        # subs
        self.sub_truth = rospy.Subscriber(self.topic_truth, Odometry, self.cb_truth, queue_size=50)
        self.sub_ekf   = rospy.Subscriber(self.topic_ekf,   Odometry, self.cb_ekf,   queue_size=50)
        self.sub_imu   = rospy.Subscriber(self.topic_imu,   Imu,      self.cb_imu,   queue_size=50)
        self.sub_js    = rospy.Subscriber(self.topic_js,    JointState,self.cb_js,   queue_size=50)

        # timers
        self.print_timer = rospy.Timer(rospy.Duration(1.0/max(1e-6,self.print_hz)), self.timer_print)
        if self.enable_plot and HAVE_MPL:
            self.fig, (self.ax_v, self.ax_r) = plt.subplots(1,2, figsize=(10,4))
            self.ax_v.set_title("Linear speed (v) [m/s]")
            self.ax_r.set_title("Yaw rate (r) [rad/s]")
            self.ax_v.grid(True); self.ax_r.grid(True)
            self.plot_timer = rospy.Timer(rospy.Duration(0.1), self.timer_plot)
        elif self.enable_plot and not HAVE_MPL:
            rospy.logwarn("Matplotlib not available; disabling plot.")
            self.enable_plot = False

        rospy.loginfo("[ekf_compare] ekf=%s truth=%s imu=%s js=%s plot=%s csv=%s",
            self.topic_ekf, self.topic_truth, self.topic_imu, self.topic_js,
            str(self.enable_plot), str(self.save_csv))

    def cb_truth(self, msg):
        with self.lock:
            self.last_truth = msg

    def cb_ekf(self, msg):
        with self.lock:
            self.last_ekf = msg

    def cb_imu(self, msg):
        with self.lock:
            self.last_imu = msg

    def cb_js(self, msg):
        with self.lock:
            self.last_js = msg

    def timer_print(self, event):
        with self.lock:
            if self.last_truth is None or self.last_ekf is None:
                return

            t = self.last_ekf.header.stamp.to_sec()

            # truth
            xt = self.last_truth.pose.pose.position.x
            yt = self.last_truth.pose.pose.position.y
            psit = yaw_from_quat(self.last_truth.pose.pose.orientation)
            vt = self.last_truth.twist.twist.linear.x
            rt = self.last_truth.twist.twist.angular.z

            # ekf
            xe = self.last_ekf.pose.pose.position.x
            ye = self.last_ekf.pose.pose.position.y
            psie = yaw_from_quat(self.last_ekf.pose.pose.orientation)
            ve = self.last_ekf.twist.twist.linear.x
            re = self.last_ekf.twist.twist.angular.z

            dv = ve - vt
            dr = re - rt
            dpsi = self.wrap_angle(psie - psit)

            rospy.loginfo("[cmp] v: truth=%.3f  ekf=%.3f  dv=%.3f | r: truth=%.3f  ekf=%.3f  dr=%.3f | "
                          "pos: (%.2f,%.2f)->(%.2f,%.2f) dpsi=%.2f deg",
                          vt, ve, dv, rt, re, dr, xt, yt, xe, ye, math.degrees(dpsi))

            # buffers
            self.buf_t.append(t)
            self.buf_v_truth.append(vt); self.buf_v_ekf.append(ve)
            self.buf_r_truth.append(rt); self.buf_r_ekf.append(re)

            # csv
            if self.csv_writer:
                self.csv_writer.writerow([t, xt, yt, psit, vt, rt, xe, ye, psie, ve, re, dv, dr])
                self.csv_file.flush()

    def timer_plot(self, event):
        if not self.enable_plot:
            return
        with self.lock:
            if len(self.buf_t) < 2:
                return
            # trim to window
            t_now = self.buf_t[-1]
            t_min = t_now - self.window_secs
            # find first index >= t_min
            def idx_window(buf_t):
                for i in range(len(buf_t)-1, -1, -1):
                    if buf_t[i] < t_min: return i+1
                return 0
            i0 = idx_window(self.buf_t)

            T  = list(self.buf_t)[i0:]
            Vt = list(self.buf_v_truth)[i0:]
            Ve = list(self.buf_v_ekf)[i0:]
            Rt = list(self.buf_r_truth)[i0:]
            Re = list(self.buf_r_ekf)[i0:]

        # redraw
        self.ax_v.cla(); self.ax_r.cla()
        self.ax_v.grid(True); self.ax_r.grid(True)
        self.ax_v.set_title("Linear speed (v) [m/s]")
        self.ax_r.set_title("Yaw rate (r) [rad/s]")

        self.ax_v.plot(T, Vt, label="truth v")
        self.ax_v.plot(T, Ve, label="ekf v")
        self.ax_v.legend(loc="best")

        self.ax_r.plot(T, Rt, label="truth r")
        self.ax_r.plot(T, Re, label="ekf r")
        self.ax_r.legend(loc="best")

        plt.pause(0.001)

    @staticmethod
    def wrap_angle(a):
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def spin(self):
        if self.enable_plot and HAVE_MPL:
            # keep matplotlib window responsive
            r = rospy.Rate(50)
            while not rospy.is_shutdown():
                plt.pause(0.01)
                r.sleep()
        else:
            rospy.spin()

if __name__ == "__main__":
    rospy.init_node("ekf_compare")
    node = EKFCompare()
    node.spin()
