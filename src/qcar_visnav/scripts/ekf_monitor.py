#!/usr/bin/env python3
"""
Simple EKF Monitor
Shows your EKF is working correctly
"""

import rospy
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
# Adjust this import to match your actual message type
try:
    from qcar_visnav.msg import EstimatorState
except ImportError:
    # If you don't have a custom message, we'll just monitor topics
    EstimatorState = None

class EKFMonitor:
    def __init__(self):
        rospy.init_node('ekf_monitor')
        
        # Data storage
        self.estimator_data = None
        self.ground_truth_data = None
        self.imu_data = None
        
        # Counters for rates
        self.estimator_count = 0
        self.imu_count = 0
        self.last_time = time.time()
        
        # Subscribers - adjust topic names to match your setup
        if EstimatorState:
            rospy.Subscriber('/qcar_visnav/estimator_state', EstimatorState, self.estimator_callback)
        rospy.Subscriber('/qcar/ground_truth/state', Odometry, self.ground_truth_callback)
        rospy.Subscriber('/qcar/imu', Imu, self.imu_callback)
        
        print("🚗 EKF Monitor Started")
        print("=" * 60)
        
    def estimator_callback(self, msg):
        self.estimator_data = msg
        self.estimator_count += 1
        
    def ground_truth_callback(self, msg):
        self.ground_truth_data = msg
        
    def imu_callback(self, msg):
        self.imu_data = msg
        self.imu_count += 1
        
    def print_status(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt < 2.0:  # Update every 2 seconds
            return
            
        print("\n" + "="*60)
        print(f"EKF Status - {time.strftime('%H:%M:%S')}")
        print("="*60)
        
        # Data rates
        estimator_rate = self.estimator_count / dt if dt > 0 else 0
        imu_rate = self.imu_count / dt if dt > 0 else 0
        
        print(f" DATA RATES:")
        print(f"   EKF Output:    {estimator_rate:.1f} Hz")
        print(f"   IMU Input:     {imu_rate:.1f} Hz")
        
        # EKF State
        if self.estimator_data and hasattr(self.estimator_data, 'N'):
            print(f"\n EKF STATE:")
            print(f"   Position (N,E): ({self.estimator_data.N:.2f}, {self.estimator_data.E:.2f}) m")
            print(f"   Velocity (u,v): ({self.estimator_data.u:.2f}, {self.estimator_data.v:.2f}) m/s")
            print(f"   Heading (ψ):    {self.estimator_data.psi:.3f} rad ({self.estimator_data.psi*57.3:.1f}°)")
            print(f"   Yaw rate (r):   {self.estimator_data.r:.3f} rad/s")
            
            # Check for issues
            issues = []
            if abs(self.estimator_data.N) > 1000 or abs(self.estimator_data.E) > 1000:
                issues.append("Large position values")
            if abs(self.estimator_data.u) > 50 or abs(self.estimator_data.v) > 50:
                issues.append("Large velocity values")
                
            if issues:
                print(f"     Issues: {', '.join(issues)}")
            else:
                print(f"    State looks healthy")
        else:
            print(f"\n NO EKF DATA - Check estimator node")
            
        # Ground Truth Comparison
        if self.estimator_data and self.ground_truth_data and hasattr(self.estimator_data, 'N'):
            gt_x = self.ground_truth_data.pose.pose.position.x
            gt_y = self.ground_truth_data.pose.pose.position.y
            
            pos_error = ((self.estimator_data.N - gt_y)**2 + (self.estimator_data.E - gt_x)**2)**0.5
            
            print(f"\n ACCURACY:")
            print(f"   Ground Truth:   ({gt_x:.2f}, {gt_y:.2f}) m")
            print(f"   Position Error: {pos_error:.3f} m")
            
            if pos_error < 0.2:
                print(f"    Good accuracy")
            elif pos_error < 0.5:
                print(f"     Moderate error")
            else:
                print(f"    Large error")
                
        # IMU Status
        if self.imu_data:
            accel_mag = (self.imu_data.linear_acceleration.x**2 + 
                        self.imu_data.linear_acceleration.y**2 + 
                        self.imu_data.linear_acceleration.z**2)**0.5
            print(f"\n IMU STATUS:")
            print(f"   Acceleration: ({self.imu_data.linear_acceleration.x:.2f}, "
                  f"{self.imu_data.linear_acceleration.y:.2f}, "
                  f"{self.imu_data.linear_acceleration.z:.2f}) m/s²")
            print(f"   Magnitude:    {accel_mag:.2f} m/s²")
            
            if 8 < accel_mag < 12:
                print(f"    Reasonable acceleration")
            else:
                print(f"     Check accelerometer")
        else:
            print(f"\n NO IMU DATA")
            
        # Reset counters
        self.estimator_count = 0
        self.imu_count = 0
        self.last_time = current_time
        
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        print("Waiting for data...")
        
        while not rospy.is_shutdown():
            self.print_status()
            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = EKFMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        print("\n Monitor stopped")