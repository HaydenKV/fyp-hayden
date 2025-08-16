#!/usr/bin/env python3
"""
Simple motion test script for QCar EKF validation
Sends basic forward/turning commands to test velocity estimation
"""

import rospy
import math
from std_msgs.msg import Float32, Float64

class SimpleMotionTest:
    def __init__(self):
        rospy.init_node('simple_motion_test', anonymous=True)
        
        # Publishers for wheel motors
        self.pub_fl = rospy.Publisher('/wheelfl_motor/command', Float32, queue_size=1)
        self.pub_fr = rospy.Publisher('/wheelfr_motor/command', Float32, queue_size=1)
        self.pub_rl = rospy.Publisher('/wheelrl_motor/command', Float32, queue_size=1)
        self.pub_rr = rospy.Publisher('/wheelfr_motor/command', Float32, queue_size=1)
        
        # Publishers for steering
        self.pub_steer_l = rospy.Publisher('/qcar/base_fl_controller/command', Float64, queue_size=1)
        self.pub_steer_r = rospy.Publisher('/qcar/base_fr_controller/command', Float64, queue_size=1)
        
        rospy.loginfo("[Motion Test] Simple motion test node started")
        rospy.loginfo("[Motion Test] Will send motion commands in 3 seconds...")
        rospy.sleep(3.0)
        
    def send_wheel_command(self, left_vel, right_vel):
        """Send velocity commands to wheels (rad/s)"""
        cmd_l = Float32()
        cmd_r = Float32()
        cmd_l.data = left_vel
        cmd_r.data = right_vel
        
        self.pub_fl.publish(cmd_l)
        self.pub_fr.publish(cmd_r) 
        self.pub_rl.publish(cmd_l)
        self.pub_rr.publish(cmd_r)
        
    def send_steering_command(self, angle):
        """Send steering angle command (rad)"""
        cmd = Float64()
        cmd.data = angle
        self.pub_steer_l.publish(cmd)
        self.pub_steer_r.publish(cmd)
        
    def run_motion_sequence(self):
        """Run a sequence of motions to test EKF"""
        rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("[Motion Test] Starting motion sequence...")
        
        # Phase 1: Forward motion (5 seconds)
        rospy.loginfo("[Motion Test] Phase 1: Forward motion")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 5.0:
            self.send_wheel_command(2.0, 2.0)  # 2 rad/s forward
            self.send_steering_command(0.0)    # Straight
            rate.sleep()
            
        # Phase 2: Stop (2 seconds)
        rospy.loginfo("[Motion Test] Phase 2: Stop")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.send_wheel_command(0.0, 0.0)  # Stop
            self.send_steering_command(0.0)    # Straight
            rate.sleep()
            
        # Phase 3: Turn left while moving (3 seconds)
        rospy.loginfo("[Motion Test] Phase 3: Turn left")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 3.0:
            self.send_wheel_command(1.5, 1.5)     # Slow forward
            self.send_steering_command(0.3)       # Left turn
            rate.sleep()
            
        # Phase 4: Turn right while moving (3 seconds)  
        rospy.loginfo("[Motion Test] Phase 4: Turn right")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 3.0:
            self.send_wheel_command(1.5, 1.5)     # Slow forward
            self.send_steering_command(-0.3)      # Right turn
            rate.sleep()
            
        # Phase 5: Final stop
        rospy.loginfo("[Motion Test] Phase 5: Final stop")
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 2.0:
            self.send_wheel_command(0.0, 0.0)  # Stop
            self.send_steering_command(0.0)    # Straight
            rate.sleep()
            
        rospy.loginfo("[Motion Test] Motion sequence complete!")
        
        # Keep publishing zeros to maintain stop
        while not rospy.is_shutdown():
            self.send_wheel_command(0.0, 0.0)
            self.send_steering_command(0.0)
            rate.sleep()

if __name__ == '__main__':
    try:
        tester = SimpleMotionTest()
        tester.run_motion_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("[Motion Test] Shutting down motion test")