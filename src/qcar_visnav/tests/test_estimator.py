#!/usr/bin/env python3

import rospy
import numpy as np
from qcar_visnav.msg import EstimatorState, EventDebug
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt

class EstimatorTester:
    def __init__(self):
        rospy.init_node('estimator_tester')
        
        # Data storage
        self.states = []
        self.times = []
        self.event_stats = []
        
        # Subscribers
        rospy.Subscriber('/qcar_visnav/estimator_state', EstimatorState, self.state_callback)
        rospy.Subscriber('/qcar_visnav/event_debug', EventDebug, self.debug_callback)
        
        rospy.loginfo("Estimator tester initialized")
        
    def state_callback(self, msg):
        """Store state data for analysis"""
        self.states.append([msg.u, msg.v, msg.r, msg.N, msg.E, msg.psi])
        self.times.append(rospy.get_time())
        
        # Print key states occasionally
        if len(self.states) % 50 == 0:
            rospy.loginfo("State [%d]: u=%.3f, v=%.3f, N=%.2f, E=%.2f, psi=%.3f", 
                         len(self.states), msg.u, msg.v, msg.N, msg.E, msg.psi)
    
    def debug_callback(self, msg):
        """Monitor event processing"""
        self.event_stats.append({
            'time': msg.current_time,
            'queue_size': msg.queue_size,
            'events_processed': msg.events_processed
        })
        
        if msg.queue_size % 100 == 0:
            rospy.loginfo("Event queue: size=%d, time=%.3f", msg.queue_size, msg.current_time)
    
    def plot_results(self):
        """Plot estimation results"""
        if len(self.states) < 10:
            rospy.logwarn("Not enough data to plot")
            return
            
        states = np.array(self.states)
        times = np.array(self.times) - self.times[0]  # Relative time
        
        plt.figure(figsize=(12, 8))
        
        # Velocity plot
        plt.subplot(2, 2, 1)
        plt.plot(times, states[:, 0], label='u (forward)')
        plt.plot(times, states[:, 1], label='v (lateral)')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.title('Vehicle Velocities')
        
        # Position plot
        plt.subplot(2, 2, 2)
        plt.plot(states[:, 4], states[:, 3], 'b-', label='EKF trajectory')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.legend()
        plt.title('Vehicle Trajectory')
        plt.axis('equal')
        
        # Yaw rate and heading
        plt.subplot(2, 2, 3)
        plt.plot(times, states[:, 2], label='r (yaw rate)')
        plt.xlabel('Time (s)')
        plt.ylabel('Yaw rate (rad/s)')
        plt.legend()
        plt.title('Yaw Rate')
        
        plt.subplot(2, 2, 4)
        plt.plot(times, states[:, 5], label='ψ (heading)')
        plt.xlabel('Time (s)')
        plt.ylabel('Heading (rad)')
        plt.legend()
        plt.title('Heading Angle')
        
        plt.tight_layout()
        plt.savefig('/tmp/ekf_results.png')
        rospy.loginfo("Results saved to /tmp/ekf_results.png")

if __name__ == '__main__':
    tester = EstimatorTester()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Plotting results...")
        tester.plot_results()
        rospy.loginfo("Testing complete")