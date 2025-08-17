#!/usr/bin/env python3
"""
Simple Forward/Backward EKF Validation Test
Tests basic velocity estimation with forward and backward motion
Plots commanded, EKF, and truth velocities for comparison
"""

import rospy
import math
import csv
import numpy as np
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Odometry

try:
    import matplotlib.pyplot as plt
    HAVE_MATPLOTLIB = True
except ImportError:
    HAVE_MATPLOTLIB = False
    rospy.logwarn("[Simple Test] matplotlib not available - no real-time plotting")

class SimpleForwardBackwardTest:
    def __init__(self):
        rospy.init_node('simple_forward_backward_test', anonymous=True)
        
        # Test parameters (configurable) - INCREASED for visibility
        self.forward_omega = rospy.get_param('~forward_omega', 8.0)     # rad/s for forward motion (INCREASED)
        self.backward_omega = rospy.get_param('~backward_omega', -6.0)  # rad/s for backward motion (INCREASED)
        self.phase_duration = rospy.get_param('~phase_duration', 5.0)   # seconds per phase (INCREASED)
        self.log_to_csv = rospy.get_param('~log_to_csv', True)
        self.csv_file = rospy.get_param('~csv_file', '/tmp/simple_ekf_test.csv')
        self.enable_plotting = rospy.get_param('~enable_plotting', True)
        
        # Vehicle parameters
        self.wheel_radius = 0.033  # m (from your config)
        
        # Publishers - EXACT topic names from your rostopic list
        self.pub_rl = rospy.Publisher('/wheelrl_motor/command', Float32, queue_size=1)
        self.pub_rr = rospy.Publisher('/wheelrr_motor/command', Float32, queue_size=1)
        self.pub_fl = rospy.Publisher('/wheelfl_motor/command', Float32, queue_size=1)
        self.pub_fr = rospy.Publisher('/wheelfr_motor/command', Float32, queue_size=1)
        
        # Steering publishers (always straight)
        self.pub_steer_l = rospy.Publisher('/qcar/base_fl_controller/command', Float64, queue_size=1)
        self.pub_steer_r = rospy.Publisher('/qcar/base_fr_controller/command', Float64, queue_size=1)
        
        # Subscribers for data collection
        self.sub_ekf = rospy.Subscriber('/qcar/ekf/odom', Odometry, self.ekf_callback)
        self.sub_truth = rospy.Subscriber('/odom', Odometry, self.truth_callback)
        
        # Data storage
        self.ekf_vx = 0.0
        self.truth_vx = 0.0
        self.commanded_vx = 0.0
        self.current_omega = 0.0
        
        # Data logging for plotting
        self.time_data = []
        self.ekf_data = []
        self.truth_data = []
        self.cmd_data = []
        self.phase_data = []
        
        # CSV logging
        self.csv_writer = None
        if self.log_to_csv:
            self.init_csv_logging()
        
        rospy.loginfo(f"[Simple Test] Configuration:")
        rospy.loginfo(f"  Forward omega: {self.forward_omega} rad/s -> {self.omega_to_velocity(self.forward_omega):.3f} m/s")
        rospy.loginfo(f"  Backward omega: {self.backward_omega} rad/s -> {self.omega_to_velocity(self.backward_omega):.3f} m/s")
        rospy.loginfo(f"  Phase duration: {self.phase_duration} seconds")
        rospy.loginfo(f"  CSV logging: {'ON' if self.log_to_csv else 'OFF'}")
        rospy.loginfo(f"  Real-time plotting: {'ON' if self.enable_plotting and HAVE_MATPLOTLIB else 'OFF'}")
        rospy.loginfo(f"[Simple Test] Motor topics: /wheelfl_motor, /wheelfr_motor, /wheelrl_motor, /wheelrr_motor")
        rospy.loginfo(f"[Simple Test] EKF topic: /qcar/ekf/odom")
        rospy.loginfo(f"[Simple Test] Truth topic: /odom")
        
        rospy.sleep(2.0)  # Wait for connections
        
    def init_csv_logging(self):
        """Initialize CSV file for data logging"""
        try:
            self.csv_file_handle = open(self.csv_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file_handle)
            
            header = ['time', 'phase', 'commanded_omega', 'commanded_vx', 'ekf_vx', 'truth_vx', 'error_vx']
            self.csv_writer.writerow(header)
            rospy.loginfo(f"[Simple Test] Logging to: {self.csv_file}")
        except Exception as e:
            rospy.logwarn(f"[Simple Test] CSV logging failed: {e}")
            self.log_to_csv = False
            
    def omega_to_velocity(self, omega):
        """Convert wheel angular velocity to forward velocity"""
        return omega * self.wheel_radius
        
    def ekf_callback(self, msg):
        """Store EKF velocity data"""
        self.ekf_vx = msg.twist.twist.linear.x
        
    def truth_callback(self, msg):
        """Store ground truth velocity data"""
        self.truth_vx = msg.twist.twist.linear.x
        
    def send_command(self, omega, delta=0.0):
        """Send commands exactly like your controller does"""
        # Wheel commands (matching your controller exactly)
        vel_cmd_l = Float32()
        vel_cmd_r = Float32()
        vel_cmd_l.data = -omega  # Note: negative for left wheels (from your controller)
        vel_cmd_r.data = omega
        
        # Send to specific motor publishers
        self.pub_rl.publish(vel_cmd_l)  # Rear left gets negative omega
        self.pub_rr.publish(vel_cmd_r)  # Rear right gets positive omega
        self.pub_fl.publish(vel_cmd_l)  # Front left gets negative omega
        self.pub_fr.publish(vel_cmd_r)  # Front right gets positive omega
        
        # Steering commands (always straight for this test)
        steer_cmd = Float64()
        steer_cmd.data = delta
        self.pub_steer_l.publish(steer_cmd)
        self.pub_steer_r.publish(steer_cmd)
        
        # Store commanded values
        self.current_omega = omega
        self.commanded_vx = self.omega_to_velocity(omega)
        
        # Debug output every few seconds
        rospy.logdebug(f"[Simple Test] Sent: omega={omega:.1f}, vx_cmd={self.commanded_vx:.3f}")
        
    def log_data(self, phase_name, current_time):
        """Log current data for analysis"""
        error_vx = self.ekf_vx - self.truth_vx
        
        # Store for plotting
        self.time_data.append(current_time)
        self.ekf_data.append(self.ekf_vx)
        self.truth_data.append(self.truth_vx)
        self.cmd_data.append(self.commanded_vx)
        self.phase_data.append(phase_name)
        
        # CSV logging
        if self.csv_writer:
            row = [current_time, phase_name, self.current_omega, self.commanded_vx, 
                   self.ekf_vx, self.truth_vx, error_vx]
            self.csv_writer.writerow(row)
            self.csv_file_handle.flush()
            
        # Console output - more frequent for debugging
        rospy.loginfo_throttle(0.5, 
            f"[{phase_name}] CMD: {self.commanded_vx:+.3f} m/s | "
            f"TRUTH: {self.truth_vx:+.3f} | EKF: {self.ekf_vx:+.3f} | "
            f"ERR: {error_vx:+.3f}")
            
    def run_phase(self, phase_name, omega, duration):
        """Run a single test phase"""
        rospy.loginfo("="*50)
        rospy.loginfo(f"[Simple Test] STARTING PHASE: {phase_name}")
        rospy.loginfo(f"  Omega command: {omega:+.1f} rad/s")
        rospy.loginfo(f"  Expected velocity: {self.omega_to_velocity(omega):+.3f} m/s")
        rospy.loginfo(f"  Duration: {duration:.1f} seconds")
        rospy.loginfo("="*50)
        
        start_time = rospy.Time.now()
        test_start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)  # 20 Hz logging
        
        # Send initial commands
        self.send_command(omega)
        rospy.sleep(0.1)  # Brief pause for command to take effect
        
        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            # Send commands continuously
            self.send_command(omega)
            
            # Log data
            current_time = rospy.Time.now().to_sec() - test_start_time
            self.log_data(phase_name, current_time)
            
            rate.sleep()
            
        rospy.loginfo(f"[Simple Test] COMPLETED PHASE: {phase_name}")
        rospy.loginfo(f"  Final EKF velocity: {self.ekf_vx:+.3f} m/s")
        rospy.loginfo(f"  Final truth velocity: {self.truth_vx:+.3f} m/s")
            
    def run_test_sequence(self):
        """Run the complete forward/backward test"""
        rospy.loginfo("[Simple Test] Starting Forward/Backward validation test...")
        
        # Wait for data connections
        rospy.loginfo("[Simple Test] Waiting for data streams...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while (self.ekf_vx == 0.0 or self.truth_vx == 0.0) and rospy.Time.now() < timeout:
            rospy.loginfo_throttle(2.0, "[Simple Test] Waiting for EKF and truth data...")
            rospy.sleep(0.1)
            
        if self.ekf_vx == 0.0:
            rospy.logerr("[Simple Test] No EKF data received! Check /qcar/ekf/odom topic")
            return
        if self.truth_vx == 0.0:
            rospy.logerr("[Simple Test] No truth data received! Check /odom topic")
            return
            
        rospy.loginfo("[Simple Test] ✅ Data streams connected!")
        
        try:
            # Phase 1: Rest (ensure starting from zero)
            self.run_phase("Initial Rest", 0.0, 2.0)
            
            # Phase 2: Forward motion
            self.run_phase("Forward Motion", self.forward_omega, self.phase_duration)
            
            # Phase 3: Stop
            self.run_phase("Stop", 0.0, 2.0)
            
            # Phase 4: Backward motion  
            self.run_phase("Backward Motion", self.backward_omega, self.phase_duration)
            
            # Phase 5: Final stop
            self.run_phase("Final Stop", 0.0, 2.0)
            
            rospy.loginfo("[Simple Test] Test sequence completed!")
            
            # Generate plot if available
            if self.enable_plotting and HAVE_MATPLOTLIB:
                self.generate_plot()
                
            # Print summary
            self.print_summary()
            
        except rospy.ROSInterruptException:
            rospy.loginfo("[Simple Test] Test interrupted")
        finally:
            # Ensure stopped
            self.send_command(0.0)
            if self.log_to_csv and hasattr(self, 'csv_file_handle'):
                self.csv_file_handle.close()
                rospy.loginfo(f"[Simple Test] Data saved to: {self.csv_file}")
                
    def generate_plot(self):
        """Generate velocity comparison plot"""
        try:
            plt.figure(figsize=(12, 8))
            
            # Convert data to numpy arrays
            times = np.array(self.time_data)
            cmd_vels = np.array(self.cmd_data)
            ekf_vels = np.array(self.ekf_data)
            truth_vels = np.array(self.truth_data)
            
            # Main velocity plot
            plt.subplot(2, 1, 1)
            plt.plot(times, cmd_vels, 'r-', linewidth=2, label='Commanded Velocity')
            plt.plot(times, truth_vels, 'g-', linewidth=2, label='Ground Truth')
            plt.plot(times, ekf_vels, 'b--', linewidth=2, label='EKF Estimate')
            
            plt.title('Forward/Backward Velocity Validation', fontsize=14, fontweight='bold')
            plt.ylabel('Velocity (m/s)', fontsize=12)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            
            # Add phase annotations
            phase_times = [0, 2, 2+self.phase_duration, 4+self.phase_duration, 
                          4+2*self.phase_duration, 6+2*self.phase_duration]
            phase_names = ['Rest', 'Forward', 'Stop', 'Backward', 'Stop']
            
            for i, (t, name) in enumerate(zip(phase_times[:-1], phase_names)):
                plt.axvline(x=t, color='gray', linestyle=':', alpha=0.7)
                if i < len(phase_names):
                    plt.text(t + 0.5, max(cmd_vels) * 0.8, name, 
                            rotation=90, fontsize=9, alpha=0.7)
            
            # Error plot
            plt.subplot(2, 1, 2)
            errors = ekf_vels - truth_vels
            plt.plot(times, errors, 'r-', linewidth=1.5, label='EKF Error')
            plt.axhline(y=0, color='black', linestyle='-', alpha=0.3)
            plt.title('EKF Estimation Error', fontsize=12)
            plt.xlabel('Time (seconds)', fontsize=12)
            plt.ylabel('Error (m/s)', fontsize=12)
            plt.legend(fontsize=10)
            plt.grid(True, alpha=0.3)
            
            plt.tight_layout()
            
            # Save plot
            plot_file = '/tmp/simple_ekf_validation_plot.png'
            plt.savefig(plot_file, dpi=150, bbox_inches='tight')
            rospy.loginfo(f"[Simple Test] Plot saved to: {plot_file}")
            
            plt.show(block=False)
            rospy.loginfo("[Simple Test] Plot displayed (close window to continue)")
            
        except Exception as e:
            rospy.logwarn(f"[Simple Test] Plotting failed: {e}")
            
    def print_summary(self):
        """Print test summary statistics"""
        if len(self.ekf_data) == 0:
            return
            
        ekf_array = np.array(self.ekf_data)
        truth_array = np.array(self.truth_data)
        errors = ekf_array - truth_array
        
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("         SIMPLE EKF TEST SUMMARY")
        rospy.loginfo("="*50)
        rospy.loginfo(f"Total test duration: {self.time_data[-1]:.1f} seconds")
        rospy.loginfo(f"Data points collected: {len(self.ekf_data)}")
        rospy.loginfo(f"")
        rospy.loginfo(f"Error Statistics:")
        rospy.loginfo(f"  Mean error:     {np.mean(errors):+.4f} m/s")
        rospy.loginfo(f"  RMS error:      {np.sqrt(np.mean(errors**2)):+.4f} m/s")
        rospy.loginfo(f"  Max error:      {np.max(np.abs(errors)):+.4f} m/s")
        rospy.loginfo(f"  Std deviation:  {np.std(errors):+.4f} m/s")
        
        # Phase-specific analysis
        rospy.loginfo(f"")
        rospy.loginfo(f"Command Tracking:")
        cmd_array = np.array(self.cmd_data)
        cmd_errors = ekf_array - cmd_array
        rospy.loginfo(f"  Mean EKF vs CMD error: {np.mean(cmd_errors):+.4f} m/s")
        rospy.loginfo(f"  RMS EKF vs CMD error:  {np.sqrt(np.mean(cmd_errors**2)):+.4f} m/s")
        
        # Pass/Fail assessment
        max_acceptable_error = 0.05  # 5cm/s
        rms_acceptable = 0.03        # 3cm/s RMS
        
        passed = (np.sqrt(np.mean(errors**2)) < rms_acceptable and 
                 np.max(np.abs(errors)) < max_acceptable_error)
        
        rospy.loginfo(f"")
        rospy.loginfo(f"Assessment (RMS < {rms_acceptable}, Max < {max_acceptable_error}):")
        rospy.loginfo(f"  Result: {'✅ PASS' if passed else '❌ FAIL'}")
        
        if not passed:
            rospy.loginfo(f"")
            rospy.loginfo(f"Tuning suggestions:")
            if np.abs(np.mean(errors)) > 0.01:
                rospy.loginfo(f"  - Large bias detected, check wheel radius calibration")
            if np.std(errors) > 0.03:
                rospy.loginfo(f"  - High noise, consider increasing process noise Q")
                
        rospy.loginfo("="*50)

if __name__ == '__main__':
    try:
        tester = SimpleForwardBackwardTest()
        tester.run_test_sequence()
        
        # Keep node alive to show plot
        if tester.enable_plotting and HAVE_MATPLOTLIB:
            rospy.loginfo("[Simple Test] Press Ctrl+C to exit...")
            rospy.spin()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("[Simple Test] Test completed")