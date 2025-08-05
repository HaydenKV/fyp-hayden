#!/bin/bash

# Quick EKF Check Script
# Run this in a separate terminal after launching your EKF

echo " QCar EKF Quick Check"
echo "======================="

echo ""
echo " Checking ROS topics..."
if rostopic list | grep -q "qcar_visnav"; then
    echo " EKF topics found"
else
    echo " No EKF topics - is estimator_node running?"
fi

echo ""
echo " Data rates:"
echo "EKF output rate:"
timeout 5 rostopic hz /qcar_visnav/estimator_state 2>/dev/null || echo " No EKF data"

echo ""
echo "IMU input rate:"  
timeout 5 rostopic hz /qcar/imu 2>/dev/null || echo " No IMU data"

echo ""
echo " Current EKF state (5 samples):"
timeout 10 rostopic echo -n5 /qcar_visnav/estimator_state 2>/dev/null || echo " No EKF state data"

echo ""
echo " Current IMU data (2 samples):"
timeout 5 rostopic echo -n2 /qcar/imu/linear_acceleration 2>/dev/null || echo " No IMU acceleration data"

echo ""
echo " Ground truth position (2 samples):"
timeout 5 rostopic echo -n2 /qcar/ground_truth/state/pose/pose/position 2>/dev/null || echo " No ground truth data"

echo ""
echo " Check complete!"
echo ""
echo "If you see data above, your EKF is working!"
echo "If not, check that estimator_node is running and connected properly."