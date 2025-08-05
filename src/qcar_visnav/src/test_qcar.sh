#!/bin/bash

# QCar Testing Script
# Comprehensive testing for the UON-QCAR-BASE project

echo "=========================================="
echo "QCar Testing Suite"
echo "=========================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a process is running
check_process() {
    if pgrep -f "$1" > /dev/null; then
        return 0
    else
        return 1
    fi
}

# Function to wait for ROS to be ready
wait_for_ros() {
    print_status "Waiting for ROS to be ready..."
    until rostopic list &> /dev/null; do
        sleep 1
    done
    print_status "ROS is ready!"
}

# Test 1: Build Test
test_build() {
    print_status "Testing workspace build..."
    cd ~/catkin_ws
    if catkin_make; then
        print_status "Build successful!"
        source devel/setup.bash
        return 0
    else
        print_error "Build failed!"
        return 1
    fi
}

# Test 2: Basic Launch Test
test_basic_launch() {
    print_status "Testing basic Track1 example..."
    
    # Launch in background
    roslaunch qcar_gazebo Track1Example.launch &
    LAUNCH_PID=$!
    
    # Wait for ROS to be ready
    sleep 10
    wait_for_ros
    
    # Check if key topics exist
    if rostopic list | grep -q "/qcar"; then
        print_status "Basic launch test passed!"
        
        # Check specific topics
        print_status "Checking key topics..."
        rostopic echo -n1 /qcar/joint_states &> /dev/null && print_status "Joint states topic OK"
        rostopic echo -n1 /qcar/ground_truth/state &> /dev/null && print_status "Ground truth topic OK"
        
        kill $LAUNCH_PID
        sleep 5
        return 0
    else
        print_error "Basic launch test failed!"
        kill $LAUNCH_PID
        sleep 5
        return 1
    fi
}

# Test 3: Navigation System Test  
test_navigation_system() {
    print_status "Testing full navigation system..."
    
    # Launch navigation system
    roslaunch qcar_visnav navigation_system.launch debug:=true simulation_time:=30.0 &
    NAV_PID=$!
    
    sleep 15
    wait_for_ros
    
    # Check if estimator is running
    if rostopic list | grep -q "/qcar_visnav/estimator_state"; then
        print_status "Navigation system test passed!"
        
        # Test estimator output
        print_status "Checking estimator output..."
        if timeout 5 rostopic echo -n1 /qcar_visnav/estimator_state &> /dev/null; then
            print_status "Estimator publishing data OK"
        else
            print_warning "Estimator not publishing data"
        fi
        
        kill $NAV_PID
        sleep 5
        return 0
    else
        print_error "Navigation system test failed!"
        kill $NAV_PID
        sleep 5
        return 1
    fi
}

# Test 4: Individual Node Tests
test_individual_nodes() {
    print_status "Testing individual nodes..."
    
    # Launch basic track first
    roslaunch qcar_gazebo Track1Example.launch &
    TRACK_PID=$!
    sleep 10
    wait_for_ros
    
    # Test guidance node
    print_status "Testing guidance node..."
    timeout 10 rosrun qcar_guidance example_guidance_node.py &
    GUIDANCE_PID=$!
    sleep 3
    
    if check_process "example_guidance_node.py"; then
        print_status "Guidance node OK"
    else
        print_warning "Guidance node not running"
    fi
    
    # Test estimator node
    print_status "Testing estimator node..."
    timeout 10 rosrun qcar_visnav estimator_node &
    ESTIMATOR_PID=$!
    sleep 3
    
    if check_process "estimator_node"; then
        print_status "Estimator node OK"
    else
        print_warning "Estimator node not running"
    fi
    
    # Cleanup
    kill $GUIDANCE_PID $ESTIMATOR_PID $TRACK_PID 2>/dev/null
    sleep 5
}

# Test 5: Topic Verification
test_topics() {
    print_status "Testing topic data flow..."
    
    # Launch system
    roslaunch qcar_visnav navigation_system.launch &
    SYSTEM_PID=$!
    sleep 15
    wait_for_ros
    
    # Check topic rates
    print_status "Checking topic rates..."
    
    # IMU should be ~100Hz
    IMU_RATE=$(timeout 10 rostopic hz /qcar/imu 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$IMU_RATE" ]; then
        print_status "IMU rate: $IMU_RATE Hz"
    else
        print_warning "Could not measure IMU rate"
    fi
    
    # Estimator should be ~50Hz
    EST_RATE=$(timeout 10 rostopic hz /qcar_visnav/estimator_state 2>/dev/null | grep "average rate" | awk '{print $3}')
    if [ ! -z "$EST_RATE" ]; then
        print_status "Estimator rate: $EST_RATE Hz"
    else
        print_warning "Could not measure estimator rate"
    fi
    
    kill $SYSTEM_PID
    sleep 5
}

# Test 6: Parameter Testing
test_parameters() {
    print_status "Testing different parameter configurations..."
    
    # Test different initial positions
    print_status "Testing custom initial position..."
    roslaunch qcar_visnav navigation_system.launch x:=-2.0 y:=1.0 yaw:=1.57 simulation_time:=15.0 &
    PARAM_PID=$!
    sleep 15
    
    if rostopic echo -n1 /qcar/ground_truth/state &> /dev/null; then
        print_status "Parameter test passed!"
    else
        print_warning "Parameter test had issues"
    fi
    
    kill $PARAM_PID
    sleep 5
}

# Main execution
main() {
    print_status "Starting QCar testing suite..."
    
    # Kill any existing ROS processes
    pkill -f ros
    pkill -f gazebo
    sleep 3
    
    TESTS_PASSED=0
    TOTAL_TESTS=6
    
    # Run tests
    if test_build; then ((TESTS_PASSED++)); fi
    if test_basic_launch; then ((TESTS_PASSED++)); fi
    if test_navigation_system; then ((TESTS_PASSED++)); fi
    test_individual_nodes && ((TESTS_PASSED++))
    test_topics && ((TESTS_PASSED++))
    test_parameters && ((TESTS_PASSED++))
    
    # Final cleanup
    pkill -f ros
    pkill -f gazebo
    
    # Report results
    echo "=========================================="
    print_status "Testing Complete!"
    print_status "Tests Passed: $TESTS_PASSED/$TOTAL_TESTS"
    
    if [ $TESTS_PASSED -eq $TOTAL_TESTS ]; then
        print_status "All tests passed! Your QCar system is working correctly."
    elif [ $TESTS_PASSED -gt $((TOTAL_TESTS/2)) ]; then
        print_warning "Most tests passed. Check warnings above for minor issues."
    else
        print_error "Several tests failed. Please review the output above."
    fi
    echo "=========================================="
}

# Run main function
main "$@"