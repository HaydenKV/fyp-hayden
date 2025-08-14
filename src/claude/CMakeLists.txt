cmake_minimum_required(VERSION 3.0.2)
project(qcar_visnav)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2_ros
)

## System dependencies
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs nav_msgs tf2_ros geometry_msgs tf
  DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Velocity EKF node and sources
set(QCAR_VISNAV_SRCS
  # Core model and EKF
  src/estimation/model/kinematic_model.cpp
  src/estimation/integrators/rk4sde.cpp
  src/estimation/ekf/ekf_core.cpp
  
  # Sensor classes
  src/estimation/sensors/gyro.cpp
  src/estimation/sensors/speed.cpp
  src/estimation/sensors/pseudo_measurements.cpp
  
  # Utilities
  src/estimation/utils/odometry_buffer.cpp
  
  # Main node
  src/estimation/node/ekf_node.cpp
)

add_executable(ekf_node ${QCAR_VISNAV_SRCS})

## Rename executable for clarity
set_target_properties(ekf_node PROPERTIES OUTPUT_NAME ekf_node PREFIX "")

## Add cmake target dependencies
add_dependencies(ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link
target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ekf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/tests/ekf_compare.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)