cmake_minimum_required(VERSION 3.0.2)
project(qcar_visnav)

add_compile_options(-std=c++14)

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

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs nav_msgs tf2_ros geometry_msgs tf
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Minimal EKF sources - only what we need
set(QCAR_VISNAV_SRCS
  src/estimation/model/kinematic_model.cpp
  src/estimation/ekf/ekf_core.cpp
  src/estimation/sensors/gyro.cpp
  src/estimation/sensors/speed.cpp
  src/estimation/utils/odometry_buffer.cpp
  src/estimation/node/ekf_node.cpp
)

add_executable(ekf_node ${QCAR_VISNAV_SRCS})

set_target_properties(ekf_node PROPERTIES OUTPUT_NAME ekf_node PREFIX "")

add_dependencies(ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# Install
install(TARGETS ekf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/simple_motion_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)