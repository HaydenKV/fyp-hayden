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
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

## --- Messages (NEW) ---
add_message_files(
  FILES
  Cone.msg
  ConeArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    nav_msgs
    geometry_msgs
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    visualization_msgs
    message_runtime
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Minimal EKF sources - only what we need  (YOUR ORIGINAL SECTION)
set(QCAR_VISNAV_SRCS
  src/estimation/model/kinematic_model.cpp
  src/estimation/ekf/ekf_core.cpp
  src/estimation/sensors/gyro.cpp
  src/estimation/sensors/speed.cpp
  src/estimation/utils/odometry_buffer.cpp
  src/estimation/node/ekf_node.cpp
  src/estimation/sensors/accelerometer.cpp
)

add_executable(ekf_node ${QCAR_VISNAV_SRCS})
set_target_properties(ekf_node PROPERTIES OUTPUT_NAME ekf_node PREFIX "")
add_dependencies(ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

## --- New nodes (stubs) ---
add_executable(cone_detector_node
  src/perception/cone_detector_node.cpp
)
add_dependencies(cone_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cone_detector_node ${catkin_LIBRARIES})

add_executable(fastslam2_node
  src/slam/fastslam2_node.cpp
)
add_dependencies(fastslam2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fastslam2_node ${catkin_LIBRARIES})

add_executable(cones_viz_node src/perception/cones_viz_node.cpp)
add_dependencies(cones_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cones_viz_node ${catkin_LIBRARIES})

# --- NEW: cone_tracker_node ---
add_executable(cone_tracker_node
  src/perception/cone_tracker_node.cpp
)
add_dependencies(cone_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cone_tracker_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# Install
install(TARGETS ekf_node cone_detector_node fastslam2_node cones_viz_node cone_tracker_node
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
  scripts/simple_forward_backward_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
