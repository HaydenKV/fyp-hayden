cmake_minimum_required(VERSION 3.0.2)
project(qcar_visnav)

## =========================================================
## Build options / language
## =========================================================
add_compile_options(-std=c++14)

## =========================================================
## Dependencies
## =========================================================
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
  topic_tools
  image_transport
  cv_bridge
  image_geometry
)

# Eigen (header-only in Noetic; use include dirs instead of a target)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

## =========================================================
## Messages
## =========================================================
add_message_files(
  FILES
  Cone.msg
  ConeArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## =========================================================
## Catkin package
## =========================================================
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
    image_transport
    cv_bridge
    image_geometry
  DEPENDS Eigen3
)

## =========================================================
## Include paths
## =========================================================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## =========================================================
## Sources / Targets
## =========================================================
# --- Estimation EKF
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
add_dependencies(ekf_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ekf_node
  ${catkin_LIBRARIES}
)

# --- Perception nodes
add_executable(cone_detector_node
  src/perception/cone_detector_node.cpp
)
add_dependencies(cone_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cone_detector_node
  ${catkin_LIBRARIES}
)

add_executable(cones_viz_node
  src/perception/cones_viz_node.cpp
)
add_dependencies(cones_viz_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cones_viz_node
  ${catkin_LIBRARIES}
)

add_executable(cone_tracker_node
  src/perception/cone_tracker_node.cpp
)
add_dependencies(cone_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cone_tracker_node
  ${catkin_LIBRARIES}
)

# -- colour detection node
add_executable(cone_colour_node
  src/perception/cone_colour_node.cpp
)
add_dependencies(cone_colour_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cone_colour_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


# --- SLAM (FastSLAM2) node
set(SLAM_SRCS
  src/slam/fastslam2.cpp
  src/slam/slam_node.cpp
  src/slam/motion_model.cpp
  src/slam/measurement_model.cpp
  src/slam/data_association.cpp
  src/slam/resampling.cpp
  src/slam/tf_utils.cpp
)

add_executable(slam_node ${SLAM_SRCS})
add_dependencies(slam_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(slam_node
  ${catkin_LIBRARIES}
)

# --- FastSLAM viz node (publish layered markers to /slam/map_markers)
add_executable(fastslam2_viz_node
  src/slam/fastslam2_viz_node.cpp
)
add_dependencies(fastslam2_viz_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fastslam2_viz_node
  ${catkin_LIBRARIES}
)


## =========================================================
## Installation
## =========================================================
install(TARGETS
  ekf_node
  cone_detector_node
  cones_viz_node
  cone_tracker_node
  slam_node
  fastslam2_viz_node
  cone_colour_node
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

# (Optional) install RViz configs if you add any later
# install(DIRECTORY rviz/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
#   FILES_MATCHING PATTERN "*.rviz"
# )

## =========================================================
## Python scripts
## =========================================================
catkin_install_python(PROGRAMS
  scripts/simple_forward_backward_test.py
  scripts/show_rgbd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
