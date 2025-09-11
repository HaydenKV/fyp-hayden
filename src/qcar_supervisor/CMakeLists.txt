cmake_minimum_required(VERSION 3.0.2)
project(qcar_supervisor)

## —————————————————————————————————————————————————————————————
## Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  std_msgs
  message_filters
)
find_package(Eigen3 REQUIRED)

## —————————————————————————————————————————————————————————————
## Path to the *visnav* folder in your workspace:
## (Assumes your layout is ~/fyp-hayden/src/qcar_visnav)
set(QCAR_VISNAV_PATH    ${PROJECT_SOURCE_DIR}/../qcar_visnav)
set(QCAR_VISNAV_INC_DIR    ${QCAR_VISNAV_PATH}/include)

## —————————————————————————————————————————————————————————————
## Declare this catkin package
catkin_package(
  INCLUDE_DIRS
    include
    ${QCAR_VISNAV_INC_DIR}
  LIBRARIES
    model_lib
    jmf_lib
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    tf
    std_msgs
    message_filters
# We do NOT list Eigen3 here – it’s handled by find_package + include_directories
)

## —————————————————————————————————————————————————————————————
## Includes
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${QCAR_VISNAV_INC_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## —————————————————————————————————————————————————————————————
## Model library
# add_library(model_lib
#   src/model/kinematic_model.cpp
# )
# target_link_libraries(model_lib
#   ${catkin_LIBRARIES}
#   Eigen3::Eigen
# )
# # Add namespace definition for model_lib
# target_compile_definitions(model_lib PUBLIC
#   QCAR_NAV_NAMESPACE=qcar_nav
# )

## —————————————————————————————————————————————————————————————
## Build the VisNav EKF  and model
file(GLOB VISNAV_EKF_SRCS
  ${QCAR_VISNAV_PATH}/src/estimation/ekf/ekf_core.cpp
  ${QCAR_VISNAV_PATH}/src/estimation/model/kinematic_model.cpp
)

add_library(qcar_visnav_lib
  ${VISNAV_EKF_SRCS}
)

target_include_directories(qcar_visnav_lib PUBLIC
  ${QCAR_VISNAV_INC_DIR}
)

target_link_libraries(qcar_visnav_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

## —————————————————————————————————————————————————————————————
## Install Python scripts via catkin helper
catkin_install_python(PROGRAMS
  scripts/plot_mode_probs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## —————————————————————————————————————————————————————————————
## JMF library
add_library(jmf_lib
  src/JMF.cpp
  src/transition_model.cpp
  src/measurement_accel/measurement_accel.cpp
  src/measurement_velocity.cpp
  src/measurement_gyro.cpp
)

target_compile_definitions(jmf_lib PUBLIC
  USING_QCAR_NAV_MODEL
)

target_link_libraries(jmf_lib
  qcar_visnav_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

## —————————————————————————————————————————————————————————————
## Node executables
## JMF NODE
add_executable(sup_jmf_node
  src/nodes/sup_jmf_node.cpp
)
target_link_libraries(sup_jmf_node
  jmf_lib
  qcar_visnav_lib
  ${catkin_LIBRARIES}
)