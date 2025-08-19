cmake_minimum_required(VERSION 3.0.2)
project(qcar_supervisor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  std_msgs 
  message_filters
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES model_lib ekf_lib jmf_lib
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf std_msgs message_filters
  DEPENDS Eigen3
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


# ----------------------------------------------------------------------------
# Model library
add_library(model_lib
  src/model/kinematic_model.cpp
)
target_link_libraries(model_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# ----------------------------------------------------------------------------
# EKF library
add_library(ekf_lib
  src/EKF/ekf.cpp
)

target_link_libraries(ekf_lib
  model_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# JMF library
add_library(jmf_lib
  src/JMF/JMF.cpp
)

target_link_libraries(jmf_lib
  ekf_lib
  model_lib
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# ----------------------------------------------------------------------------
# Nodes
add_executable(sup_ekf_node src/nodes/sup_ekf_node.cpp)
target_link_libraries(sup_ekf_node
  ekf_lib
  ${catkin_LIBRARIES}
)

add_executable(sup_jmf_node src/nodes/sup_jmf_node.cpp)
target_link_libraries(sup_jmf_node
  jmf_lib
  model_lib
  ${catkin_LIBRARIES}
)

# ----------------------------------------------------------------------------
#  Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(ekf_test src/tests/ekf_test.cpp)
  target_link_libraries(ekf_test ekf_lib)

  #catkin_add_gtest(jmf_test src/tests/jmf_test.cpp)
  #target_link_libraries(jmf_test jmf_lib model_lib)
endif()

