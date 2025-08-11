cmake_minimum_required(VERSION 3.0.2)
project(qcar_visnav)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs          # ADDED: we publish nav_msgs/Odometry
  tf
  tf2_ros           # ADDED: we broadcast TF
  geometry_msgs     # ADDED: used by TF message
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)   # ADDED: EKF uses Eigen

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## (unchanged; none generated here)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include              # ADDED: expose public headers
  # LIBRARIES qcar_visnav
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs nav_msgs tf2_ros geometry_msgs tf
  DEPENDS Eigen3                    # ADDED: Eigen is a system dep
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include                           # ADDED: our public headers live here
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}            # ADDED
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/qcar_visnav.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

# ADDED: EKF node and sources
set(QCAR_VISNAV_SRCS
  src/estimation/model/kinematic_model.cpp
  src/estimation/integrators/rk4sde.cpp
  src/estimation/ekf/ekf_core.cpp
  src/estimation/sensors/gyro.cpp
  src/estimation/node/ekf_node.cpp
  src/estimation/sensors/speed.cpp
)

add_executable(qcar_ekf_node ${QCAR_VISNAV_SRCS})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(qcar_ekf_node PROPERTIES OUTPUT_NAME ekf_node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(qcar_ekf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(qcar_ekf_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS qcar_ekf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
