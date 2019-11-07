cmake_minimum_required(VERSION 2.8.3)
project(ca_swarm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  amcl
  move_base
  move_base_msgs
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  geometry_msgs
  rospy
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/swarm_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
