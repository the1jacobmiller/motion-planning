cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dbw_mkz_msgs
  autoware_msgs
  tf
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################



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
#  INCLUDE_DIRS include
#  LIBRARIES motion_planner
  CATKIN_DEPENDS roscpp rospy std_msgs autoware_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/motion_planner.cpp
# )

add_executable(motion_planner_node src/motion_planner.cpp src/occupancy_distribution src/motion_prediction.cpp src/visualizer.cpp src/util.cpp)
target_link_libraries(motion_planner_node ${catkin_LIBRARIES})
add_dependencies(motion_planner_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
