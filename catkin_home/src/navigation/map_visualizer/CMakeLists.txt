cmake_minimum_required(VERSION 2.8.3)
project(map_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  rospy tf
  sensor_msgs
  std_msgs
  rospy
  message_generation
  geometry_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MapDisplayState.msg
  Room.msg
  MapContext.msg
  ObjInt.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  visualization_msgs
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES map_visualizer
  CATKIN_DEPENDS roscpp visualization_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
include_directories(include ${catkin_INCLUDE_DIRS})


#############
## Install ##
#############

catkin_install_python(PROGRAMS scripts/map_tagger.py scripts/map_tagger_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
