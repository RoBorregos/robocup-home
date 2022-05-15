cmake_minimum_required(VERSION 2.8.3)
project(object_manipulation)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  roscpp
  roslib
  rospy
  pcl_ros
  pcl_conversions
  std_msgs
  std_srvs
  message_generation
  tf
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  laser_geometry
)


generate_dynamic_reconfigure_options(
  cfg/SphericalGrasp.cfg
)

add_action_files(
  FILES
  PickUpPose.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    roscpp
    roslib
    message_runtime
    moveit_core
    moveit_ros_planning_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=shadow")

install(PROGRAMS
        scripts/pick_and_place_server.py
        scripts/pick_client.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

