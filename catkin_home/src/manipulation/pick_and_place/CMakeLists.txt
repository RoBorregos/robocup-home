cmake_minimum_required(VERSION 2.8.3)
project(pick_and_place)

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
  gpd_ros
)

add_service_files(
  FILES
  EnableOctomap.srv
)

add_message_files(
  FILES
  img.msg
  img_list.msg
  move.msg
)

add_action_files(
  FILES
  PickAndPlace.action
  manipulationServ.action
)

generate_messages(
  DEPENDENCIES
  gpd_ros
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
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(test_c scripts/test_c.cpp)
target_link_libraries(test_c ${catkin_LIBRARIES})
add_dependencies(test_c ${catkin_EXPORTED_TARGETS})

#add_executable(test_place scripts/test_place.cpp)
#target_link_libraries(test_place ${catkin_LIBRARIES})
#add_dependencies(test_place ${catkin_EXPORTED_TARGETS})
