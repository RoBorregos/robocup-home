cmake_minimum_required(VERSION 3.1.3)
project(dashgo_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hardware_interface
  controller_manager
  xarm_msgs
  tf  
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy hardware_interface controller_manager
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(MyController src/MyController.cpp)
target_link_libraries(MyController ${catkin_LIBRARIES})
add_dependencies(MyController ${catkin_EXPORTED_TARGETS})

add_executable(OctomapInput src/OctomapInput.cpp)
target_link_libraries(OctomapInput ${catkin_LIBRARIES})
add_dependencies(OctomapInput ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
