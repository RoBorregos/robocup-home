cmake_minimum_required(VERSION 3.1.3)
project(robot_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  hardware_interface
  controller_manager
  xarm_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy hardware_interface controller_manager
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(MyRobot src/MyRobot.cpp)
target_link_libraries(MyRobot ${catkin_LIBRARIES})
add_dependencies(MyRobot ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
