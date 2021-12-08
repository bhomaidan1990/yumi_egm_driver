cmake_minimum_required(VERSION 3.0.2)
project(yumi_motion_api)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rospy_message_converter
)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(
  FILES
  YuMiJointsCmd.srv
  YuMiPoseCmd.srv
  YuMiPickPlace.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  rospy 
  std_msgs 
  tf 
  geometry_msgs 
  sensor_msgs 
  moveit_core 
  moveit_ros_planning_interface 
  rospy_message_converter 
  abb_robot_driver
  # LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  scripts/utils.py
  nodes/yumi_motion_api_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)