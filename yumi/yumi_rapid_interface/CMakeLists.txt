cmake_minimum_required(VERSION 3.0.2)
project(yumi_rapid_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  # rospy_message_converter
  abb_robot_msgs
  abb_rapid_msgs
  abb_rapid_sm_addin_msgs
  # std_msgs
  message_generation
)

catkin_python_setup()

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   PickPlacePoints.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  abb_robot_msgs
  abb_rapid_msgs
  abb_rapid_sm_addin_msgs
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
  rospy
  # rospy_message_converter 
  abb_robot_driver
  abb_robot_msgs
  abb_rapid_msgs
  abb_rapid_sm_addin_msgs
  # LIBRARIES ${PROJECT_NAME}
)

catkin_install_python(PROGRAMS
  nodes/yumi_pp_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)