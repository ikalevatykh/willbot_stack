cmake_minimum_required(VERSION 2.8.3)
project(recorder)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  robotiq_3f_gripper_control
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
 DIRECTORY srv 
 FILES
 Command.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime control_msgs geometry_msgs robotiq_3f_gripper_control roscpp rospy sensor_msgs std_msgs
)
