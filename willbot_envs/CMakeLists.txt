cmake_minimum_required(VERSION 2.8.3)
project(willbot_envs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_commander
  roscpp
  rospy
  tf
  tf_conversions
)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES
  EnvReset.action
  EnvStep.action
)

add_service_files(
  DIRECTORY srv
  FILES
  Close.srv
  Init.srv
  Seed.srv
  Step.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
)
