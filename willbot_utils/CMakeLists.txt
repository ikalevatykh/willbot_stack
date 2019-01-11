cmake_minimum_required(VERSION 2.8.3)
project(willbot_utils)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  roscpp
  rospy
  tf
  tf_conversions
)

catkin_python_setup()

catkin_package()