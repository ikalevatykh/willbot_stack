cmake_minimum_required(VERSION 2.8.3)
project(willbot_recognition)

find_package(catkin REQUIRED COMPONENTS
  object_recognition_core
  object_recognition_msgs
  object_recognition_tabletop
  roscpp
  rospy
)

catkin_package()
