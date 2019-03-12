cmake_minimum_required(VERSION 2.8.3)
project(willbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  robotiq_3f_gripper_control
  ur_modern_driver
  willbot_description
  freenect_launch
  kinect2_bridge
)

catkin_package(
)
