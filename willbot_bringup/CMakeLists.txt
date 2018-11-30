cmake_minimum_required(VERSION 2.8.3)
project(willbot_bringup)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  robotiq_s_model_control
  ur_modern_driver
  willbot_description
)

catkin_package(
)
