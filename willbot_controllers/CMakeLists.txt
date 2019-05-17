cmake_minimum_required(VERSION 2.8.3)
project(willbot_controllers)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  control_toolbox
  controller_interface
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  kdl_conversions
  kdl_parser  
  pluginlib
  realtime_tools
  roscpp
  urdf 
)

generate_dynamic_reconfigure_options(
  cfg/compliance_param.cfg
)

catkin_package(
  LIBRARIES
    willbot_controllers
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
)

add_library(willbot_controllers
  src/cartesian_admittance_controller.cpp
  src/cartesian_velocity_controller.cpp
  src/joint_velocity_controller.cpp
  src/performance_controller.cpp
)

add_dependencies(willbot_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  # ${PROJECT_NAME}_generate_messages_cpp
  # ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(willbot_controllers PUBLIC
  ${catkin_LIBRARIES}
)

target_include_directories(willbot_controllers SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(willbot_controllers PUBLIC
  include
)

# Installation
install(TARGETS willbot_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES willbot_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})