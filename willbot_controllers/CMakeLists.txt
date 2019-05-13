cmake_minimum_required(VERSION 2.8.3)
project(willbot_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  kdl_conversions
  kdl_parser  
  pluginlib
  realtime_tools
  roscpp
  urdf 
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    geometry_msgs
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(admittance_controller
  src/admittance_controller.cpp 
  include/willbot_controllers/admittance_controller.h
)
target_link_libraries(admittance_controller ${catkin_LIBRARIES})

add_library(performance_controller
  src/performance_controller.cpp 
  include/willbot_controllers/performance_controller.h
)
target_link_libraries(performance_controller ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install libraries
install(TARGETS 
    admittance_controller
    performance_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install export descriptions
install(FILES 
    admittance_controller_plugin.xml
    performance_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})