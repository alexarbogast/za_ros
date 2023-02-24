cmake_minimum_required(VERSION 3.4)
project(za_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  za_hw
  za_msgs
  geometry_msgs
  hardware_interface
  joint_limits_interface
  pluginlib
  realtime_tools
  roscpp
  urdf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES za_controllers
  CATKIN_DEPENDS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  za_hw
  za_msgs
  geometry_msgs
  hardware_interface
  joint_limits_interface
  message_runtime
  pluginlib
  realtime_tools
  roscpp
  urdf
)

add_library(za_controllers
    src/cartesian_velocity_controller.cpp
    src/cartesian_posvel_controller.cpp
)
add_dependencies(za_controllers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(za_controllers PUBLIC ${catkin_LIBRARIES})
target_include_directories(za_controllers SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_include_directories(za_controllers PUBLIC
  include
)

## Installation
install(TARGETS za_controllers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES za_controllers_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)