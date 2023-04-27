cmake_minimum_required(VERSION 3.4)
project(za_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  za_hw
  za_msgs
  pluginlib
  realtime_tools
  roscpp
  sensor_msgs
  tf2_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES za_state_controller za_hw
  CATKIN_DEPENDS
    controller_interface
    za_hw
    za_msgs
    pluginlib
    realtime_tools
    roscpp
    sensor_msgs
    tf2_msgs
    tf
)

add_library(za_state_controller src/za_state_controller.cpp)
add_dependencies(za_state_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(za_state_controller ${catkin_LIBRARIES})
target_include_directories(za_state_controller SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(za_state_controller PUBLIC include)

## Installation
install(TARGETS za_state_controller
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
install(FILES za_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)