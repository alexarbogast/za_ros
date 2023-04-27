cmake_minimum_required(VERSION 3.4)
project(za_hw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    hardware_interface
    roscpp
    pluginlib
    urdf
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES za_hw
    CATKIN_DEPENDS
        controller_interface
        hardware_interface
        roscpp
        pluginlib
        urdf
)

add_library(za_hw src/trigger_rate.cpp)
add_dependencies(za_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(za_hw ${catkin_LIBRARIES})
target_include_directories(za_hw SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(za_hw PUBLIC include)

## Installation
install(TARGETS za_hw
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)