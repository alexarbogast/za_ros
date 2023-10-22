cmake_minimum_required(VERSION 3.4)
project(za_gazebo)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    roscpp
    gazebo_ros_control
    kdl_parser
    boost_sml
    control_toolbox
    hardware_interface
    joint_limits_interface
    eigen_conversions
    urdf
    za_hw
    za_control
    za_controllers
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES franka_hw_sim
    CATKIN_DEPENDS
        roscpp
        gazebo_ros_control
        kdl_parser
        boost_sml
        control_toolbox
        hardware_interface
        joint_limits_interface
        eigen_conversions
        urdf
        za_hw
        za_control
        za_controllers
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(za_hw_sim 
  src/za_hw_sim.cpp
  src/joint.cpp
  src/model_kdl.cpp
  src/controller_verifier.cpp  
)
target_link_libraries(za_hw_sim ${catkin_LIBRARIES})
target_include_directories(za_hw_sim SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
add_dependencies(za_hw_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES za_hw_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS za_hw_sim 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)