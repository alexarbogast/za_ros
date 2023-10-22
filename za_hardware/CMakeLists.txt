cmake_minimum_required(VERSION 3.0.2)
project(za_hardware)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    rospy
)

install(DIRECTORY halfiles config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)