cmake_minimum_required(VERSION 3.5)
project(za_hw)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} src/trigger_rate.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
ament_target_dependencies(
    ${PROJECT_NAME}
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()