cmake_minimum_required(VERSION 3.8)
project(aleph2_hardware)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nanotec_driver REQUIRED)

add_library(${PROJECT_NAME}
  src/nanotec_system.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}
  pluginlib::pluginlib
  rclcpp::rclcpp
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
)

pluginlib_export_plugin_description_file(
  hardware_interface aleph2_hardware.xml
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(${PROJECT_NAME}Targets)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  nanotec_driver
)
ament_package()