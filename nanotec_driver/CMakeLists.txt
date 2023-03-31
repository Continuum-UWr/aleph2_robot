cmake_minimum_required(VERSION 3.8)
project(nanotec_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_dcf(nanotec_c5)
generate_dcf(nanotec_cl3)

add_library(${PROJECT_NAME} SHARED
  src/node_interfaces/node_canopen_nanotec_driver.cpp
  src/auto_setup.cpp
  src/command.cpp
  src/lely_nanotec_bridge.cpp
  src/motor.cpp
  src/nanotec_driver.cpp
  src/nanotec_system.cpp
  src/state.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  canopen_proxy_driver::node_canopen_proxy_driver
  rclcpp_components::component
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(${PROJECT_NAME}
  canopen_interfaces
  hardware_interface
  sensor_msgs
)

rclcpp_components_register_nodes(${PROJECT_NAME} "nanotec_driver::NanotecDriver")

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
