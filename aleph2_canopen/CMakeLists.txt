cmake_minimum_required(VERSION 3.8)
project(aleph2_canopen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lely_core_libraries REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(canopen_402_driver REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_dcf(nanotec)

add_library(aleph2_canopen SHARED
  src/node_interfaces/node_canopen_nanotec_driver.cpp
  src/nanotec_driver.cpp
  src/motor.cpp
  src/auto_setup.cpp
)
target_include_directories(aleph2_canopen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(aleph2_canopen
  canopen_proxy_driver::node_canopen_proxy_driver
  canopen_402_driver::lely_motion_controller_bridge
)
rclcpp_components_register_nodes(aleph2_canopen "aleph2_canopen::NanotecDriver")

install(
  TARGETS aleph2_canopen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
