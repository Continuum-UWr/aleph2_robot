cmake_minimum_required(VERSION 3.8)
project(aleph2_bringup)

find_package(ament_cmake_ros REQUIRED)
find_package(lely_core_libraries REQUIRED)

generate_dcf(nanotec_bus)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
