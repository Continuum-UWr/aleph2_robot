cmake_minimum_required(VERSION 3.8)
project(aleph2_bringup)

find_package(ament_cmake_ros REQUIRED)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
