cmake_minimum_required(VERSION 2.8.3)
project(aleph2_hardware_interface)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS
    hardware_interface
    joint_limits_interface
    controller_manager
    roscpp
    rubi_server
)

catkin_package(
    INCLUDE_DIRS
        include/
    LIBRARIES
        aleph2_hardware_interface
    CATKIN_DEPENDS
        hardware_interface
        joint_limits_interface
        controller_manager
        roscpp
        rubi_server
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

  
# Hardware interface
add_library( aleph2_hardware_interface 
    src/aleph2_hardware_interface.cpp )

target_link_libraries( aleph2_hardware_interface
    ${catkin_LIBRARIES} )


# Controller Manager
add_executable( controller_manager
    nodes/controller_manager.cpp )

add_dependencies( controller_manager 
    aleph2_hardware_interface )

target_link_libraries( controller_manager
    aleph2_hardware_interface
    ${catkin_LIBRARIES} )

    
install( DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE )
