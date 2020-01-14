cmake_minimum_required(VERSION 2.8.3)
project(aleph2_hardware_interface)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS
    hardware_interface
    joint_limits_interface
    controller_manager
    roscpp
    rubi_server
    nanotec_driver
    dynamic_reconfigure
    std_srvs
)


generate_dynamic_reconfigure_options(
  cfg/Nanotec.cfg )


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
        nanotec_driver
        dynamic_reconfigure
        std_srvs
)

catkin_install_python(PROGRAMS
  nodes/joint_state_republisher
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})


# Joint
add_library( aleph2_joint
    src/aleph2_joint/rubi_joint.cpp
    src/aleph2_joint/nanotec_joint.cpp
    src/aleph2_joint/rubi_encoder_addon.cpp)

target_link_libraries( aleph2_joint
    ${catkin_LIBRARIES} )

add_dependencies( aleph2_joint
    ${PROJECT_NAME}_gencfg )


# Hardware interface
add_library( aleph2_hardware_interface 
    src/aleph2_hardware_interface/aleph2_hardware_interface.cpp )

target_link_libraries( aleph2_hardware_interface
    ${catkin_LIBRARIES} 
    aleph2_joint )


# Controller Manager
add_executable( controller_manager
    nodes/controller_manager.cpp )

add_dependencies( controller_manager 
    aleph2_hardware_interface )

target_link_libraries( controller_manager
    aleph2_hardware_interface
    ${catkin_LIBRARIES} )


### Install targets
    
install( DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(TARGETS controller_manager aleph2_hardware_interface aleph2_joint
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)