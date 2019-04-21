cmake_minimum_required(VERSION 2.8.3)
project(aleph2_hardware_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    hardware_interface
    joint_limits_interface
    controller_manager
    roscpp
)

find_package(Boost )

catkin_package(
    INCLUDE_DIRS
        include/
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        hardware_interface
        joint_limits_interface
        controller_manager
        roscpp
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
#add_dependencies(aleph2_hardware_interface aleph2cpp)
target_link_libraries(${PROJECT_NAME}
#	${aleph2cpp_LIBRARIES}
	${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node 
    src/${PROJECT_NAME}_node.cpp)

add_dependencies(${PROJECT_NAME}_node 
    ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES})

    
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)
