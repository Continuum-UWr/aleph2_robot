cmake_minimum_required(VERSION 3.0.2)
project(nanotec_driver)

add_compile_options(-std=c++11)

find_package( catkin REQUIRED COMPONENTS
    kacanopen)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS kacanopen)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/nanotec.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(nanotec_autocalib examples/nanotec_autocalib.cpp)
target_link_libraries(nanotec_autocalib ${PROJECT_NAME})


### Install targets

# Libraries
install(TARGETS nanotec_driver
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Library headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Examples
install(TARGETS nanotec_autocalib
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)