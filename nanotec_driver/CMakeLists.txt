cmake_minimum_required(VERSION 3.8)
project(nanotec_driver)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(kacanopen REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  program_options
)

add_library(${PROJECT_NAME} SHARED
  src/nanotec_driver.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} 
  PUBLIC
    kacanopen::kacanopen_master
)

add_executable(nanotec_autocalib
  src/examples/nanotec_autocalib.cpp
)
target_include_directories(nanotec_autocalib
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(nanotec_autocalib
  ${PROJECT_NAME}
  Boost::program_options
)

### Install targets

# Libraries
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
	LIBRARY DESTINATION lib
)

# Library headers
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Examples
install(TARGETS nanotec_autocalib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGETS)
ament_export_dependencies(kacanopen)
ament_package()
