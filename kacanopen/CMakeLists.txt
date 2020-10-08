cmake_minimum_required(VERSION 2.8)

project(kacanopen)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra") # -Werror
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra") # -Werror

### settings ###

include(CMakeSettings.txt)

## Driver begin ##

if (DEFINED CAN_DRIVER_NAME)
	message(WARNING "CAN_DRIVER_NAME is deprecated. Use DRIVER instead. Overwriting DRIVER with CAN_DRIVER_NAME (${CAN_DRIVER_NAME}) now...")
	set(DRIVER "${CAN_DRIVER_NAME}")
endif()

foreach(BUILD_DRIVER ${BUILD_DRIVERS})
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND SUPPORTED_DRIVERS "${BUILD_DRIVER}" supported)
	if (${supported} EQUAL -1)
		message(FATAL_ERROR "Driver ${BUILD_DRIVER} does not exist, but you've specified it in BUILD_DRIVERS.")
	endif()
endforeach()

list(FIND SUPPORTED_DRIVERS "${DRIVER}" driver_supported)
if (${driver_supported} EQUAL -1)
	message(FATAL_ERROR "Driver ${DRIVER} does not exist, but you've set it as DRIVER.")
endif()

set(DRIVER_LINK_NAME can_${DRIVER})
message(STATUS "Using CAN driver ${DRIVER_LINK_NAME}")

if(${BUILD_ALL_DRIVERS})
	message(STATUS "Building all available drivers.")
endif()

## Driver end ##

if(${CMAKE_BUILD_TYPE} MATCHES "Release")
	message(STATUS "You are in release mode.")
else()
	if(${EXHAUSTIVE_DEBUGGING})
		message(STATUS "Enabled exhaustive debugging.")
		add_definitions("-DEXHAUSTIVE_DEBUGGING")
	else()
		message(STATUS "Enabled debugging.")
	endif()
endif()



### constants ###

add_definitions("-DSDO_RESPONSE_TIMEOUT_MS=${SDO_RESPONSE_TIMEOUT_MS}")
message(STATUS "SDO response timeout is set to ${SDO_RESPONSE_TIMEOUT_MS}ms")

add_definitions("-DBUSNAME=\"${BUSNAME}\"")
message(STATUS "Busname is set to ${BUSNAME} (only used by examples)")

add_definitions("-DBAUDRATE=${BAUDRATE}")
message(STATUS "Baudrate is set to ${BAUDRATE} (only used by examples)")

add_definitions("-DCONSECUTIVE_SEND_PAUSE_MS=${CONSECUTIVE_SEND_PAUSE_MS}")
message(STATUS "Pause between two consecutively sent CAN frames is set to ${CONSECUTIVE_SEND_PAUSE_MS}ms")


### dependencies ###

find_package(Threads)
find_package(Boost 1.46.1 COMPONENTS system filesystem REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslib
	std_msgs
	sensor_msgs
	# message_generation
)

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
	message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
message(STATUS "Found Boost system library: ${Boost_SYSTEM_LIBRARY}")
message(STATUS "Found Boost filesystem library: ${Boost_FILESYSTEM_LIBRARY}")
message(STATUS "Found Threads library: ${CMAKE_THREAD_LIBS_INIT}")


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${CAN_DRIVER} kacanopen_core kacanopen_master kacanopen_ros_bridge
	CATKIN_DEPENDS roscpp roslib std_msgs sensor_msgs
)


### includes ###

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

### Drivers

add_subdirectory(drivers)
add_subdirectory(drivers_lgpl)

### Core

file(GLOB CORE_SRC "src/core/*.cpp")

add_library(kacanopen_core ${CORE_SRC})
target_link_libraries(kacanopen_core 
	${DRIVER_LINK_NAME} ${CMAKE_DL_LIBS} ${CMAKE_THREAD_LIBS_INIT})

### Master

file(GLOB MASTER_SRC "src/master/*.cpp")

add_library(kacanopen_master ${MASTER_SRC})
target_link_libraries(kacanopen_master 
	${Boost_LIBRARIES} ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} kacanopen_core)

### ROS Bridge

file(GLOB ROS_BRIDGE_SRC "src/ros_bridge/*.cpp")

add_library(kacanopen_ros_bridge ${ROS_BRIDGE_SRC})
target_link_libraries(kacanopen_ros_bridge ${catkin_LIBRARIES} kacanopen_master)

### Examples

file(GLOB EXAMPLES_SRC
	"src/examples/*.cpp"
	"src/examples/test/*.cpp"
	"src/examples/ros/*.cpp"
)

foreach(TARGET_SRC ${EXAMPLES_SRC})

	get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
	# set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
	add_executable(${TARGET_NAME} ${TARGET_SRC})
	target_link_libraries(${TARGET_NAME} kacanopen_core kacanopen_master kacanopen_ros_bridge)
	target_link_libraries(${TARGET_NAME} ${catkin_LIBRARIES})
	
	list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})

endforeach(TARGET_SRC ${EXAMPLES_SRC})


### Install targets

# Libraries
install(TARGETS kacanopen_core kacanopen_master kacanopen_ros_bridge
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Library headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Examples
install(TARGETS ${KACANOPEN_EXAMPLES_TARGET_NAMES}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# EDS files
install(DIRECTORY eds_library/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/eds_library
)
