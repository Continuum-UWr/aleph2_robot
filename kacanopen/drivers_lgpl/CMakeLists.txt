cmake_minimum_required(VERSION 2.8)
project(KaCanOpen_drivers C)

message(STATUS "Configuring LGPL drivers")

### Includes ###
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

if(WIN32)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/include/win32)
else()
	include_directories(${CMAKE_CURRENT_LIST_DIR}/include/unix)
	include_directories(${CMAKE_CURRENT_LIST_DIR}/include/timers_unix)
endif()

### Building macro ###
macro(lgpl_driver_switch)
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND BUILD_DRIVERS "${ARGV0}" build_this)
	if (("${DRIVER}" MATCHES "${ARGV0}") OR (${build_this} GREATER -1) OR (${BUILD_ALL_DRIVERS}))
		add_subdirectory("${ARGV0}")
	endif()
endmacro()

### List of drivers ###
lgpl_driver_switch("lincan")
lgpl_driver_switch("peak_linux")
lgpl_driver_switch("serial")
lgpl_driver_switch("socket")
lgpl_driver_switch("virtual")