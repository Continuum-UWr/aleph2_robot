cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring drivers")

### Building macro ###
macro(driver_switch)
	# TODO: Replace with CMake 3.3 command IN_LIST
	list(FIND BUILD_DRIVERS "${ARGV0}" build_this)
	if (("${DRIVER}" MATCHES "${ARGV0}") OR (${build_this} GREATER -1) OR (${BUILD_ALL_DRIVERS}))

		set(TARGET_NAME "can_${ARGV0}")
		add_library(${TARGET_NAME} SHARED "${ARGV0}.cpp")
		target_link_libraries(${TARGET_NAME} ${CMAKE_DL_LIBS})

	endif()
endmacro()

### List of drivers ###
driver_switch("dummy")
