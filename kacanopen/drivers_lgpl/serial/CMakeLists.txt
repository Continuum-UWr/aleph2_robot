cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring serial driver")

set(LIB_NAME can_serial)
set(HUB_NAME can_serial_hub)

add_library(${LIB_NAME} OBJECT can_serial.c)
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)

add_definitions("-fno-strict-aliasing")
add_executable(${HUB_NAME}
	can_serial_hub.c
	$<TARGET_OBJECTS:${LIB_NAME}>
)

install(TARGETS ${HUB_NAME}
	DESTINATION lib/${PROJECT_NAME}
)