cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring lincan driver")

set(TARGET_NAME can_lincan)

file(GLOB TARGET_SRC
	"*.c"
)

add_library(${TARGET_NAME} OBJECT ${TARGET_SRC})
set_property(TARGET ${TARGET_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
