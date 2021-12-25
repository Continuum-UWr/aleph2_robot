cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring virtual driver")

set(LIB_NAME can_virtual)

file(GLOB LIB_SRC
	"*.c"
)

add_library(${LIB_NAME} OBJECT ${LIB_SRC})
set_property(TARGET ${LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE 1)
