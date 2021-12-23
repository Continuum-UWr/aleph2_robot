cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring lincan driver")

set(TARGET_NAME can_lincan)

file(GLOB TARGET_SRC
	"*.c"
)

add_library(${TARGET_NAME} ${TARGET_SRC})
