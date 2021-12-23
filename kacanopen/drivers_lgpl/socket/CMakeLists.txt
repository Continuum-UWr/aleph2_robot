cmake_minimum_required(VERSION 3.8)

message(STATUS "Configuring socket driver")

set(LIB_NAME can_socket)

file(GLOB TARGET_SRC
	"*.c"
)

add_library(${LIB_NAME} SHARED ${TARGET_SRC})

install(TARGETS ${LIB_NAME}
	DESTINATION lib
)