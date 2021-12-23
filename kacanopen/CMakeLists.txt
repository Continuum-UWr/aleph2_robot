cmake_minimum_required(VERSION 3.8)
project(kacanopen)

add_compile_options(-Wall -Wextra -Wpedantic)

### settings ###

include(CMakeSettings.txt)

## Driver begin ##

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

find_package(ament_cmake REQUIRED)
find_package(Threads)
find_package(Boost 1.46.1 REQUIRED COMPONENTS 
    system
    filesystem
) 
find_package(ament_index_cpp REQUIRED)

# Boost required for property_tree / ini_parser
if(NOT(Boost_FOUND))
    message(FATAL_ERROR "Boost > 1.46.1 not found. Boost is required for ini_parser class. This is a header-only library. You can simply download the latest boost package from http://www.boost.org/, extract it into any directory and include it using the following catkin/CMake argument: -DBOOST_ROOT=\"/path/to/boost/\"")
endif()

message(STATUS "Found Boost headers in ${Boost_INCLUDE_DIRS}")
message(STATUS "Found Boost system library: ${Boost_SYSTEM_LIBRARY}")
message(STATUS "Found Boost filesystem library: ${Boost_FILESYSTEM_LIBRARY}")
message(STATUS "Found Threads library: ${CMAKE_THREAD_LIBS_INIT}")

### includes ###

include_directories(include)

### Drivers

add_subdirectory(drivers)
add_subdirectory(drivers_lgpl)

### Core

file(GLOB CORE_SRC "src/core/*.cpp")

add_library(kacanopen_core ${CORE_SRC})
target_link_libraries(kacanopen_core 
    ${DRIVER_LINK_NAME} ${CMAKE_DL_LIBS}
)

### Master

file(GLOB MASTER_SRC "src/master/*.cpp")

add_library(kacanopen_master ${MASTER_SRC})
ament_target_dependencies(kacanopen_master
    Boost
    ament_index_cpp
)
target_link_libraries(kacanopen_master 
    kacanopen_core
    ${CMAKE_THREAD_LIBS_INIT}
)

### Examples

file(GLOB EXAMPLES_SRC
    "src/examples/*.cpp"
    "src/examples/test/*.cpp"
)

foreach(TARGET_SRC ${EXAMPLES_SRC})

    get_filename_component(TARGET_NAME ${TARGET_SRC} NAME_WE)
    # set(TARGET_NAME "kacanopen_example_${TARGET_NAME}")
    add_executable(${TARGET_NAME} ${TARGET_SRC})
    target_link_libraries(${TARGET_NAME} kacanopen_core kacanopen_master)
    
    list(APPEND KACANOPEN_EXAMPLES_TARGET_NAMES ${TARGET_NAME})

endforeach(TARGET_SRC ${EXAMPLES_SRC})

### Install targets

# Libraries
install(TARGETS
    kacanopen_core kacanopen_master
    DESTINATION lib
)

# Library headers
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include
)

# Examples
install(TARGETS ${KACANOPEN_EXAMPLES_TARGET_NAMES}
    DESTINATION lib/${PROJECT_NAME}
)

# EDS files
install(DIRECTORY eds_library
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
