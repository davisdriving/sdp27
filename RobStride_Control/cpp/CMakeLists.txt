cmake_minimum_required(VERSION 3.12)
project(tianrkingControl VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -DDEBUG")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pthread")

# Include directories
include_directories(include)

# Add executable
add_executable(robstride-mit-position src/position_control.cpp)

# Link libraries
target_link_libraries(robstride-mit-position pthread)

# Installation
install(TARGETS robstride-mit-position
    RUNTIME DESTINATION bin
)

# Testing
enable_testing()
add_test(NAME motor_connection_test
         COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/test_motor.sh
         WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

# Custom targets
add_custom_target(setup-can
    COMMAND sudo ip link set can0 type can bitrate 1000000
    COMMAND sudo ip link set up can0
    COMMENT "Setting up CAN interface"
)

add_custom_target(install-deps
    COMMAND sudo apt-get update
    COMMAND sudo apt-get install -y build-essential can-utils cmake
    COMMENT "Installing dependencies"
)

# Package configuration
set(CPACK_PACKAGE_NAME "robstride-control")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "tianrking Motor Control Library")
set(CPACK_PACKAGE_VENDOR "tianrking")
set(CPACK_GENERATOR "DEB;RPM")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.17), libgcc1 (>= 1:4.2), libstdc++6 (>= 5.2)")

include(CPack)