cmake_minimum_required(VERSION 3.16)
project(stepper_motion_core VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags for clean code and warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

option(BUILD_TESTS "Build unit tests on host PC" ON)
option(BUILD_APPS "Build applications (scanner and minimal-template)" ON)
set(MOTION_CORE_HAL "mock" CACHE STRING "HAL implementation: mock, esp32s3, rp2040")

message(STATUS "Configuring stepper-motion-core...")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  HAL implementation: ${MOTION_CORE_HAL}")
message(STATUS "  Build unit tests: ${BUILD_TESTS}")
message(STATUS "  Build applications: ${BUILD_APPS}")

if(BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(packages/motion-core)

if(BUILD_APPS)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/minimal-template/CMakeLists.txt")
        add_subdirectory(apps/minimal-template)
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/apps/high-detail-3d-scanner/CMakeLists.txt")
        add_subdirectory(apps/high-detail-3d-scanner)
    endif()
endif()
