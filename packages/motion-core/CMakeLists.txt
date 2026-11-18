cmake_minimum_required(VERSION 3.16)

# motion-core package definition
project(motion_core LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Base include directories
set(MOTION_CORE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(MOTION_CORE_MOCK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/mock)
set(MOTION_CORE_ESP32S3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/esp32s3)

# Collect Protocol sources
file(GLOB_RECURSE PROTOCOL_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/protocol/*.cpp
)

# Collect Planner sources (if exist)
file(GLOB_RECURSE PLANNER_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner/*.cpp
)

# Collect HAL sources depending on target
set(HAL_SRCS "")
if(MOTION_CORE_HAL STREQUAL "mock")
    file(GLOB_RECURSE HAL_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/mock/*.cpp
    )
elseif(MOTION_CORE_HAL STREQUAL "esp32s3")
    file(GLOB_RECURSE HAL_SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hal/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/esp32s3/*.cpp
    )
endif()

# Core static library
add_library(motion_core STATIC
    ${PROTOCOL_SRCS}
    ${PLANNER_SRCS}
    ${HAL_SRCS}
)

target_include_directories(motion_core PUBLIC
    $<BUILD_INTERFACE:${MOTION_CORE_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${MOTION_CORE_MOCK_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

if(MOTION_CORE_HAL STREQUAL "esp32s3")
    target_include_directories(motion_core PUBLIC
        $<BUILD_INTERFACE:${MOTION_CORE_ESP32S3_INCLUDE_DIR}>
    )
endif()

target_compile_features(motion_core PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(motion_core PRIVATE /W4)
else()
    target_compile_options(motion_core PRIVATE -Wall -Wextra -pedantic)
endif()

# Unit tests build
if(BUILD_TESTS)
    enable_testing()

    macro(add_motion_test TEST_NAME TEST_SOURCE)
        if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/${TEST_SOURCE}")
            add_executable(${TEST_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/tests/${TEST_SOURCE}")
            target_link_libraries(${TEST_NAME} PRIVATE motion_core)
            target_include_directories(${TEST_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/include
                ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/mock
                ${CMAKE_CURRENT_SOURCE_DIR}
                ${CMAKE_CURRENT_SOURCE_DIR}/tests
            )
            target_compile_features(${TEST_NAME} PRIVATE cxx_std_17)
            if(MSVC)
                target_compile_options(${TEST_NAME} PRIVATE /W4)
            else()
                target_compile_options(${TEST_NAME} PRIVATE -Wall -Wextra)
            endif()
            add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
        endif()
    endmacro()

    add_motion_test(test_protocol test_protocol.cpp)
    add_motion_test(test_scurve test_scurve.cpp)
    add_motion_test(test_planner test_planner.cpp)
    add_motion_test(test_backlash test_backlash.cpp)
    add_motion_test(test_homing test_homing.cpp)

    # Consolidated master test runner
    add_executable(test_main
        tests/test_main.cpp
        tests/test_protocol.cpp
        tests/test_scurve.cpp
        tests/test_planner.cpp
        tests/test_backlash.cpp
        tests/test_homing.cpp
    )
    target_link_libraries(test_main PRIVATE motion_core)
    target_include_directories(test_main PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/hal_impl/mock
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    target_compile_definitions(test_main PRIVATE TEST_COMBINED_RUNNER)
    target_compile_features(test_main PRIVATE cxx_std_17)
    if(MSVC)
        target_compile_options(test_main PRIVATE /W4)
    else()
        target_compile_options(test_main PRIVATE -Wall -Wextra)
    endif()
    add_test(NAME test_main COMMAND test_main)
endif()
