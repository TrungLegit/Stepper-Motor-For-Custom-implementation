cmake_minimum_required(VERSION 3.16)
project(minimal_template_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl
)

file(GLOB_RECURSE PROTOCOL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/src/protocol/*.cpp")
file(GLOB_RECURSE PLANNER_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/src/planner/*.cpp")
file(GLOB_RECURSE MOCK_HAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock/*.cpp")

add_executable(minimal_template
    src/main.cpp
    ${PROTOCOL_SRCS}
    ${PLANNER_SRCS}
    ${MOCK_HAL_SRCS}
)
