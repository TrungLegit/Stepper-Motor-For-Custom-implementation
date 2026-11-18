cmake_minimum_required(VERSION 3.16)
project(high_detail_3d_scanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SCANNER_APP_SOURCES
    src/main.cpp
    src/ScanPathGenerator.cpp
    src/CaptureEngine.cpp
    src/PoseExporter.cpp
)

if(TARGET motion_core)
    add_executable(high_detail_3d_scanner ${SCANNER_APP_SOURCES})
    target_include_directories(high_detail_3d_scanner PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock
    )
    target_link_libraries(high_detail_3d_scanner PRIVATE motion_core)
else()
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl
        ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock
    )

    file(GLOB_RECURSE PROTOCOL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/src/protocol/*.cpp")
    file(GLOB_RECURSE PLANNER_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/src/planner/*.cpp")
    file(GLOB_RECURSE MOCK_HAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock/*.cpp")
    file(GLOB_RECURSE HAL_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/src/hal/*.cpp")

    add_executable(high_detail_3d_scanner
        ${SCANNER_APP_SOURCES}
        ${PROTOCOL_SRCS}
        ${PLANNER_SRCS}
        ${MOCK_HAL_SRCS}
        ${HAL_SRCS}
    )
endif()

if(BUILD_TESTS)
    set(SCANNER_TEST_SOURCES
        tests/test_scanner.cpp
        src/ScanPathGenerator.cpp
        src/CaptureEngine.cpp
        src/PoseExporter.cpp
    )

    if(TARGET motion_core)
        add_executable(test_scanner ${SCANNER_TEST_SOURCES})
        target_include_directories(test_scanner PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock
        )
        target_link_libraries(test_scanner PRIVATE motion_core)
    else()
        add_executable(test_scanner
            ${SCANNER_TEST_SOURCES}
            ${PROTOCOL_SRCS}
            ${PLANNER_SRCS}
            ${MOCK_HAL_SRCS}
            ${HAL_SRCS}
        )
        target_include_directories(test_scanner PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl
            ${CMAKE_CURRENT_SOURCE_DIR}/../../packages/motion-core/hal_impl/mock
        )
    endif()

    add_test(NAME test_scanner COMMAND test_scanner)
endif()
