cmake_minimum_required(VERSION 3.22)

project(Tests C CXX ASM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

# Include FetchContent and get GTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
)
FetchContent_MakeAvailable(googletest)

# Create unified test executable
add_executable(stepper_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_Stepper.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/test_Converter.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../Stepper.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../Converter.cxx
)
target_include_directories(stepper_tests PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(stepper_tests PRIVATE
    gtest
    gtest_main
    gmock
    gmock_main
)

# Enable testing
enable_testing()
include(GoogleTest)
gtest_discover_tests(stepper_tests)