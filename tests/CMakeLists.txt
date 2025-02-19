cmake_minimum_required(VERSION 3.22)

project(Tests C CXX ASM)

# Set include directories


# Include FetchContent and get GTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
)
FetchContent_MakeAvailable(googletest)

# Create test executable
add_executable(converter_tests
    ${CMAKE_CURRENT_SOURCE_DIR}/test_Converter.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/../Converter.cxx
)
target_include_directories(converter_tests PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(converter_tests PRIVATE
    gtest
    gtest_main
)

# Enable testing
enable_testing()
# add_test(NAME converter_tests COMMAND converter_tests)
include(GoogleTest)
gtest_discover_tests(converter_tests)