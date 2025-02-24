if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH must be defined before including this library")
endif()

# Option for building tests
option(BUILD_TESTS "Build tests" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create library target
add_library(PIOStepperSpeedController
    ${CMAKE_CURRENT_SOURCE_DIR}/PIOStepper.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/PIOStepper.hxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Converter.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Converter.hxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Stepper.hxx
)

# Generate PIO header
pico_generate_pio_header(PIOStepperSpeedController
    ${CMAKE_CURRENT_SOURCE_DIR}/PIOStepperSpeedController.pio
)

# Set include directories
target_include_directories(PIOStepperSpeedController PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against pico dependencies
target_link_libraries(PIOStepperSpeedController PUBLIC
    hardware_pio
)

if(BUILD_TESTS)
    # Include FetchContent and get GTest
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
    )
    FetchContent_MakeAvailable(googletest)

    # Create test executable
    add_executable(converter_tests
        ${CMAKE_CURRENT_SOURCE_DIR}/ConverterTest.cxx
        ${CMAKE_CURRENT_SOURCE_DIR}/Converter.cxx
    )

    target_link_libraries(converter_tests PRIVATE
        gtest
        gtest_main
    )

    # Enable testing
    enable_testing()
    add_test(NAME converter_tests COMMAND converter_tests)
endif()
