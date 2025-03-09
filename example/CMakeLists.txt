# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
# CXX 20 is required due to the use of concepts in the library
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hello_PIOStepperSpeedController C CXX ASM)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-O0)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(hello_PIOStepperSpeedController)

target_sources(hello_PIOStepperSpeedController PRIVATE main.cxx)


# Example of using FetchContent to install
# include(FetchContent)
# FetchContent_Declare(
#     PIOStepperSpeedController
#     GIT_REPOSITORY https://github.com/digiexchris/PIOStepperSpeedController.git
#     GIT_TAG main
# )
# FetchContent_MakeAvailable(PIOStepperSpeedController)

add_subdirectory(lib/PIOStepper PIOStepperSpeedController)

target_link_libraries(hello_PIOStepperSpeedController PRIVATE
        pico_stdlib
        pico_multicore
        PIOStepperSpeedController
        )

pico_add_extra_outputs(hello_PIOStepperSpeedController)

pico_enable_stdio_usb(hello_PIOStepperSpeedController 1)
pico_enable_stdio_uart(hello_PIOStepperSpeedController 0)
