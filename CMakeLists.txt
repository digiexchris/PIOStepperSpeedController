if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH must be defined before including this library")
endif()



# Create library target
add_library(PIOStepperSpeedController
${CMAKE_CURRENT_SOURCE_DIR}/PIOStepperSpeedController.cxx
${CMAKE_CURRENT_SOURCE_DIR}/PIOStepperSpeedController.hxx
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
