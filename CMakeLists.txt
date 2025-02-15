if(NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH must be defined before including this library")
endif()

# Verify pico_sdk_init() has been called
if(NOT TARGET pico_sdk)
    message(FATAL_ERROR "pico_sdk_init() must be called before including this library") 
endif()

# Generate PIO header
pico_generate_pio_header(PIOStepper
    ${CMAKE_CURRENT_SOURCE_DIR}/PIOStepper.pio
)

# Create library target
add_library(PIOStepper
    PIOStepper.cxx
    PIOStepper.hxx
)

# Set include directories
target_include_directories(PIOStepper PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link against pico dependencies
target_link_libraries(PIOStepper PUBLIC
    hardware_pio
)
