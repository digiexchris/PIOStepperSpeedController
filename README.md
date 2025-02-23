# [PIOStepperSpeedController](https://github.com/digiexchris/PIOStepperSpeedController.git)

A PIO-based stepper motor speed controller for the RP2040 microcontroller that provides smooth acceleration and deceleration profiles. It was originally intended to be used for uses where a stepper is used in speed mode, rather than for tracking a position.

## Features
- Uses the RP2040's PIO state machine for precise pulse timing
- Separately configurable acceleration and deceleration
- Adjustable minimum and maximum speeds
- Prescaler support for higher speed ranges
- Optional state change callbacks

## Requirements
- C++20 capable compiler
- Raspberry Pi Pico SDK
- CMake 3.13 or newer
- Visual Studio Code (recommended)

## Installation
1. Clone this repository
2. Include in your CMake project using FetchContent:
```cmake
include(FetchContent)
FetchContent_Declare(
    PIOStepperSpeedController
    GIT_REPOSITORY https://github.com/digiexchris/PIOStepperSpeedController.git
    GIT_TAG main
)
FetchContent_MakeAvailable(PIOStepperSpeedController)

target_link_libraries(your_target PRIVATE PIOStepperSpeedController)
```

## Example
example/ contains an advanced sample including details about configuring the sysclk and prescaler parameters.

## Usage
```cpp
// Initialize with pins, speed settings and callbacks
// See PIOStepper.hxx and Stepper.hxx for more details
// See the example for information on using the callbacks
PIOStepper stepper(
    stepPin,         // GPIO pin number
    minSpeed,        // Minimum speed in Hz (must be > 0)
    maxSpeed,        // Maximum speed in Hz 
    acceleration,    // Steps/second²
    deceleration,   // Steps/second²
    sysclk,         // System clock (default 125MHz)
    prescaler       // Clock divider (default 1)
);

// Start the motor at the minimum configured speed
stepper.Start();

// Set target speed to 1000 hz (1000 steps per second)
stepper.SetTargetHz(1000);

// Must call Update() frequently to maintain motion profile
while(true) {
    stepper.Update();
}

// Stop with deceleration
stepper.Stop();
```

## Important Notes
- Minimum speed must be greater than 0 Hz
- Lower minimum speeds result in longer initial step times. For example, a minimum of 0.25 hz would take 4 seconds to complete the first step.
- Maximum speed is limited by system clock and prescaler, see Converter.hxx and Stepper.hxx for more information.
- The Update() function should be called as frequently as possible, and will block until the step has been sent to the PIO fifo. Given that the fifo can contain up to 4 steps in it's queue, it's ideal if you call this in a way that lets it run as fast as possible and queue up all steps, and then wait. I typically use a freertos task or similar.

## Development
Development container configuration is included for VS Code. Required extensions will be suggested when opening the project. Open the pico-project.code-workspace in the example folder.

## Testing
Tests use Google Test framework and can be built with:
```bash
cd tests
mkdir build && cd build
cmake ..
make
./stepper_tests
```

## Uses in the wild
[PicoMillPowerFeed](https://github.com/digiexchris/PicoMillPowerFeed) - A milling machine power feed replacement