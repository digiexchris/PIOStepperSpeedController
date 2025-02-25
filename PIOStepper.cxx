#include <PIOStepperSpeedController/PIOStepper.hxx>
#include <PIOStepperSpeedController/PIOStepperSpeedController.pio.h>
#include <PIOStepperSpeedController/Stepper.hxx>
#include <algorithm>
#include <cassert>
#include <cstdio>
#include <hardware/clocks.h>
#include <pico/time.h>

namespace PIOStepperSpeedController {

PIOStepper::PIOStepper(uint32_t stepPin, float aMinSpeed, float aMaxSpeed,
                       uint32_t aAcceleration, uint32_t aDeceleration,
                       uint32_t aSysClk, uint32_t aPrescaler,
                       Callback aStoppedCallback, Callback aCoastingCallback,
                       Callback aAcceleratingCallback,
                       Callback aDeceleratingCallback)
    : Stepper<PIOStepper>(aMinSpeed, aMaxSpeed, aAcceleration, aDeceleration,
                          aSysClk, aPrescaler, aStoppedCallback,
                          aCoastingCallback, aAcceleratingCallback,
                          aDeceleratingCallback),
      myStepPin(stepPin) {

  assert(aMinSpeed > 0);
  assert(aMaxSpeed > 0);
  assert(aMaxSpeed < aSysClk / aPrescaler);
  assert(aAcceleration > 0);
  assert(aDeceleration > 0);

  bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
      &StepperSpeedController_program, &myPio, &mySm, &myOffset, stepPin, 1,
      true);
  assert(success);

  // GPIO setup
  pio_gpio_init(myPio, stepPin);
  pio_gpio_init(myPio, stepPin + 1);
  pio_sm_set_consecutive_pindirs(myPio, mySm, stepPin, 2, true);

  // SM configuration
  pio_sm_config c = StepperSpeedController_program_get_default_config(myOffset);
  // sm_config_set_sideset_pins(&c, stepPin);
  sm_config_set_set_pins(&c, stepPin, 1);
  sm_config_set_sideset_pins(&c, stepPin + 1);

  sm_config_set_clkdiv(&c, aPrescaler);

  // Initialize and clear
  pio_sm_init(myPio, mySm, myOffset, &c);
  pio_sm_clear_fifos(myPio, mySm);
}

void PIOStepper::EnableImpl() { pio_sm_set_enabled(myPio, mySm, true); }

void PIOStepper::DisableImpl() {
  pio_sm_set_enabled(myPio, mySm, false);
  gpio_put(myStepPin, 0);
}

bool PIOStepper::IsSmEnabled() {
  // Check if state machine is enabled using CTRL register
  return (myPio->ctrl & (1u << (PIO_CTRL_SM_ENABLE_LSB + mySm))) != 0;
}

bool PIOStepper::PutStep(float aFrequency) {

  bool is_enabled = IsSmEnabled();
  assert(is_enabled);

  uint32_t period = myConverter.ToPeriod(aFrequency);

  uint16_t half = std::max<uint16_t>(
      1u,
      period >>
          1); // bitshift by 1 to divide by 2, oohh fancy
              // pants. Also ensure minimum of 1 so the PIO registers
              // doesn't underflow. IMO it's better than adding an
              // additional 2 cycles into the PIO program to check for
              // zero. It would have a small effect at slow speeds, but
              // increasingly large effect as speed increases since as it is, it
              // already takes 6? cycles minimum to execute 1 complete step.
  uint32_t packed = (half << 16) | half;

  pio_sm_put_blocking(myPio, mySm, packed);

  return true;
}

} // namespace PIOStepperSpeedController