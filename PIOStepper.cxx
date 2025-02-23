#include "PIOStepper.hxx"
#include "PIOStepperSpeedController.pio.h"
#include "Stepper.hxx"
#include <cassert>

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
  pio_sm_set_consecutive_pindirs(myPio, mySm, stepPin, 1, true);

  // SM configuration
  pio_sm_config c = StepperSpeedController_program_get_default_config(myOffset);
  sm_config_set_sideset_pins(&c, stepPin);
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

bool PIOStepper::PutStep(float aFrequency) {

  // Calculate the period in system clocks
  uint32_t period = myConverter.ToPeriod(aFrequency);
  pio_sm_put_blocking(myPio, mySm, period);
  return true;
}

} // namespace PIOStepperSpeedController