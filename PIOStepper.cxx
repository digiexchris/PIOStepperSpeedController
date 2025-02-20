#include "PIOStepper.hxx"

namespace PIOStepperSpeedController {

PIOStepper::PIOStepper(uint32_t stepPin, uint32_t startSpeedHz,
                       uint32_t maxSpeedHz, uint32_t aAcceleration,
                       uint32_t aDeceleration, uint32_t aSysClk,
                       uint32_t aPrescaler, Callback aStoppedCallback,
                       Callback aCoastingCallback,
                       Callback aAcceleratingCallback,
                       Callback aDeceleratingCallback)
    : IStepper(aAcceleration, aDeceleration, aSysClk, aPrescaler,
               aStoppedCallback, aCoastingCallback, aAcceleratingCallback,
               aDeceleratingCallback) {

  assert(startSpeedHz > 0);
  assert(maxSpeedHz > 0);
  assert(stepsPerRotation > 0);
  assert(acceleration > 0);
  assert(deceleration > 0);

  //   mySysClk = aSysClk;
  //   myConfiguredPrescaler = 68.72f;

  // I THINK THIS CALCULATION IS WRONG <
  //     1 / maxfreq = minPeriod,
  //         1 / minfreq = maxPeriod

  // myMaxPeriod = mySysClk * ((1 / minSpeedHz) / myConfiguredPrescaler);
  // myMaxPeriod = std::min(myMaxPeriod, UINT32_MAX);
  myMaxPeriod = UINT32_MAX; //(1hz)

  myMinPeriod = mySysClk * ((1 / maxSpeedHz) / myConfiguredPrescaler);
  myMinPeriod = std::max(myMinPeriod, 1); // 227,389.73hz

  // Convert acceleration from steps/s² to cycles per REAL LIFE integer period
  // change Higher acceleration = fewer cycles between period changes eg: if
  // this value is 10, then the PIO program requires 10 cycles to reduce
  // the real time period the equivalent of by 1/(mySysClk/myConfiguredDiv) I
  // think...
  myAcccelerationCyclesPerPeriodChange =
      static_cast<uint32_t>((mySysClk / myConfiguredPrescaler) / acceleration);

  myDecelerationCyclesPerPeriodChange =
      static_cast<uint32_t>((mySysClk / myConfiguredPrescaler) / deceleration);

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
  sm_config_set_clkdiv(&c, myConfiguredPrescaler);

  // Initialize and clear
  pio_sm_init(myPio, mySm, myOffset, &c);
  pio_sm_clear_fifos(myPio, mySm);

  myState = StepperState::STOPPED;

  // myMinPeriod = 1;
  myStartPeriod = PrivFrequencyToPeriod(startSpeedHz);
  myStepAngleRadians = 2 * PI / myStepsPerRotation;
  myCurrentPeriod = myStartPeriod;
}

void PIOStepper::EnableImpl() { pio_sm_set_enabled(myPio, mySm, true); }

void PIOStepper::DisableImpl() {
  pio_sm_set_enabled(myPio, mySm, false);
  gpio_put(myStepPin, 0);
}

bool PIOStepper::Step(StepperState state) {}

void Stepper::PutStep(uint32_t aPeriod) {
  pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
}

} // namespace PIOStepperSpeedController