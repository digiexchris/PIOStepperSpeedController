#include "PIOStepperSpeedController.hxx"
#include "PIOStepperSpeedController.pio.h"
#include <cassert> // Needed for assert()
#include <cmath>   // Needed for sqrt() and PI calculations
#include <cstdint> // Needed for uint32_t, int32_t types
#include <hardware/pio.h>

namespace PIOStepperSpeedController {
Stepper::Stepper(uint32_t stepPin, uint32_t startSpeedHz, uint32_t maxSpeedHz,
                 uint32_t stepsPerRotation, uint32_t acceleration,
                 uint32_t deceleration, Callback aStoppedCallback,
                 Callback aCoastingCallback, Callback aAcceleratingCallback,
                 Callback aDeceleratingCallback)
    : myStepPin(stepPin), myCurrentPeriod(0),
      myStepsPerRotation(stepsPerRotation), myOffset(0), mySysClk(0),
      myStoppedCallback(aStoppedCallback),
      myCoastingCallback(aCoastingCallback),
      myAcceleratingCallback(aAcceleratingCallback),
      myDeceleratingCallback(aDeceleratingCallback) {

  assert(startSpeedHz > 0);
  assert(maxSpeedHz > 0);
  assert(stepsPerRotation > 0);
  assert(acceleration > 0);
  assert(deceleration > 0);

  // Configure state machine
  // Calculate clock divider: Keep the time range between 0 and 2^32-1 cycles
  // For timing calculation: period = (sysclk / div) / (2 * frequency)
  // Maximum period should be less than 2^32-1 (will be truncated to 32 bits)
  mySysClk = clock_get_hz(clk_sys);
  // At max speed (div=1): period = (sysclk/div)/(freq) = 12,500

  // IMPORTANT: Periods are in clock cycles, not any time based unit such as
  // seconds!!!!

  // So I remember this:
  // The resulting period is essentially p(clk*divisor) units.
  // So, if we have a divisor of 2, and a sysclk of 125,000,000, and a period of
  // 10, we have 10 divided clock cycles.

  // example breakdown with 1 as a divisor and a 10khz frequency:
  // sysclk = 125MHz = one cycle every 8ns
  // sysclk period = 12,500 sysclk cycles
  // real time = 12,500 * 8ns = 50µs

  // So, if we have a divisor of 2, we have 12,500 * 2 = 25000 sysclk cycles
  // real time = 25000 * 8ns = 200µs

  // myConfiguredPrescaler =
  //     mySysClk / maxSpeedHz; // should give us a divider that gives us our
  //     max
  // This gives a range of 1hz = UINT32_MAX,
  myConfiguredPrescaler = 68.72f;

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

bool Stepper::Update() {

  if (!mySmIsEnabled) {
    return false;
  }

  switch (myState) {
  case StepperState::STOPPED:
    return false;
    break;
  case StepperState::STOPPING:
    if (myCurrentPeriod >= myStartPeriod) {
      pio_sm_set_enabled(myPio, mySm, false);
      mySmIsEnabled = false;
      gpio_put(myStepPin, 0);
      TransitionTo(StepperState::STOPPED);
      return false;
    } else {
      return Step(StepperState::DECELERATING);
    }
    break;
  case StepperState::STARTING:
    if (myCurrentPeriod < myStartPeriod) {
      TransitionTo(StepperState::ACCELERATING);
      return Step(StepperState::ACCELERATING);
    } else if (myCurrentPeriod <= myTargetPeriod) {
      TransitionTo(StepperState::COASTING);
      return Step(StepperState::COASTING);
    } else {
      TransitionTo(StepperState::ACCELERATING);
      return Step(StepperState::STARTING);
    }
  case StepperState::ACCELERATING:
    if (myCurrentPeriod > myTargetPeriod) {
      return Step(StepperState::ACCELERATING);
    } else {
      TransitionTo(StepperState::COASTING);
      return Step(StepperState::COASTING);
    }
    break;
  case StepperState::COASTING:
    if (myCurrentPeriod == myTargetPeriod) {
      return Step(StepperState::COASTING);
    } else {
      // only thing that can result in a speed change after coasting is a
      // SetTargetHz or Stop
      return false;
    }
    break;
  case StepperState::DECELERATING:
    if (myCurrentPeriod < myTargetPeriod) {
      return Step(StepperState::DECELERATING);
    } else {
      TransitionTo(StepperState::COASTING);
      return Step(StepperState::COASTING);
    }
    break;
  }

  return false;
}

bool Stepper::Step(StepperState state) {

  switch (state) {
  case StepperState::STARTING: {
    pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
    return true;
  } break;

  case StepperState::ACCELERATING: {
    int32_t nextPeriod =
        CalculateNextPeriod(myStepsPerRotation, myCurrentPeriod,
                            myAcccelerationCyclesPerPeriodChange);

    if (nextPeriod <= myMinPeriod) {
      myCurrentPeriod = myMinPeriod;
      myTargetPeriod = myMinPeriod;
    } else if (nextPeriod <= myTargetPeriod) {
      myCurrentPeriod = myTargetPeriod;
    } else {
      myCurrentPeriod = nextPeriod;
    }
    pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
    return true;
  } break;

  case StepperState::DECELERATING: {
    int32_t nextPeriod = CalculateNextPeriod(
        myStepsPerRotation,

        myCurrentPeriod, -myAcccelerationCyclesPerPeriodChange);
    myCurrentPeriod = nextPeriod;
    if (nextPeriod >= myTargetPeriod) {
      myCurrentPeriod = myTargetPeriod;
    }

    if (myCurrentPeriod >= myStartPeriod) {
      myCurrentPeriod = myStartPeriod;
    }

    pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
    return true;
  }

  break;
  case StepperState::COASTING: {
    pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
    return true;
  } break;
  default:
    assert(false);
  }
  return false;
}

// todo unused and probably wrong
uint32_t Stepper::PrivSecondsToPeriod(float seconds) const {
  return seconds > 0 ? seconds * mySysClk / myConfiguredPrescaler
                     : myStartPeriod;
}

float Stepper::PrivNearestPeriodToFrequency(float hz) const {
  float newPeriod =
      (static_cast<float>(mySysClk) / (myConfiguredPrescaler)) / hz;
  return std::round(newPeriod);
}

uint32_t Stepper::PrivFrequencyToPeriod(float hz) const {
  if (hz <= 0.0f) {
    return myStartPeriod;
  }

  float speedRatio = myMaxPeriod / hz;

  float newPeriod = static_cast<float>(mySysClk) / (myConfiguredPrescaler) / hz;

  return static_cast<uint32_t>(std::floor(newPeriod));
}

float Stepper::PrivPeriodToFrequency(uint32_t period) const {
  if (period > 0 && period < UINT32_MAX) {
    return static_cast<float>(mySysClk) / (myConfiguredPrescaler * period);
  }
  return 0;
}

void Stepper::SetTargetHz(uint32_t hz) {

  if (hz == 0) {
    myTargetPeriod = myStartPeriod;
    return;
  }

  uint32_t targetPeriod = PrivNearestPeriodToFrequency(static_cast<float>(hz));

  if (targetPeriod > myStartPeriod) {
    targetPeriod = myStartPeriod;
  }

  if (targetPeriod < myCurrentPeriod) {
    if (targetPeriod < myMinPeriod) {
      targetPeriod = myMinPeriod;
    }
    if (myState != StepperState::STARTING &&
        myState != StepperState::ACCELERATING) {
      TransitionTo(StepperState::ACCELERATING);
    }

  }

  else if (targetPeriod > myCurrentPeriod) {
    if (myState != StepperState::DECELERATING) {
      TransitionTo(StepperState::DECELERATING);
    }
  }

  myTargetPeriod = targetPeriod;
}

uint32_t Stepper::CalculateNextPeriod(int stepsPerRotation,
                                      uint32_t currentPeriod,
                                      int32_t cyclesPerPeriodChange) {
  if (cyclesPerPeriodChange == 0) {
    return currentPeriod;
  }

  return currentPeriod - cyclesPerPeriodChange;
}

} // namespace PIOStepperSpeedController