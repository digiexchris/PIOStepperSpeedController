#include "PIOStepperSpeedController.hxx"
#include "PIOStepperSpeedController.pio.h"
#include <cassert> // Needed for assert()
#include <cmath>   // Needed for sqrt() and PI calculations
#include <cstdint> // Needed for uint32_t, int32_t types

namespace PIOStepperSpeedController {
Stepper::Stepper(uint stepPin, uint startSpeedHz, uint maxSpeed,
                 uint stepsPerRotation, uint acceleration, uint deceleration,
                 Callback aStoppedCallback, Callback aCoastingCallback,
                 Callback aAcceleratingCallback, Callback aDeceleratingCallback)
    : myStepPin(stepPin), myStartSpeedHz(startSpeedHz), myIsRunning(false),
      myTargetHz(0), myCurrentPeriod(0), myMaxSpeed(maxSpeed),
      myStepsPerRotation(stepsPerRotation), myAcceleration(acceleration),
      myDeceleration(deceleration), myOffset(0), mySysClk(0),
      myStoppedCallback(aStoppedCallback),
      myCoastingCallback(aCoastingCallback),
      myAcceleratingCallback(aAcceleratingCallback),
      myDeceleratingCallback(aDeceleratingCallback) {

  assert(startSpeedHz > 0);
  assert(maxSpeed > 0);
  assert(stepsPerRotation > 0);
  assert(acceleration > 0);
  assert(deceleration > 0);

  mySysClk = clock_get_hz(clk_sys);

  // Configure state machine
  // Calculate clock divider: Keep the time range between 0 and 2^32-1 cycles
  // For timing calculation: period = (sysclk / div) / (2 * frequency)
  // Maximum period should be less than 2^32-1 (will be truncated to 32 bits)
  mySysClk = clock_get_hz(clk_sys);
  myConfiguredDiv = std::max(1.0f, std::ceil(static_cast<float>(mySysClk) /
                                             (2.0f * maxSpeed * (1ULL << 32))));

  bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
      &StepperSpeedController_program, &myPio, &mySm, &myOffset, stepPin, 1,
      true);
  assert(success);

  // GPIO setup
  pio_gpio_init(myPio, stepPin);
  pio_sm_set_consecutive_pindirs(myPio, mySm, stepPin, 1, true);

  // SM configuration
  pio_sm_config c = StepperSpeedController_program_get_default_config(myOffset);
  sm_config_set_wrap(&c, myOffset + WRAP_TARGET, myOffset + WRAP);
  sm_config_set_sideset(&c, SIDESET_BITS, true, false);
  sm_config_set_sideset_pins(&c, stepPin);
  sm_config_set_out_pins(&c, stepPin, 1);
  sm_config_set_clkdiv(&c, myConfiguredDiv);

  // Initialize and clear
  pio_sm_init(myPio, mySm, myOffset, &c);
  pio_sm_clear_fifos(myPio, mySm);

  myState = CallbackEvent::STOPPED;
}

void Stepper::Start(float targetHz) {
  assert(targetHz > 0);
  myTargetHz = targetHz;
  if (myState != CallbackEvent::STOPPED) {
    return;
  }

  myCurrentPeriod = PrivFrequencyToPeriod(myStartSpeedHz);
  pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
  // Jump to start of program (offset 0)
  pio_sm_exec(myPio, mySm, pio_encode_jmp(myOffset));
  pio_sm_set_enabled(myPio, mySm, true);
}

void Stepper::ForceStop() {
  pio_sm_set_enabled(myPio, mySm, false);
  gpio_put(myStepPin, 0);
  myState = CallbackEvent::STOPPED;
  if (myStoppedCallback != nullptr) {
    myStoppedCallback(CallbackEvent::STOPPED);
  }
}

bool Stepper::Step() {
  if (myState == CallbackEvent::STOPPED) {
    return false;
  }

  // Calculate next period using internal target_hz
  int32_t nextPeriod = PrivCalculateNextPeriod(myCurrentPeriod, myTargetHz);

  if (nextPeriod >= PrivFrequencyToPeriod(myStartSpeedHz)) {
    ForceStop();
    return false;
  }

  // todo: calculate this in the constructor instead and cache it
  auto minPeriod = PrivFrequencyToPeriod(myMaxSpeed);

  // it's gone as fast as it's configured to top out at, stop going faster
  if (nextPeriod < minPeriod) {
    nextPeriod = minPeriod;
  }

  // Check if we're coasting
  else if (nextPeriod == myCurrentPeriod) {
    if (myState != CallbackEvent::COASTING) {
      myState = CallbackEvent::COASTING;
      if (myCoastingCallback != nullptr) {
        myCoastingCallback(CallbackEvent::COASTING);
      }
    }
  }

  else if (nextPeriod < myCurrentPeriod) {
    if (myState != CallbackEvent::ACCELERATING) {
      myState = CallbackEvent::ACCELERATING;
      if (myDeceleratingCallback != nullptr) {
        myDeceleratingCallback(CallbackEvent::ACCELERATING);
      }
    }
  }

  else if (nextPeriod > myCurrentPeriod) {

    if (myState != CallbackEvent::DECELERATING) {
      myState = CallbackEvent::DECELERATING;
      if (myDeceleratingCallback != nullptr) {
        myDeceleratingCallback(CallbackEvent::DECELERATING);
      }
    } else if (nextPeriod >= PrivFrequencyToPeriod(myTargetHz)) {
      ForceStop();
      return false;
    }
  }

  // Update period and step
  myCurrentPeriod = nextPeriod;
  pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
  return true;
}

void Stepper::PrivSetEnabled(bool enabled) {
  pio_sm_set_enabled(myPio, mySm, enabled);
  if (!enabled) {
    gpio_put(myStepPin, 0);
  }
}

int32_t Stepper::PrivCalculateNextPeriod(uint32_t currentPeriod,
                                         float targetHz) {
  if (currentPeriod <= 0) {
    // First interval calculation
    double alpha = 2 * PI / myStepsPerRotation;
    if (myAcceleration <= 0) {
      return 0; // Prevent divide by zero
    }
    return (mySysClk / myConfiguredDiv) * std::sqrt(2 * alpha / myAcceleration);
  }

  // Calculate current speed in Hz (protect against divide by zero)
  if (currentPeriod == 0) {
    return 0;
  }
  double currentHz = (mySysClk / myConfiguredDiv) / (2.0 * currentPeriod);

  if (targetHz == 0) {
    // Specifically handle stopping case
    double currentHz = PrivPeriodToFrequency(currentPeriod);
    if (currentHz <= 0.0f) {
      return 0; // This will trigger a stop in the Step() function
    }
  }

  // Determine if we need to accelerate or decelerate
  if (currentHz > targetHz) {
    // Need to decelerate - increase period
    double alpha = 2 * PI / myStepsPerRotation;
    double currentInterval = currentPeriod * myConfiguredDiv / mySysClk;
    double radsPerSecondSquared =
        myDeceleration * alpha; // Use deceleration value

    if (alpha == 0) {
      return currentPeriod; // Prevent divide by zero
    }

    double nextInterval =
        currentInterval +
        (2 * currentInterval * currentInterval * radsPerSecondSquared) / alpha;

    // Check if we're stopping and would go below zero Hz
    if (targetHz == 0) {
      if (nextInterval <= 0) {
        return currentPeriod; // Prevent divide by zero
      }
      double nextHz = mySysClk / (myConfiguredDiv * nextInterval * 2.0);
      if (nextHz <= 0) {
        return 0;
      }
    }

    return (mySysClk / myConfiguredDiv) * nextInterval;
  } else if (currentHz < targetHz) {
    // Need to accelerate - decrease period
    double alpha = 2 * PI / myStepsPerRotation;
    if (alpha == 0) {
      return currentPeriod; // Prevent divide by zero
    }

    double currentInterval = currentPeriod * myConfiguredDiv / mySysClk;
    double radsPerSecondSquared = myAcceleration * alpha;

    double nextInterval =
        currentInterval -
        (2 * currentInterval * currentInterval * radsPerSecondSquared) / alpha;

    return (mySysClk / myConfiguredDiv) * nextInterval;
  }

  // At target speed - maintain current period
  return currentPeriod;
}

uint32_t Stepper::PrivFrequencyToPeriod(float hz) const {
  return hz > 0 ? (mySysClk / (myConfiguredDiv * hz * 2)) : 0;
}

float Stepper::PrivPeriodToFrequency(uint32_t period) const {
  return period > 0 ? (mySysClk / (myConfiguredDiv * period * 2.0)) : 0;
}
} // namespace PIOStepperSpeedController