#pragma once

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include <cstdint>

namespace PIOStepperSpeedController {
class Stepper {
public:
  enum class CallbackEvent { STOPPED, ACCELERATING, DECELERATING, COASTING };

  enum class StepperState {
    STOPPED,
    STOPPING,
    STARTING,
    ACCELERATING,
    COASTING,
    DECELERATING
  };

  using Callback = void (*)(CallbackEvent event);

  Stepper(uint32_t stepPin, uint32_t startSpeedHz, uint32_t maxSpeedHz,
          uint32_t stepsPerRotation, uint32_t acceleration,
          uint32_t deceleration, Callback aStoppedCallback = nullptr,
          Callback aCoastingCallback = nullptr,
          Callback aAcceleratingCallback = nullptr,
          Callback aDeceleratingCallback = nullptr);

  void Start();
  void Stop();
  bool Update();
  void SetTargetHz(uint32_t aSpeedHz);
  uint32_t GetCurrentPeriod() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return myCurrentPeriod;
    }
  }
  float GetCurrentFrequency() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return PrivPeriodToFrequency(myCurrentPeriod);
    }
  }

  float GetTargetFrequency() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return PrivPeriodToFrequency(myTargetPeriod);
    }
  }

  StepperState GetState() { return myState; }

private:
  // Period calculation methods
  void TransitionTo(StepperState aState);
  float PrivNearestPeriodToFrequency(float hz) const;
  uint32_t PrivFrequencyToPeriod(float hz) const;
  float PrivPeriodToFrequency(uint32_t period) const;
  uint32_t PrivSecondsToPeriod(float seconds) const;
  bool Step(StepperState state);

  /**
          @brief Calculate the time interval between the previous step and
     this step
          @param stepsPerRotation The number of steps per rotation of the
     stepper motor
          @param acceleration The acceleration of the stepper motor in steps
     per divided clock cycle (not per second) (negative for deceleration)
          @param previousInterval The previous interval in seconds
          @return The next interval in seconds
   */
  uint32_t CalculateNextPeriod(int stepsPerRotation, uint32_t currentPeriod,
                               int32_t accelerationStepsPerCyclequared);

  // Callbacks
  Callback myStoppedCallback;
  Callback myCoastingCallback;
  Callback myAcceleratingCallback;
  Callback myDeceleratingCallback;

  StepperState myState;

  PIO myPio;
  uint mySm;
  uint myOffset;
  uint myStepPin;
  bool mySmIsEnabled = false;

  uint32_t mySysClk;
  float myConfiguredPrescaler;
  uint32_t myMinPeriod; // aka max speed
  uint32_t myMaxPeriod; // aka minimum speed
  int myStepsPerRotation;
  uint32_t
      myAcccelerationCyclesPerPeriodChange; // in number of divided clock cycles
                                            // per 1 integer period change
  uint32_t
      myDecelerationCyclesPerPeriodChange; // in number of divided clock cycles
                                           // per 1 integer period change
  uint32_t myCurrentPeriod;
  uint32_t myTargetPeriod;
  uint32_t myStartPeriod;
  double myStepAngleRadians;

  const double PI = 3.14159265358979323846;
};

} // namespace PIOStepperSpeedController