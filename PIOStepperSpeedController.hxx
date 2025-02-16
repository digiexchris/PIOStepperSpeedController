#pragma once

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include <cstdint>

namespace PIOStepperSpeedController {
class Stepper {
public:
  enum class CallbackEvent { STOPPED, ACCELERATING, DECELERATING, COASTING };

  using Callback = void (*)(CallbackEvent event);

  Stepper(uint stepPin, uint startSpeedHz, uint maxSpeed, uint stepsPerRotation,
          uint acceleration, uint deceleration,
          Callback aStoppedCallback = nullptr,
          Callback aCoastingCallback = nullptr,
          Callback aAcceleratingCallback = nullptr,
          Callback aDeceleratingCallback = nullptr);

  void Init(float div, int stepsPerRotation, int32_t acceleration,
            int32_t deceleration);
  void Start(float targetHz);
  void ForceStop();
  bool Step();
  void SetTargetHz(float hz) { myTargetHz = hz; }
  uint32_t GetCurrentPeriod() const { return myCurrentPeriod; }
  uint32_t GetCurrentFrequency() const {
    return PrivPeriodToFrequency(myCurrentPeriod);
  }

private:
  void PrivSetEnabled(bool enabled);
  // Period calculation methods
  int32_t PrivCalculateNextPeriod(uint32_t currentPeriod, float targetHz);
  uint32_t PrivFrequencyToPeriod(float hz) const;
  float PrivPeriodToFrequency(uint32_t period) const;

  // Callbacks
  Callback myStoppedCallback;
  Callback myCoastingCallback;
  Callback myAcceleratingCallback;
  Callback myDeceleratingCallback;

  CallbackEvent myState;

  // Program definition constants
  static constexpr uint SIDESET_BITS = 1;
  static constexpr uint WRAP_TARGET = 2;
  static constexpr uint WRAP = 6;

  PIO myPio;
  uint mySm;
  uint myOffset;
  uint myStepPin;

  uint32_t mySysClk;
  float myConfiguredDiv;
  uint myMaxSpeed;
  int myStepsPerRotation;
  uint32_t myAcceleration;
  uint32_t myDeceleration;
  uint32_t myCurrentPeriod;
  uint32_t myTargetHz;
  bool myIsRunning;
  uint32_t myStartSpeedHz;

  static constexpr double PI = 3.14159265358979323846;
};

} // namespace PIOStepperSpeedController