#pragma once

#include "IStepper.hxx"
#include <cstdint>

namespace PIOStepperSpeedController {

class PIOStepper : public IStepper {

public:
  PIOStepper(uint32_t stepPin, uint32_t startSpeedHz, uint32_t maxSpeedHz,
             uint32_t aAcceleration = 1000, uint32_t aDeceleration = 1000,
             uint32_t aSysClk = 125000000, uint32_t aPrescaler = 1,
             Callback aStoppedCallback = nullptr,
             Callback aCoastingCallback = nullptr,
             Callback aAcceleratingCallback = nullptr,
             Callback aDeceleratingCallback = nullptr);

  void StartImpl() override;
  void StopImpl() override;
  bool UpdateImpl() override;
  void SetTargetHzImpl(uint32_t aTargetHz) override;
  uint32_t GetCurrentPeriodImpl() override;
  float GetCurrentFrequencyImpl() override;
  float GetTargetFrequencyImpl() override;

private:
  uint32_t myStepPin;
};

} // namespace PIOStepperSpeedController
