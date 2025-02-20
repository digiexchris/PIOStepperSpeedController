
#pragma once

#include "Converter.hxx"
#include <concepts>
#include <cstdint>

namespace PIOStepperSpeedController {

template <typename T>
concept Stepper = requires(T stepper) {
  {stepper.EnableImpl()};
  {stepper.DisableImpl()};
  {stepper.PutStep(uint32_t aPeriod)};
  ->std::convertable_to<bool>;
}
&&std::derived_from<IStepper>;

class IStepper {

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

  IStepper(uint32_t aAcceleration = 1000, uint32_t aDeceleration = 1000,
           uint32_t aSysClk = 125000000, uint32_t aPrescaler = 1,
           Callback aStoppedCallback = nullptr,
           Callback aCoastingCallback = nullptr,
           Callback aAcceleratingCallback = nullptr,
           Callback aDeceleratingCallback = nullptr);

  void Start();
  void Stop();
  void Step(StepperState state);
  bool Update();
  void SetTargetHz(uint32_t aSpeedHz);
  uint32_t GetCurrentPeriod() const;
  float GetCurrentFrequency() const;
  float GetTargetFrequency() const;
  StepperState GetState();

private:
  void TransitionTo(StepperState aState);
  Converter myConverter;
  uint32_t myAcceleration;
  uint32_t myDeceleration;
  uint32_t mySysClk;
  uint32_t myPrescaler;
  Callback myStoppedCallback;
  Callback myCoastingCallback;
  Callback myAcceleratingCallback;
  Callback myDeceleratingCallback;
  StepperState myState;

  uint32_t myCurrentPeriod;
  uint32_t myTargetPeriod;
  uint32_t myStartPeriod;
  uint32_t myMaxPeriod;
  uint32_t myMinPeriod;
};

} // namespace PIOStepperSpeedController