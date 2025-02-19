
#pragma once

#include "Converter.hxx"
#include <concepts>
#include <cstdint>

namespace PIOStepperSpeedController {

template <typename T>
concept Stepper = requires(T stepper) {
  {stepper.StartImpl()};
  {stepper.StopImpl()};
  { stepper.UpdateImpl() } -> std::convertible_to<bool>;
  {stepper.SetTargetHzImpl(uint32_t aTargetHz)};
  { stepper.GetCurrentPeriodImpl() } -> std::convertible_to<uint32_t>;
  { stepper.GetCurrentFrequencyImpl() } -> std::floating_point;
  { stepper.GetTargetFrequencyImpl() } -> std::floating_point;
}
&&std::derived_from<T, Stepper>;

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
  bool Update();
  void SetTargetHz(uint32_t aSpeedHz);
  uint32_t GetCurrentPeriod() const;
  float GetCurrentFrequency() const;
  float GetTargetFrequency() const;
  StepperState GetState();

private:
  void TransitionTo(StepperState aState);
  uint32_t myAcceleration;
  uint32_t myDeceleration;
  uint32_t mySysClk;
  uint32_t myPrescaler;
  Callback myStoppedCallback;
  Callback myCoastingCallback;
  Callback myAcceleratingCallback;
  Callback myDeceleratingCallback;
  StepperState myState;
  Converter myConverter;
};

} // namespace PIOStepperSpeedController