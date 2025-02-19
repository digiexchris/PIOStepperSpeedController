#include "IStepper.hxx"
#include "Converter.hxx"

namespace PIOStepperSpeedController {

IStepper::IStepper(uint32_t aAcceleration, uint32_t aDeceleration,
                   uint32_t aSysClk, uint32_t aPrescaler,
                   Callback aStoppedCallback, Callback aCoastingCallback,
                   Callback aAcceleratingCallback,
                   Callback aDeceleratingCallback)
    : myAcceleration(aAcceleration), myDeceleration(aDeceleration),
      mySysClk(aSysClk), myPrescaler(aPrescaler),
      myStoppedCallback(aStoppedCallback),
      myCoastingCallback(aCoastingCallback),
      myAcceleratingCallback(aAcceleratingCallback),
      myDeceleratingCallback(aDeceleratingCallback),
      myState(StepperState::STOPPED) {
  myConverter = Converter(mySysClk, myPrescaler);
}

void IStepper::TransitionTo(StepperState aState) {
  switch (aState) {
  case StepperState::ACCELERATING:
    if (myState != StepperState::ACCELERATING) {
      myState = StepperState::ACCELERATING;
      if (myAcceleratingCallback != nullptr) {
        myAcceleratingCallback(CallbackEvent::ACCELERATING);
      }
    }
    break;

  case StepperState::COASTING:
    if (myState != StepperState::COASTING) {
      myState = StepperState::COASTING;
      if (myCoastingCallback != nullptr) {
        myCoastingCallback(CallbackEvent::COASTING);
      }
    }
    break;

  case StepperState::DECELERATING:
    if (myState != StepperState::DECELERATING) {
      myState = StepperState::DECELERATING;
      if (myDeceleratingCallback != nullptr) {
        myDeceleratingCallback(CallbackEvent::DECELERATING);
      }
    }
    break;

  case StepperState::STARTING:
    if (myState != StepperState::STARTING) {
      myState = StepperState::STARTING;
    }
    break;

  case StepperState::STOPPING:
    if (myState != StepperState::STOPPING) {
      myState = StepperState::STOPPING;
    }
    break;

  case StepperState::STOPPED:
    if (myState != StepperState::STOPPED) {
      myState = StepperState::STOPPED;
      if (myStoppedCallback != nullptr) {
        myStoppedCallback(CallbackEvent::STOPPED);
      }
    }
    break;
  }
}

void IStepper::Stop() {
  if (myState == StepperState::STOPPED || myState == StepperState::STOPPING) {
    return;
  }

  myState = StepperState::STOPPING;
}

void IStepper::Start() {

  if (!mySmIsEnabled) {
    myCurrentPeriod = myStartPeriod;
    StartImpl();
    pio_sm_set_enabled(myPio, mySm, true);
    TransitionTo(StepperState::STARTING);
    mySmIsEnabled = true;
  }
}
} // namespace PIOStepperSpeedController