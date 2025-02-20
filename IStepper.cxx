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
  myMinPeriod = 1;
  myMaxPeriod = UINT32_MAX - 1;
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

bool IStepper::Step(StepperState aState) {
  switch (aState) {
  case StepperState::STARTING: {
    PutStep(myCurrentPeriod);
    return true;
  } break;

  case StepperState::ACCELERATING: {
    int32_t nextPeriod =
        myConverter.ToPeriod(myConverter.CalculateNextFrequency(myCurrentPeriod,
                            myAcceleration);

    if (nextPeriod <= myMinPeriod) {
      myCurrentPeriod = myMinPeriod;
      myTargetPeriod = myMinPeriod;
    } else if (nextPeriod <= myTargetPeriod) {
      myCurrentPeriod = myTargetPeriod;
    } else {
      myCurrentPeriod = nextPeriod;
    }
    PutStep(myCurrentPeriod);
    return true;
  } break;

  case StepperState::DECELERATING: {
    int32_t nextPeriod = myConverter.ToPeriod(myConverter.CalculateNextFrequency(myCurrentPeriod,
                            -myDeceleration);
    myCurrentPeriod = nextPeriod;
    if (nextPeriod >= myTargetPeriod) {
      myCurrentPeriod = myTargetPeriod;
    }

    if (myCurrentPeriod >= myStartPeriod) {
      myCurrentPeriod = myStartPeriod;
    }

    PutStep(myCurrentPeriod);
    return true;
  }

  break;
  case StepperState::COASTING: {
    PutStep(myCurrentPeriod);
    return true;
  } break;
  default:
    assert(false);
  }
  return false;
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
    TransitionTo(StepperState::STARTING);
    mySmIsEnabled = true;
  }
}

void IStepper::Update() {
  if (!mySmIsEnabled) { // todo rename to myIsRunning
    return false;
  }

  switch (myState) {
  case StepperState::STOPPED:
    return false;
    break;
  case StepperState::STOPPING:
    if (myCurrentPeriod >= myStartPeriod) {
      mySmIsEnabled = false;
      DisableImpl();
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

void IStepper::SetTargetHz(uint32_t aSpeedHz) {
  if (hz == 0) {
    myTargetPeriod = myStartPeriod;
    return;
  }

  uint32_t targetPeriod = myConverter.ToPeriod(static_cast<float>(hz));

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

uint32_t IStepper::GetCurrentPeriod() const {
  if (myState == StepperState::STOPPED) {
    return 0;
  } else {
    return myCurrentPeriod;
  }
}

float IStepper::GetCurrentFrequency() const {
  if (myState == StepperState::STOPPED) {
    return 0;
  } else {
    return myConverter.ToFrequency(myCurrentPeriod);
  }
}

float IStepper::GetTargetFrequency() const {
  if (myState == StepperState::STOPPED) {
    return 0;
  } else {
    return myConverter.ToFrequency(myTargetPeriod);
  }
}

StepperState IStepper::GetState() { return myState; }

} // namespace PIOStepperSpeedController