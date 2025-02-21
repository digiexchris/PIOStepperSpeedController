
#pragma once

#include "Converter.hxx"
#include <concepts>
#include <cstdint>

namespace PIOStepperSpeedController {

enum class StepperState {
  STOPPED,
  STOPPING,
  STARTING,
  ACCELERATING,
  COASTING,
  DECELERATING
};

enum class CallbackEvent { STOPPED, ACCELERATING, DECELERATING, COASTING };

using Callback = void (*)(CallbackEvent event);

template <typename Derived> class Stepper;

template <typename Derived>
concept StepperImpl = requires(Derived stepper, uint32_t aPeriod) {
  {stepper.EnableImpl()};
  {stepper.DisableImpl()};
  { stepper.PutStep(aPeriod) } -> std::convertible_to<bool>;
}
&&std::derived_from<Derived, Stepper<Derived>>;

template <typename Derived> class Stepper {
public:
  Stepper(uint32_t aAcceleration = 1000, uint32_t aDeceleration = 1000,
          uint32_t aSysClk = 125000000, uint32_t aPrescaler = 1,
          Callback aStoppedCallback = nullptr,
          Callback aCoastingCallback = nullptr,
          Callback aAcceleratingCallback = nullptr,
          Callback aDeceleratingCallback = nullptr)
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
    myMinFrequency = 1;
    myMaxFrequency = myConverter.ToFrequency(myMinPeriod);
    myCurrentPeriod = myMaxPeriod;
    myTargetPeriod = myMaxPeriod;
    myIsRunning = false;
    YOU WERE CONVERTING PERIOD TO FREQUENCY
  }

  void Start() {

    if (!myIsRunning) {
      myCurrentPeriod = myMaxPeriod;
      static_cast<Derived *>(this)->EnableImpl();
      TransitionTo(StepperState::STARTING);
      myIsRunning = true;
    }
  }
  void Stop() {
    if (myState == StepperState::STOPPED || myState == StepperState::STOPPING) {
      return;
    }

    myState = StepperState::STOPPING;
  }

  bool Update() {
    if (!myIsRunning) {
      return false;
    }

    switch (myState) {
    case StepperState::STOPPED:
      return false;
      break;
    case StepperState::STOPPING:
      if (myCurrentPeriod >= myMaxPeriod) {
        myIsRunning = false;
        static_cast<Derived *>(this)->DisableImpl();
        TransitionTo(StepperState::STOPPED);
        return false;
      } else {
        Step(StepperState::DECELERATING);
      }
      break;
    case StepperState::STARTING:
      if (myCurrentPeriod > myTargetPeriod) {
        TransitionTo(StepperState::ACCELERATING);
        Step(StepperState::ACCELERATING);
      } else if (myCurrentPeriod <= myTargetPeriod) {
        TransitionTo(StepperState::COASTING);
        Step(StepperState::COASTING);
      } else {
        TransitionTo(StepperState::ACCELERATING);
        Step(StepperState::STARTING);
      }
      break;
    case StepperState::ACCELERATING:
      if (myCurrentPeriod > myTargetPeriod) {
        Step(StepperState::ACCELERATING);
      } else {
        TransitionTo(StepperState::COASTING);
        Step(StepperState::COASTING);
      }
      break;
    case StepperState::COASTING:
      if (myCurrentPeriod == myTargetPeriod) {
        Step(StepperState::COASTING);
      } else {
        // only thing that can result in a speed change after coasting is a
        // SetTargetHz or Stop
        return false;
      }
      break;
    case StepperState::DECELERATING:
      if (myCurrentPeriod < myTargetPeriod) {
        Step(StepperState::DECELERATING);
      } else {
        TransitionTo(StepperState::COASTING);
        Step(StepperState::COASTING);
      }
      break;
    }

    return true;
  }

  void SetTargetHz(uint32_t aSpeedHz) {
    if (aSpeedHz == 0) {
      myTargetPeriod = myMaxPeriod;
      return;
    }

    uint32_t targetPeriod = myConverter.ToPeriod(static_cast<float>(aSpeedHz));

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
      return myConverter.ToFrequency(myCurrentPeriod);
    }
  }

  float GetTargetFrequency() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return myConverter.ToFrequency(myTargetPeriod);
    }
  }
  StepperState GetState() { return myState; }

private:
  bool Step(StepperState aState) {
    switch (aState) {
    case StepperState::STARTING: {
      static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
      return true;
    } break;

    case StepperState::ACCELERATING: {
      int32_t nextPeriod = myConverter.ToPeriod(
          myConverter.CalculateNextFrequency(myCurrentPeriod, myAcceleration));

      if (nextPeriod <= myMinPeriod) {
        myCurrentPeriod = myMinPeriod;
        myTargetPeriod = myMinPeriod;
      } else if (nextPeriod <= myTargetPeriod) {
        myCurrentPeriod = myTargetPeriod;
      } else {
        myCurrentPeriod = nextPeriod;
      }
      static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
      return true;
    } break;

    case StepperState::DECELERATING: {
      int32_t nextPeriod = myConverter.ToPeriod(
          myConverter.CalculateNextFrequency(myCurrentPeriod, -myDeceleration));
      myCurrentPeriod = nextPeriod;
      if (nextPeriod >= myTargetPeriod) {
        myCurrentPeriod = myTargetPeriod;
      }

      if (myCurrentPeriod >= myMaxPeriod) {
        myCurrentPeriod = myMaxPeriod;
      }

      static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
      return true;
    }

    break;
    case StepperState::COASTING: {
      static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
      return true;
    } break;
    default:
      break;
    }
    return false;
  }
  void TransitionTo(StepperState aState) {
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
  Converter myConverter;

  // Function pointers (typically 8 bytes on 64-bit systems)
  Callback myStoppedCallback;
  Callback myCoastingCallback;
  Callback myAcceleratingCallback;
  Callback myDeceleratingCallback;

  // 4-byte aligned members
  uint32_t myAcceleration;
  uint32_t myDeceleration;
  uint32_t mySysClk;
  uint32_t myPrescaler;
  uint32_t myCurrentPeriod;
  uint32_t myTargetPeriod;
  uint32_t myMaxPeriod;
  uint32_t myMinPeriod;
  float myMaxFrequency;
  float myMinFrequency;
  float myCurrentFrequency;

  // 1-byte members
  StepperState myState;
  bool myIsRunning;
};

} // namespace PIOStepperSpeedController