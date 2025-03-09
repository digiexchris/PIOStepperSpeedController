#pragma once

#include "Converter.hxx"
#include <algorithm>
#include <concepts>
#include <cstdint>

namespace PIOStepperSpeedController {

  static bool IsEq(float a, float b, float epsilon = 0.1f) {
    return std::abs(a - b) < epsilon;
  }

  static bool IsLT(float a, float b, float epsilon = 0.1f) {
    return a < b - epsilon;
  }

  static bool IsLTEQ(float a, float b, float epsilon = 0.1f) {
    return a <= b || IsEq(a, b, epsilon);
  }

  static bool IsGT(float a, float b, float epsilon = 0.1f) {
    return a > b + epsilon;
  }

  static bool IsGTEQ(float a, float b, float epsilon = 0.1f) {
    return a >= b || IsEq(a, b, epsilon);
  }

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
concept StepperImpl = requires(Derived stepper, uint32_t aFrequency) {
  {stepper.EnableImpl()};
  {stepper.DisableImpl()};
  { stepper.PutStep(aFrequency) } -> std::convertible_to<bool>;
}
&&std::derived_from<Derived, Stepper<Derived>>;

template <typename Derived> class Stepper {
public:
  /**
  @brief Constructor for the Stepper class
  @param aMinSpeed Minimum speed in Hz
  Note: aMinSpeed impacts how long the first
  step takes. eg. 0.33hz will mean the first step takes 3.03 seconds regardless
  of acceleration! 4hz will take 0.25 seconds for the first step so is a
  reasonable starting speed for a human controlled power feed. It MUST be
  greater than 0.
  @param aMaxSpeed Maximum speed in Hz
  aMaxSpeed will be capped by the maximum possible speed provided by the clock
  speed and prescaler values if aMaxSpeed is greater than the maximum possible
  speed of the pio state machine for that configuration.
  @param aAcceleration Acceleration in Hz/s
  @param aDeceleration Deceleration in Hz/s. It is perfectly valid for
  deceleration and acceleration to match, but they must both be greater than
  zero.
  @param aSysClk System clock frequency in Hz. The RP2040 can go up to
  133,000,000, 125,000,000 is a common value and is the default. You may pass in
  what your system is configured for, but if you change this from how your
  system is configured, the math will be wrong. this does NOT configure your
  sysclk.
  @param aPrescaler Prescaler value. With a prescaler of 1, the system can
  operate at around 10,000 hz. Increase the prescaler if you need to go faster.
  See the Converter class for the formula. This configures the divisor for the
  pio state machine.

  Callbacks should be a reference to a function with a Callback signiture, eg.
  void (*)(CallbackEvent event); When the callback is called, it will pass in
  which callback event occurred.

  @param aStoppedCallback Callback function for stopped state. Optional, for
  your use. It will be called when the state machine stops.
  @param aCoastingCallback Callback function for coasting state. Optional, for
  your use. It will be called when the stepper is coasting.
  @param aAcceleratingCallback Callback function for accelerating state.
  Optional, for your use. It will be called when the stepper is accelerating.
  @param aDeceleratingCallback Callback function for decelerating state.
  Optional, for your use. It will be called when the stepper is decelerating.
  */
  Stepper(float aMinSpeed, float aMaxSpeed, uint32_t aAcceleration,
          uint32_t aDeceleration, uint32_t aSysClk = 125000000,
          uint32_t aPrescaler = 1, Callback aStoppedCallback = nullptr,
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
    myMinFrequency =
        std::max(myConverter.ToFrequency(UINT32_MAX - 1), aMinSpeed);
    myMaxFrequency = std::min(myConverter.ToFrequency(1), aMaxSpeed);
    myCurrentFrequency = myMinFrequency;
    myTargetFrequency = myMinFrequency;
    myRequestedFrequency = myMinFrequency;
    myIsRunning = false;
  }

  void Start() {
    if (!myIsRunning) {
      myCurrentFrequency = myMinFrequency;
      static_cast<Derived *>(this)->EnableImpl();
      
      myIsRunning = true;
    }
    
    //shoiuld be handled by the Starting case in Update()
    // // Set target frequency to the user's requested frequency (or min speed)
    // myTargetFrequency = std::max(myRequestedFrequency, myMinFrequency);
    
    TransitionTo(StepperState::STARTING);
  }

  void Stop() {
    if (myState == StepperState::STOPPED || myState == StepperState::STOPPING) {
      return;
    }

    myTargetFrequency = myMinFrequency;

    myState = StepperState::STOPPING;
  }

  bool Update() {
    if (!myIsRunning) {
      return false;
    }

    switch(myState) {
      case StepperState::STOPPED:
      case StepperState::STOPPING:

        //if the speed changed since last update and we are not stopping
        if(!IsEq(myTargetFrequency, myMinFrequency)) {
          myTargetFrequency = myMinFrequency;
        }
        myTargetFrequency = myMinFrequency;
        break;
      default:

        //if the speed changed since last update and we are not stopping
        if(!IsEq(myTargetFrequency, myRequestedFrequency)) {
          myTargetFrequency = myRequestedFrequency;
        }
        break;
    }

    switch (myState) {
    case StepperState::STOPPED:
      return false;
      break;
    case StepperState::STOPPING:
      if (IsLTEQ(myCurrentFrequency, myMinFrequency)) {
        myIsRunning = false;
        static_cast<Derived *>(this)->DisableImpl();
        TransitionTo(StepperState::STOPPED);
        return false;
      } else {
        Step(StepperState::DECELERATING);
      }
      break;
    case StepperState::STARTING:
      if (IsLT(myCurrentFrequency, myTargetFrequency)) {
        TransitionTo(StepperState::ACCELERATING);
        Step(StepperState::ACCELERATING);
      } else if (IsEq(myCurrentFrequency, myTargetFrequency)) {
        TransitionTo(StepperState::COASTING);
        Step(StepperState::COASTING);
      } else {
        TransitionTo(StepperState::DECELERATING);
        Step(StepperState::DECELERATING);
      }
      break;
    case StepperState::ACCELERATING:
      if (IsLT(myCurrentFrequency, myTargetFrequency)) {
        Step(StepperState::ACCELERATING);
      } else {
        TransitionTo(StepperState::COASTING);
        Step(StepperState::COASTING);
      }
      break;
    case StepperState::COASTING:
      if (IsEq(myCurrentFrequency,myTargetFrequency)) {
        Step(StepperState::COASTING);
      } else {
        if (IsGT(myCurrentFrequency,myTargetFrequency)) {
          TransitionTo(StepperState::DECELERATING);
          Step(StepperState::DECELERATING);
        } else {
          TransitionTo(StepperState::ACCELERATING);
          Step(StepperState::ACCELERATING);
        }
        return false;
      }
      break;
    case StepperState::DECELERATING:
    if (IsGT(myCurrentFrequency,myTargetFrequency)) {
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
    if (myState == StepperState::STOPPED || myState == StepperState::STOPPING) {
      // Store the requested frequency even during stopping, but don't change target
      // This ensures the speed is remembered for next Start()
      if (aSpeedHz > 0) {
        myRequestedFrequency = std::min(static_cast<float>(aSpeedHz), myMaxFrequency);
      }
      return;
    }

    if (aSpeedHz == 0) {
      myRequestedFrequency = myCurrentFrequency;
      return;
    }

    // Store the user's requested frequency
    myRequestedFrequency = std::min(static_cast<float>(aSpeedHz), myMaxFrequency);
    

  }

  uint32_t GetCurrentPeriod() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return myConverter.ToPeriod(myCurrentFrequency);
    }
  }

  float GetCurrentFrequency() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return myCurrentFrequency;
    }
  }

  float GetTargetFrequency() const {
    if (myState == StepperState::STOPPED) {
      return 0;
    } else {
      return myTargetFrequency;
    }
  }
  
  float GetRequestedFrequency() const {
    return myRequestedFrequency;
  }
  
  StepperState GetState() { return myState; }

protected:
  Converter myConverter;

private:
  bool Step(StepperState aState) {
    switch (aState) {
    case StepperState::STARTING: {
      static_cast<Derived *>(this)->PutStep(myCurrentFrequency);
      return true;
    } break;

    case StepperState::ACCELERATING: {
      float nextFrequency = myConverter.CalculateNextFrequency(
          myCurrentFrequency, myAcceleration);

      if (nextFrequency >= myMaxFrequency) {
        myCurrentFrequency = myMaxFrequency;
        myTargetFrequency = myMaxFrequency;
      } else if (nextFrequency >= myTargetFrequency) {
        myCurrentFrequency = myTargetFrequency;
      } else {
        myCurrentFrequency = nextFrequency;
      }
      static_cast<Derived *>(this)->PutStep(myCurrentFrequency);
      return true;
    } break;

    case StepperState::DECELERATING: {
      float nextFrequency = myConverter.CalculateNextFrequency(
          myCurrentFrequency, -myDeceleration);
      myCurrentFrequency = nextFrequency;
      if (nextFrequency <= myTargetFrequency) {
        myCurrentFrequency = myTargetFrequency;
      }

      if (myCurrentFrequency <= myMinFrequency) {
        myCurrentFrequency = myMinFrequency;
      }

      static_cast<Derived *>(this)->PutStep(myCurrentFrequency);
      return true;
    }

    break;
    case StepperState::COASTING: {
      static_cast<Derived *>(this)->PutStep(myCurrentFrequency);
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
  float myMaxFrequency;
  float myMinFrequency;
  float myCurrentFrequency;
  float myTargetFrequency;
  float myRequestedFrequency;  // Tracks user's requested frequency separately

  // 1-byte members
  StepperState myState;
  bool myIsRunning;
};

} // namespace PIOStepperSpeedController