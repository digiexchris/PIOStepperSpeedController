// #include "Stepper.hxx"
// #include "Converter.hxx"
// #include <cassert>

// namespace PIOStepperSpeedController {

// template <typename Derived>
// Stepper<Derived>::Stepper(uint32_t aAcceleration, uint32_t aDeceleration,
//                           uint32_t aSysClk, uint32_t aPrescaler,
//                           Callback aStoppedCallback, Callback
//                           aCoastingCallback, Callback aAcceleratingCallback,
//                           Callback aDeceleratingCallback)
//     : myAcceleration(aAcceleration), myDeceleration(aDeceleration),
//       mySysClk(aSysClk), myPrescaler(aPrescaler),
//       myStoppedCallback(aStoppedCallback),
//       myCoastingCallback(aCoastingCallback),
//       myAcceleratingCallback(aAcceleratingCallback),
//       myDeceleratingCallback(aDeceleratingCallback),
//       myState(StepperState::STOPPED) {
//   myConverter = Converter(mySysClk, myPrescaler);
//   myMinPeriod = 1;
//   myMaxPeriod = UINT32_MAX - 1;
// }

// template <typename Derived>
// void Stepper<Derived>::TransitionTo(StepperState aState) {
//   switch (aState) {
//   case StepperState::ACCELERATING:
//     if (myState != StepperState::ACCELERATING) {
//       myState = StepperState::ACCELERATING;
//       if (myAcceleratingCallback != nullptr) {
//         myAcceleratingCallback(CallbackEvent::ACCELERATING);
//       }
//     }
//     break;

//   case StepperState::COASTING:
//     if (myState != StepperState::COASTING) {
//       myState = StepperState::COASTING;
//       if (myCoastingCallback != nullptr) {
//         myCoastingCallback(CallbackEvent::COASTING);
//       }
//     }
//     break;

//   case StepperState::DECELERATING:
//     if (myState != StepperState::DECELERATING) {
//       myState = StepperState::DECELERATING;
//       if (myDeceleratingCallback != nullptr) {
//         myDeceleratingCallback(CallbackEvent::DECELERATING);
//       }
//     }
//     break;

//   case StepperState::STARTING:
//     if (myState != StepperState::STARTING) {
//       myState = StepperState::STARTING;
//     }
//     break;

//   case StepperState::STOPPING:
//     if (myState != StepperState::STOPPING) {
//       myState = StepperState::STOPPING;
//     }

//     break;

//   case StepperState::STOPPED:
//     if (myState != StepperState::STOPPED) {
//       myState = StepperState::STOPPED;
//       if (myStoppedCallback != nullptr) {
//         myStoppedCallback(CallbackEvent::STOPPED);
//       }
//     }
//     break;
//   }
// }

// template <typename Derived> bool Stepper<Derived>::Step(StepperState aState)
// {
//   switch (aState) {
//   case StepperState::STARTING: {
//     static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
//     return true;
//   } break;

//   case StepperState::ACCELERATING: {
//     int32_t nextPeriod = myConverter.ToPeriod(
//         myConverter.CalculateNextFrequency(myCurrentPeriod, myAcceleration));

//     if (nextPeriod <= myMinPeriod) {
//       myCurrentPeriod = myMinPeriod;
//       myTargetPeriod = myMinPeriod;
//     } else if (nextPeriod <= myTargetPeriod) {
//       myCurrentPeriod = myTargetPeriod;
//     } else {
//       myCurrentPeriod = nextPeriod;
//     }
//     static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
//     return true;
//   } break;

//   case StepperState::DECELERATING: {
//     int32_t nextPeriod = myConverter.ToPeriod(
//         myConverter.CalculateNextFrequency(myCurrentPeriod,
//         -myDeceleration));
//     myCurrentPeriod = nextPeriod;
//     if (nextPeriod >= myTargetPeriod) {
//       myCurrentPeriod = myTargetPeriod;
//     }

//     if (myCurrentPeriod >= myStartPeriod) {
//       myCurrentPeriod = myStartPeriod;
//     }

//     static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
//     return true;
//   }

//   break;
//   case StepperState::COASTING: {
//     static_cast<Derived *>(this)->PutStep(myCurrentPeriod);
//     return true;
//   } break;
//   default:
//     assert(false);
//   }
//   return false;
// }

// template <typename Derived> void Stepper<Derived>::Stop() {
//   if (myState == StepperState::STOPPED || myState == StepperState::STOPPING)
//   {
//     return;
//   }

//   myState = StepperState::STOPPING;
// }

// template <typename Derived> void Stepper<Derived>::Start() {

//   if (!myIsRunning) {
//     myCurrentPeriod = myStartPeriod;
//     static_cast<Derived *>(this)->EnableImpl();
//     TransitionTo(StepperState::STARTING);
//     myIsRunning = true;
//   }
// }

// template <typename Derived> bool Stepper<Derived>::Update() {
//   if (!myIsRunning) {
//     return false;
//   }

//   switch (myState) {
//   case StepperState::STOPPED:
//     return false;
//     break;
//   case StepperState::STOPPING:
//     if (myCurrentPeriod >= myStartPeriod) {
//       myIsRunning = false;
//       static_cast<Derived *>(this)->DisableImpl();
//       TransitionTo(StepperState::STOPPED);
//       return false;
//     } else {
//       Step(StepperState::DECELERATING);
//     }
//     break;
//   case StepperState::STARTING:
//     if (myCurrentPeriod < myStartPeriod) {
//       TransitionTo(StepperState::ACCELERATING);
//       Step(StepperState::ACCELERATING);
//     } else if (myCurrentPeriod <= myTargetPeriod) {
//       TransitionTo(StepperState::COASTING);
//       Step(StepperState::COASTING);
//     } else {
//       TransitionTo(StepperState::ACCELERATING);
//       Step(StepperState::STARTING);
//     }
//   case StepperState::ACCELERATING:
//     if (myCurrentPeriod > myTargetPeriod) {
//       Step(StepperState::ACCELERATING);
//     } else {
//       TransitionTo(StepperState::COASTING);
//       Step(StepperState::COASTING);
//     }
//     break;
//   case StepperState::COASTING:
//     if (myCurrentPeriod == myTargetPeriod) {
//       Step(StepperState::COASTING);
//     } else {
//       // only thing that can result in a speed change after coasting is a
//       // SetTargetHz or Stop
//       return false;
//     }
//     break;
//   case StepperState::DECELERATING:
//     if (myCurrentPeriod < myTargetPeriod) {
//       Step(StepperState::DECELERATING);
//     } else {
//       TransitionTo(StepperState::COASTING);
//       Step(StepperState::COASTING);
//     }
//     break;
//   }

//   return false;
// }

// template <typename Derived>
// void Stepper<Derived>::SetTargetHz(uint32_t aSpeedHz)

//     template <typename Derived>
//     uint32_t Stepper<Derived>::GetCurrentPeriod() const {
//   if (myState == StepperState::STOPPED) {
//     return 0;
//   } else {
//     return myCurrentPeriod;
//   }
// }

// template <typename Derived>
// float Stepper<Derived>::GetCurrentFrequency() const {
//   if (myState == StepperState::STOPPED) {
//     return 0;
//   } else {
//     return myConverter.ToFrequency(myCurrentPeriod);
//   }
// }

// template <typename Derived> float Stepper<Derived>::GetTargetFrequency()
// const {
//   if (myState == StepperState::STOPPED) {
//     return 0;
//   } else {
//     return myConverter.ToFrequency(myTargetPeriod);
//   }
// }

// template <typename Derived> StepperState Stepper<Derived>::GetState() {
//   return myState;
// }

// } // namespace PIOStepperSpeedController