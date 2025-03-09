#include <PIOStepperSpeedController/Stepper.hxx>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <memory>

namespace PIOStepperSpeedController {

class MockCallbackProvider {
public:
  MOCK_METHOD(void, acceleratingCallback, (CallbackEvent));
  MOCK_METHOD(void, coastingCallback, (CallbackEvent));
  MOCK_METHOD(void, stoppedCallback, (CallbackEvent));
  MOCK_METHOD(void, deceleratingCallback, (CallbackEvent));
};

class MockStepper : public Stepper<MockStepper> {
public:
  using Stepper::Stepper;

  MOCK_METHOD(bool, PutStep, (float aFrequency), ());
  MOCK_METHOD(void, EnableImpl, (), ());
  MOCK_METHOD(void, DisableImpl, (), ());
};

class StepperTest : public ::testing::Test {
protected:
  void SetUp() override {
    stepper = std::make_unique<MockStepper>(1, 10000000, 1000, 2000);
    callbackProvider = std::make_unique<MockCallbackProvider>();

    ON_CALL(*stepper, PutStep(::testing::_))
        .WillByDefault(::testing::Return(true));
  }

  static constexpr uint32_t MAX_ITERATIONS = 100000; // Timeout threshold
  std::unique_ptr<MockStepper> stepper;
  std::unique_ptr<MockCallbackProvider> callbackProvider;
};

TEST_F(StepperTest, InitialState) {
  EXPECT_EQ(stepper->GetState(), StepperState::STOPPED);
  EXPECT_EQ(stepper->GetCurrentFrequency(), 0.0f);
  EXPECT_EQ(stepper->GetTargetFrequency(), 0.0f);
  EXPECT_EQ(stepper->GetRequestedFrequency(), 1.0f); // Default to minSpeed
}

TEST_F(StepperTest, StartStop) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, DisableImpl()).Times(1);

  stepper->Start();
  EXPECT_NE(stepper->GetState(), StepperState::STOPPED);

  stepper->Stop();
  stepper->Update(); // Need to update to process stopping state
  EXPECT_EQ(stepper->GetState(), StepperState::STOPPED);
}

TEST_F(StepperTest, StartThenSetTargetHzThenCoast) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(5000); // Set target to 5000Hz

  // Should start accelerating
  EXPECT_EQ(stepper->GetState(), StepperState::STARTING);

  EXPECT_NEAR(stepper->GetTargetFrequency(), 1.0f, 0.1f);
  EXPECT_NEAR(stepper->GetRequestedFrequency(), 5000.0f, 0.1f);

  float freq = 0.0f;

  uint32_t iterations = 0;
  uint32_t coastingIterations = 0;
  // Run updates until reaching target
  while (stepper->GetState() != StepperState::COASTING &&
         iterations < MAX_ITERATIONS * 100) {
    iterations++;
    auto previousFreq = stepper->GetCurrentFrequency();
    stepper->Update();
    auto newfreq = stepper->GetCurrentFrequency();
    if (newfreq == previousFreq) {
      if (stepper->GetState() != StepperState::COASTING) {
        coastingIterations++;
      }
      if (coastingIterations > 5) {
        FAIL() << "Stepper is not coasting" << "Frequency: " << freq
               << " Coasting Iterations: " << coastingIterations;
      }
    }
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving coasting speed. Current frequency:"

      << stepper->GetCurrentFrequency() << " Iterations: " << iterations;

  EXPECT_EQ(stepper->GetState(), StepperState::COASTING)
      << "Current frequency: " << stepper->GetCurrentFrequency()
      << "Iterations: " << iterations;
  EXPECT_NEAR(stepper->GetCurrentFrequency(), 5000.0f, 0.1f)
      << "Current frequency: " << stepper->GetCurrentFrequency()
      << "Iterations: " << iterations;
  ;
}

TEST_F(StepperTest, Acceleration) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  float startFreq = stepper->GetCurrentFrequency();
  stepper->SetTargetHz(1000);

  stepper->Update();
  EXPECT_NEAR(stepper->GetCurrentFrequency(), 1000.0, 1.0f);
}

TEST_F(StepperTest, Deceleration) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching target
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving initial coasting speed";

  // Now set a lower target
  stepper->SetTargetHz(500);

  stepper->Update();
  EXPECT_EQ(stepper->GetState(), StepperState::DECELERATING);
  // After one update, frequency should decrease by deceleration/update_rate
  // With 1000Hz^2 deceleration, after 1ms (1kHz update rate),
  // we expect frequency to decrease by 1Hz
  float expectedFreq = 998.0f; // 1000Hz^2 * (1/1000)s = 1Hz decrease
  EXPECT_NEAR(stepper->GetCurrentFrequency(), expectedFreq, 0.1f);
}

TEST_F(StepperTest, Callbacks) {
  EXPECT_CALL(*callbackProvider,
              acceleratingCallback(CallbackEvent::ACCELERATING))
      .Times(1);
  EXPECT_CALL(*callbackProvider, coastingCallback(CallbackEvent::COASTING))
      .Times(1);
  EXPECT_CALL(*callbackProvider, stoppedCallback(CallbackEvent::STOPPED))
      .Times(1);
  EXPECT_CALL(*callbackProvider,
              deceleratingCallback(CallbackEvent::DECELERATING))
      .Times(0);

  // Store callback provider pointer statically to use in callbacks
  static MockCallbackProvider *staticProvider = callbackProvider.get();

  // Use static functions that can be converted to function pointers
  static auto acceleratingCb = [](CallbackEvent e) {
    staticProvider->acceleratingCallback(e);
  };
  static auto coastingCb = [](CallbackEvent e) {
    staticProvider->coastingCallback(e);
  };
  static auto stoppedCb = [](CallbackEvent e) {
    staticProvider->stoppedCallback(e);
  };
  static auto deceleratingCb = [](CallbackEvent e) {
    staticProvider->deceleratingCallback(e);
  };

  auto stepper = std::make_unique<MockStepper>(
      1, 100000000, 100, 100, 125000000, 1, stoppedCb, coastingCb,
      acceleratingCb, deceleratingCb);
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  ON_CALL(*stepper, PutStep(::testing::_))
      .WillByDefault(::testing::Return(true));

  EXPECT_CALL(*stepper, DisableImpl()).Times(1);

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run through a complete cycle
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving coasting speed";

  stepper->Stop();
  iterations = 0;
  while (stepper->Update() && iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached during stopping sequence";
}

TEST_F(StepperTest, GetCurrentPeriod) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  EXPECT_EQ(stepper->GetCurrentPeriod(), 0); // When stopped

  stepper->Start();
  stepper->Update(); // First update after start

  // With 125MHz clock and 1kHz start frequency, expect period of 125000
  // period = sysclock / (prescaler * frequency)
  // 125000 = 125000000 / (1 * 1000)
  EXPECT_EQ(stepper->GetCurrentPeriod(), 125000000);
}

TEST_F(StepperTest, FrequencyConversion) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching target
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving coasting speed";

  EXPECT_NEAR(stepper->GetCurrentFrequency(), stepper->GetTargetFrequency(),
              0.1f);
}

TEST_F(StepperTest, StartCoastStopSetSpeedContinuesToStop) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, DisableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching target
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving coasting speed";

  stepper->Stop();
  const uint32_t NEW_TARGET_SPEED = 500;
  stepper->SetTargetHz(NEW_TARGET_SPEED); // Set new target while stopping

  // Verify requested speed is stored but target remains at min
  EXPECT_NEAR(stepper->GetRequestedFrequency(), NEW_TARGET_SPEED, 0.1f);
  EXPECT_NEAR(stepper->GetTargetFrequency(), 1.0f, 0.1f); // Min speed

  iterations = 0;
  while (stepper->Update() && iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached during stopping sequence";

  EXPECT_EQ(stepper->GetState(), StepperState::STOPPED);
  EXPECT_EQ(stepper->GetCurrentFrequency(), 0.0f);
}

TEST_F(StepperTest, StartCoastStopThenStartSetSpeed) {
  EXPECT_CALL(*stepper, EnableImpl())
      .Times(1); // Once for initial start, but since it didn't fully stop, it
                 // shouldn't call it on the second Start()
  EXPECT_CALL(*stepper, DisableImpl())
      .Times(0); // Called when stopping, but never actually stops in this test
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  // Start and reach coasting
  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching coasting at initial target speed
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving initial coasting speed";
  EXPECT_EQ(stepper->GetState(), StepperState::COASTING);

  // Initiate stopping
  stepper->Stop();

  // Let it start to decelerate but don't let it fully stop
  iterations = 0;
  const uint32_t PARTIAL_STOP_ITERATIONS = 10;
  while (stepper->Update() && iterations++ < PARTIAL_STOP_ITERATIONS) {
  }

  EXPECT_EQ(stepper->GetState(), StepperState::STOPPING);
  EXPECT_GT(stepper->GetCurrentFrequency(), 0.0f);

  // Now restart and set new target speed
  stepper->Start();
  const uint32_t NEW_TARGET_SPEED = 1500;
  stepper->SetTargetHz(NEW_TARGET_SPEED);

  EXPECT_EQ(stepper->GetState(), StepperState::STARTING);
  EXPECT_NEAR(stepper->GetTargetFrequency(), 1.0f, 0.1f);
  EXPECT_NEAR(stepper->GetRequestedFrequency(), NEW_TARGET_SPEED, 0.1f);

  // Run until reaching coasting at new target
  iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving new coasting speed";

  // Verify we're coasting at the new target speed
  EXPECT_EQ(stepper->GetState(), StepperState::COASTING);
  EXPECT_NEAR(stepper->GetCurrentFrequency(), NEW_TARGET_SPEED, 1.0f);
}

// Test cases specifically for the requested/target speed separation

TEST_F(StepperTest, SetTargetHzStoresDuringStop) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);

  stepper->Start();
  const uint32_t INITIAL_SPEED = 1000;
  stepper->SetTargetHz(INITIAL_SPEED);

  // Verify requested speed is stored
  EXPECT_NEAR(stepper->GetRequestedFrequency(), INITIAL_SPEED, 0.1f);

  stepper->Stop();

  // Set new target while stopping
  const uint32_t NEW_SPEED = 2000;
  stepper->SetTargetHz(NEW_SPEED);

  // Verify the new requested speed is stored
  EXPECT_NEAR(stepper->GetRequestedFrequency(), NEW_SPEED, 0.1f);

  // But the target speed should remain at min frequency for stopping
  EXPECT_NEAR(stepper->GetTargetFrequency(), 1.0f, 0.1f);
}

TEST_F(StepperTest, StartUsesRequestedFrequency) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(2);
  EXPECT_CALL(*stepper, DisableImpl()).Times(1);

  // Start and set a target speed
  stepper->Start();
  const uint32_t REQUESTED_SPEED = 2000;
  stepper->SetTargetHz(REQUESTED_SPEED);

  // Verify the requested speed is stored
  EXPECT_NEAR(stepper->GetRequestedFrequency(), REQUESTED_SPEED, 0.1f);

  // Run to coasting
  uint32_t iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }
  EXPECT_LT(iterations, MAX_ITERATIONS);

  // Stop fully
  stepper->Stop();
  iterations = 0;
  while (stepper->Update() && iterations++ < MAX_ITERATIONS) {
  }
  EXPECT_EQ(stepper->GetState(), StepperState::STOPPED);

  // Start again without setting a new target speed
  stepper->Start();

  // Run until coasting
  iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  // Should accelerate back to previous requested speed
  EXPECT_EQ(stepper->GetState(), StepperState::COASTING);
  EXPECT_NEAR(stepper->GetCurrentFrequency(), REQUESTED_SPEED, 1.0f);
  EXPECT_NEAR(stepper->GetTargetFrequency(), REQUESTED_SPEED, 0.1f);
}

TEST_F(StepperTest, StopWhileDeceleratingThenStartAgain) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, DisableImpl()).Times(0);

  // Start and set a target speed
  stepper->Start();
  const uint32_t INITIAL_SPEED = 1000;
  stepper->SetTargetHz(INITIAL_SPEED);

  // Run until coasting
  uint32_t iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }
  EXPECT_LT(iterations, MAX_ITERATIONS);

  // Stop
  stepper->Stop();

  // Let it decelerate for a bit, but don't go all the way to stopped
  iterations = 0;
  const uint32_t PARTIAL_STOP_ITERATIONS = 5;
  while (stepper->Update() && iterations++ < PARTIAL_STOP_ITERATIONS) {
  }
  EXPECT_EQ(stepper->GetState(), StepperState::STOPPING);

  // Start again - should accelerate back to previously requested speed
  stepper->Start();
  iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_EQ(stepper->GetState(), StepperState::COASTING);
  EXPECT_NEAR(stepper->GetCurrentFrequency(), INITIAL_SPEED, 1.0f);
}

TEST_F(StepperTest, StartAtMinSpeedThenAccelerateToTarget) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);

  // Start without setting a target first
  stepper->Start();

  // Should start at min speed
  EXPECT_NEAR(stepper->GetTargetFrequency(), 1.0f, 0.1f);

  // Run to coasting at min speed
  uint32_t iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  // Set a higher target speed while coasting
  const uint32_t NEW_SPEED = 3000;
  stepper->SetTargetHz(NEW_SPEED);

  // Should transition to accelerating
  stepper->Update();
  EXPECT_EQ(stepper->GetState(), StepperState::ACCELERATING);

  // Run until coasting at new speed
  iterations = 0;
  while (stepper->Update() && stepper->GetState() != StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  // Should now be coasting at the new target speed
  EXPECT_EQ(stepper->GetState(), StepperState::COASTING);
  EXPECT_NEAR(stepper->GetCurrentFrequency(), NEW_SPEED, 1.0f);
}

} // namespace PIOStepperSpeedController

// int main(int argc, char **argv) {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
