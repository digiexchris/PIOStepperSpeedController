#include "../Stepper.hxx"
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
  stepper->SetTargetHz(5000); // Set target to 1000Hz

  // Should start accelerating
  EXPECT_EQ(stepper->GetState(), StepperState::STARTING);

  EXPECT_NEAR(stepper->GetTargetFrequency(), 5000.0f, 0.1f);

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
        FAIL() << "Stepper is not coasting"
               << "Frequency: " << freq
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
  EXPECT_EQ(stepper->GetState(), StepperState::DECELERATING);

  stepper->Update();

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

} // namespace PIOStepperSpeedController

// int main(int argc, char **argv) {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }
