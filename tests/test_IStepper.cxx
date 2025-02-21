#include "../Stepper.hxx"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace PIOStepperSpeedController {

class MockStepper : public Stepper {
public:
  using Stepper::Stepper;

  MOCK_METHOD(bool, PutStep, (uint32_t aPeriod), ());
  MOCK_METHOD(void, EnableImpl, (), ());
  MOCK_METHOD(void, DisableImpl, (), ());
};

class TestStepper : public ::testing::Test {
protected:
  void SetUp() override {
    stepper = std::make_unique<::testing::NiceMock<MockStepper>>(1000, 1000);

    ON_CALL(*stepper, PutStep(::testing::_))
        .WillByDefault(::testing::Return(true));
  }

  static constexpr uint32_t MAX_ITERATIONS = 1000; // Timeout threshold
  std::unique_ptr<MockStepper> stepper;
};

TEST_F(TestStepper, InitialState) {
  EXPECT_EQ(stepper->GetState(), Stepper::StepperState::STOPPED);
  EXPECT_EQ(stepper->GetCurrentFrequency(), 0.0f);
  EXPECT_EQ(stepper->GetTargetFrequency(), 0.0f);
}

TEST_F(TestStepper, StartStop) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, DisableImpl()).Times(1);

  stepper->Start();
  EXPECT_NE(stepper->GetState(), Stepper::StepperState::STOPPED);

  stepper->Stop();
  stepper->Update(); // Need to update to process stopping state
  EXPECT_EQ(stepper->GetState(), Stepper::StepperState::STOPPED);
}

TEST_F(TestStepper, SetTargetHz) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000); // Set target to 1000Hz

  // Should start accelerating
  EXPECT_EQ(stepper->GetState(), Stepper::StepperState::ACCELERATING);

  uint32_t iterations = 0;
  // Run updates until reaching target
  while (stepper->Update() &&
         stepper->GetState() != Stepper::StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving coasting speed";
  EXPECT_EQ(stepper->GetState(), Stepper::StepperState::COASTING);
  EXPECT_NEAR(stepper->GetCurrentFrequency(), 1000.0f, 0.1f);
}

TEST_F(TestStepper, Acceleration) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000);

  float startFreq = stepper->GetCurrentFrequency();
  stepper->Update();

  // After one update, frequency should increase by acceleration/update_rate
  // With 1000Hz^2 acceleration, after 1ms (1kHz update rate),
  // we expect frequency to increase by 1Hz
  float expectedFreq = startFreq + 1.0f; // 1000Hz^2 * (1/1000)s = 1Hz increase
  EXPECT_NEAR(stepper->GetCurrentFrequency(), expectedFreq, 0.1f);
}

TEST_F(TestStepper, Deceleration) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching target
  while (stepper->Update() &&
         stepper->GetState() != Stepper::StepperState::COASTING &&
         iterations++ < MAX_ITERATIONS) {
  }

  EXPECT_LT(iterations, MAX_ITERATIONS)
      << "Timeout reached before achieving initial coasting speed";

  // Now set a lower target
  stepper->SetTargetHz(500);
  EXPECT_EQ(stepper->GetState(), Stepper::StepperState::DECELERATING);

  float startFreq = stepper->GetCurrentFrequency();
  stepper->Update();

  // After one update, frequency should decrease by deceleration/update_rate
  // With 1000Hz^2 deceleration, after 1ms (1kHz update rate),
  // we expect frequency to decrease by 1Hz
  float expectedFreq = startFreq - 1.0f; // 1000Hz^2 * (1/1000)s = 1Hz decrease
  EXPECT_NEAR(stepper->GetCurrentFrequency(), expectedFreq, 0.1f);
}

TEST_F(TestStepper, Callbacks) {
  bool stoppedCalled = false;
  bool acceleratingCalled = false;
  bool coastingCalled = false;
  bool deceleratingCalled = false;

  auto stepper = std::make_unique<::testing::NiceMock<MockStepper>>(
      1000, 1000, 125000000, 1,
      [&](Stepper::CallbackEvent) { stoppedCalled = true; },
      [&](Stepper::CallbackEvent) { coastingCalled = true; },
      [&](Stepper::CallbackEvent) { acceleratingCalled = true; },
      [&](Stepper::CallbackEvent) { deceleratingCalled = true; });

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run through a complete cycle
  while (stepper->Update() &&
         stepper->GetState() != Stepper::StepperState::COASTING &&
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

  EXPECT_TRUE(acceleratingCalled);
  EXPECT_TRUE(coastingCalled);
  EXPECT_TRUE(stoppedCalled);
}

TEST_F(TestStepper, GetCurrentPeriod) {
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
  EXPECT_EQ(stepper->GetCurrentPeriod(), 125000);
}

TEST_F(TestStepper, FrequencyConversion) {
  EXPECT_CALL(*stepper, EnableImpl()).Times(1);
  EXPECT_CALL(*stepper, PutStep(::testing::_))
      .Times(::testing::AtLeast(1))
      .WillRepeatedly(::testing::Return(true));

  stepper->Start();
  stepper->SetTargetHz(1000);

  uint32_t iterations = 0;
  // Run until reaching target
  while (stepper->Update() &&
         stepper->GetState() != Stepper::StepperState::COASTING &&
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
