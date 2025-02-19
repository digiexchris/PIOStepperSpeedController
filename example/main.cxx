/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tusb.h"
#include <cstdlib>
#include <pico/sem.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <random>
#include <stdio.h>

#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <PIOStepperSpeedController.hxx>
#include <format>
#include <iostream>

using namespace PIOStepperSpeedController;

void aStoppedCallback(Stepper::CallbackEvent event);
void aCoastingCallback(Stepper::CallbackEvent event);
void aAcceleratingCallback(Stepper::CallbackEvent event);
void aDeceleratingCallback(Stepper::CallbackEvent event);

Stepper *stepper = nullptr;
static semaphore_t stepperSemaphore;

const uint stepPin = 2;

// due to a divide by zero issue, startSpeedHz must NOT be 0. Consider this
// the speed at which the motor is capable of moving at from a dead stop
// without needing acceleration 1 to 10 steps per second is a good starting
// point, feel free to tune it from there if it takes too long to start
// accelerating 10hz, or 10 steps per second
const uint startSpeedHz = 10;      // steps per second
const uint stepsPerRotation = 200; // steps per rotation of the motor
const uint maxSpeed = 10000;       // steps per second
const uint acceleration = 1000;    // steps per second squared
const uint deceleration = 2000;    // steps per second squared

struct Sequence {
  uint32_t nextPhase = 0;
  uint32_t targetSpeed[2] = {0, 0};
};

Sequence *sequence = new Sequence();

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<uint32_t> dist(startSpeedHz, maxSpeed);

int main() {

  stdio_init_all();

  // All callbacks are optional, and are only used here to demonstrate a few
  // possibilities
  stepper = new Stepper(stepPin, startSpeedHz, maxSpeed, stepsPerRotation,
                        acceleration, deceleration, aStoppedCallback,
                        aCoastingCallback, aAcceleratingCallback,
                        aDeceleratingCallback);

  // At this point the pio program is initialized and ready, but in a stopped
  // state. It will automatically start the PIO SM when start() is called.

  // Now, do something interesting with the motor, like set it to a random
  // speed between 0 and the max, and wait till it reaches that speed

  sleep_ms(500);
  printf("Starting!\n");

  Stepper::StepperState state;

  bool aStepOccurred = false;

  float speed = 0;

  int count = 0;

  // quick! you! give me a random number between 10 and 10000!
  // the stepper will not exceed the configured max speed, so
  // it will coast at the lower of either the set target speed,
  // or the max speed. So, I'll just min() that so that the
  // example continues to run

  sequence->targetSpeed[0] = 5000;
  sequence->targetSpeed[1] = 7000;

  stepper->SetTargetHz(sequence->targetSpeed[1000]);
  sequence->nextPhase = 0;
  stepper->Start();
  while (true) {

    count++;
    if (count >= 100) {
      count = 0;
      // Get some info from the stepper and command it to do things
      speed = stepper->GetCurrentFrequency();
      state = stepper->GetState();
      switch (state) {
      case Stepper::StepperState::ACCELERATING:
        printf("Stepper is accelerating\n");
        break;
      case Stepper::StepperState::COASTING:
        printf("Stepper is coasting\n");
        break;
      case Stepper::StepperState::DECELERATING:
        printf("Stepper is decelerating\n");
        break;
      case Stepper::StepperState::STARTING:
        printf("Stepper is starting\n");
        break;
      case Stepper::StepperState::STOPPING:
        printf("Stepper is stopping\n");
        break;
      case Stepper::StepperState::STOPPED:
        printf("Stepper is stopped\n");
        break;
      }

      auto currentHz = stepper->GetCurrentFrequency();
      auto targetHz = stepper->GetTargetFrequency();
      std::cout << "Current Hz: " << currentHz << " Target Hz: " << targetHz
                << "\n";
    }

    // auto msg = std::format("Current Hz: {:.2f} Target Hz: {:.2f}\n",
    // currentHz,
    //                        targetHz);
    // printf(msg);

    // maintain the internal state of the stepper. Call this as fast as possible
    // and this will block until the step is complete.
    // if it returns false, the stepper is stopped and would not have executed
    // a step. For best timing, your code should be done and waiting for the
    // step to occur before the previous step has been completed (using
    // multithreading techniques, or similar)
    aStepOccurred = stepper->Update();

    tight_loop_contents();
    sleep_ms(10); // just doing this so tinyusb can maintain the serial
                  // output, normally I use freertos tasks and follow the
                  // above note about timing
  }
}

// If you take too long in any of these callbacks, it will delay the stepper
// update loop I'm using that to stop and print when it's coasting and stopped,
// but with a real stepper that will likely result in missed or extra steps
// since acceleration and deceleration is not correctly being maintained
// according to the step profile if there is any delay. I recommend treating
// these callbacks as if they were interrupt handlers, so that if the stepper is
// moving quickly, no steps are lost and the movement remains smooth.
// In other words, do not sleep in the callback like I am! This is just to see
// the output.
void aStoppedCallback(Stepper::CallbackEvent event) {
  printf("Stopped Callback called\n");
  __breakpoint();
  sequence->nextPhase = 1;
  sequence->targetSpeed[0] = dist(gen);
  sequence->targetSpeed[1] = dist(gen);
  stepper->SetTargetHz(sequence->targetSpeed[0]);
  stepper->Start();
  sleep_ms(500);
}
void aCoastingCallback(Stepper::CallbackEvent event) {
  printf("Coasting Callback called\n");
  //__breakpoint();
  switch (sequence->nextPhase) {
  case 0:
    stepper->SetTargetHz(sequence->targetSpeed[0]);
    sequence->nextPhase = 1;
    break;
  case 1:
    stepper->SetTargetHz(sequence->targetSpeed[1]);
    sequence->nextPhase = 2;
    break;
  case 2:
    stepper->Stop();
    break;
  }
  sleep_ms(500);
}

void aAcceleratingCallback(Stepper::CallbackEvent event) {
  printf("Accelerating Callback called\n");
  //__breakpoint();
  sleep_ms(500);
}

void aDeceleratingCallback(Stepper::CallbackEvent event) {
  printf("Decelerating Callback Called\n");
  //__breakpoint();
  sleep_ms(500);
}