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

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <PIOStepperSpeedController/PIOStepper.hxx>
#include <format>
#include <iostream>

using namespace PIOStepperSpeedController;

void aStoppedCallback(CallbackEvent event);
void aCoastingCallback(CallbackEvent event);
void aAcceleratingCallback(CallbackEvent event);
void aDeceleratingCallback(CallbackEvent event);

PIOStepper *stepper = nullptr;
static semaphore_t stepperSemaphore;

const uint stepPin = 6;

// due to a divide by zero issue, minSpeed must NOT be 0. Consider this
// the speed at which the motor is capable of moving at from a dead stop
// without needing acceleration. 1 to 10 steps per second is a good starting
// point, feel free to tune it from there if it takes too long to start
// accelerating. Units are in hz, so 10hz, or 10 steps per second
// Note that very small minimum frequences end up being the time the first step
// will take. So a 0.1hz starting frequency will take 10 seconds to complete the
// first step before acceleration begins. 4 steps per second is 1/4 of a second
// for the first step.
const float minSpeed = 10;      // steps per second
const float maxSpeed = 10000;   // steps per second
const uint acceleration = 1000; // steps per second squared
const uint deceleration = 2000; // steps per second squared
// the sysclk is divided by this number, which reduces the resolution of
// the stepper speed, but increases the maximum possible stepper speed.
// It also reduces how much CPU it takes to maintain the PIO FIFO. It
// scales 1/UINT32-1 as a maximum period (minumum frequency), and
// sysclk/prescaler as the maximum speed. For an averate rp2040 with a
// 125mhz sysclk, this means with a prescaler of 1250, the maximum speed
// is 100,000 steps per second.
const uint prescaler = 125;

struct Sequence {
  uint32_t nextPhase = 0;
  uint32_t targetSpeed[2] = {0, 0};
};

Sequence *sequence = new Sequence();

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<uint32_t> dist(minSpeed, maxSpeed);

int main() {

  stdio_init_all();
  uint32_t sysclk = clock_get_hz(clk_sys);

  // All callbacks are optional, and are only used here to demonstrate a few
  // possibilities
  stepper =
      new PIOStepper(stepPin, minSpeed, maxSpeed, acceleration, deceleration,
                     sysclk, prescaler, aStoppedCallback, aCoastingCallback,
                     aAcceleratingCallback, aDeceleratingCallback);

  // At this point the pio program is initialized and ready, but in a stopped
  // state. It will automatically start the PIO SM when start() is called.

  // Now, do something interesting with the motor, like set it to a random
  // speed between 0 and the max, and wait till it reaches that speed

  sleep_ms(250);
  printf("Starting!\n");
  stdio_flush();
  StepperState state;

  bool aStepOccurred = false;

  float speed = 0;

  int count = 0;

  // quick! you! give me a random number between 10 and 10000!
  // the stepper will not exceed the configured max speed, so
  // it will coast at the lower of either the set target speed,
  // or the max speed. So, I'll just min() that so that the
  // example continues to run

  sequence->targetSpeed[0] = dist(gen);
  sequence->targetSpeed[1] = dist(gen);

  stepper->SetTargetHz(sequence->targetSpeed[1000]);
  sequence->nextPhase = 0;
  stepper->Start();
  while (true) {

    count++;
    if (count >= 10000) {
      count = 0;
      // Get some info from the stepper and command it to do things
      speed = stepper->GetCurrentFrequency();
      state = stepper->GetState();
      switch (state) {
      case StepperState::ACCELERATING:
        printf("Stepper is accelerating\n");
        break;
      case StepperState::COASTING:
        printf("Stepper is coasting\n");
        break;
      case StepperState::DECELERATING:
        printf("Stepper is decelerating\n");
        break;
      case StepperState::STARTING:
        printf("Stepper is starting\n");
        break;
      case StepperState::STOPPING:
        printf("Stepper is stopping\n");
        break;
      case StepperState::STOPPED:
        printf("Stepper is stopped\n");
        break;
      }

      auto currentHz = stepper->GetCurrentFrequency();
      auto targetHz = stepper->GetTargetFrequency();
      std::cout << "Current Hz: " << currentHz << " Target Hz: " << targetHz
                << "\n";

      stdio_flush();
    }

    // maintain the internal state of the stepper. Call this as fast as
    // possible. it will block until the step is complete, so it is often useful
    // to call this in an RTOS task, or it's own thread, or a timer callback.
    // For best timing, your code should be done and waiting for this function
    // to complete and start again
    stepper->Update();

    tight_loop_contents();
    // sleep_us(10); // just doing this as an example of what not to do.
    // normally I use freertos tasks and follow the
    //  above note about timing
    //  for example, I was originally sleeping for 10ms, which
    //  ended up making the PIO program run at a maximum of 100hz!
  }
}

// If you take too long in any of these callbacks, it will delay the stepper
// update loop. I'm using that to print when it's coasting and stopped,
// but with a real stepper that will likely result in missed or extra steps
// since acceleration and deceleration is not correctly being maintained
// according to the step profile. I recommend treating
// these callbacks as if they were interrupt handlers, so that if the stepper is
// moving quickly, no steps are lost and the movement remains smooth.
// In other words, do not sleep in the callback like I am! This is just to see
// the output.
void aStoppedCallback(CallbackEvent event) {
  printf("Stopped Callback called\n");
  sleep_ms(500);
  // __breakpoint();
  sequence->nextPhase = 1;
  sequence->targetSpeed[0] = dist(gen);
  sequence->targetSpeed[1] = dist(gen);
  stepper->SetTargetHz(sequence->targetSpeed[0]);
  stepper->Start();
}
void aCoastingCallback(CallbackEvent event) {
  printf("Coasting Callback called with a frequency of %f\n",
         stepper->GetCurrentFrequency());
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
}

void aAcceleratingCallback(CallbackEvent event) {
  printf("Accelerating Callback called\n");
  //__breakpoint();
}

void aDeceleratingCallback(CallbackEvent event) {
  printf("Decelerating Callback Called\n");
  //__breakpoint();
}