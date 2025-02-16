/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <cstdlib>
#include <pico/sem.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <stdio.h>

#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <PIOStepperSpeedController.hxx>

using namespace PIOStepperSpeedController;

Stepper *stepper = nullptr;
static semaphore_t stepperSemaphore;

// If you take too long in any of these callbacks, it will delay the stepper
// update loop I'm using that to stop and print when it's coasting and stopped,
// but with a real stepper that will likely result in missed or extra steps
// since acceleration and deceleration is not correctly being maintained when I
// pause in this example. I recommend treating these callbacks as if they were
// interrupt handlers, so that if the stepper is moving quickly, no steps are
// lost and the movement remains smooth.
void aStoppedCallback(Stepper::CallbackEvent event) {
  printf("Stopped Callback called\n");
  sem_release(&stepperSemaphore);
}
void aCoastingCallback(Stepper::CallbackEvent event) {
  printf("Coasting Callback called\n");
  sem_release(&stepperSemaphore);
}

void aAcceleratingCallback(Stepper::CallbackEvent event) {
  printf("Accelerating Callback called\n");
  sem_release(&stepperSemaphore);
}

void aDeceleratingCallback(Stepper::CallbackEvent event) {
  printf("Decelerating Callback Called\n");
  sem_release(&stepperSemaphore);
}

void update() {
  bool aStepOccurred = false;

  // stepper->Step() should be repeatedly called. I normally do this in it's own
  // FreeRTOS task but for simple example's sake, I'm going to reuse the same
  // loop for running the example and setting multiple speeds.
  while (true) {

    // maintain the internal state of the stepper. Call this as fast as possible
    // and this will block until the step is complete.
    // if it returns false, the stepper is stopped and would not have executed
    // a step.
    aStepOccurred = stepper->Step();

    if (!aStepOccurred) {
      // this allows you to do any state management you wish when the stepper is
      // stopped in this case I'm just delaying for a second to demonstrating
      // the blocking nature of slowing this loop.
      sleep_ms(1000);
    }
  }
}

int main() {

  // Just for synchronizing these two threads down below
  sem_init(&stepperSemaphore, 1, 1); // Initialize semaphore with count of 1

  const uint stepPin = 2;

  // due to a divide by zero issue, startSpeedHz must NOT be 0. Consider this
  // the speed at which the motor is capable of moving at from a dead stop
  // without needing acceleration 1 to 10 steps per second is a good starting
  // point, feel free to tune it from there if it takes too long to start
  // accelerating 10hz, or 10 steps per second
  const uint startSpeedHz = 10;      // steps per second
  const uint stepsPerRotation = 200; // steps per rotation of the motor
  const uint maxSpeed = 10000;       // steps per second
  const uint acceleration = 100;     // steps per second squared
  const uint deceleration = 200;     // steps per second squared

  stdio_init_all();

  // All callbacks are optional, and are only used here to demonstrate a few
  // possibilities
  stepper = new Stepper(stepPin, startSpeedHz, maxSpeed, stepsPerRotation,
                        acceleration, deceleration, aStoppedCallback,
                        aCoastingCallback, aAcceleratingCallback,
                        aDeceleratingCallback);

  // Starting the stepper update loop on the second core. Normally I use
  // FreeRTOS tasks for this, but for simplicity I'm using the multicore
  // functions.
  multicore_reset_core1();
  multicore_launch_core1(update);

  // At this point the pio program is initialized and ready, but in a stopped
  // state. It will automatically start when a speed is requested.

  // Now, do something interesting with the motor, like set it to a random
  // speed between 0 and the max, and wait till it reaches that speed

  enum class state {
    stopped,
    accelerating,
    decelerating,
    coasting
  } state = state::stopped;
  uint targetSpeed = 0;
  while (true) {

    // wait until the stepper has changed states and notifies via the callbacks
    sem_acquire_blocking(&stepperSemaphore);
    if (state == state::stopped) {
      targetSpeed = rand() % maxSpeed;
      stepper->Start(targetSpeed);
    } else if (state == state::accelerating) {
      printf("Still accelerating!");
    } else if (state == state::decelerating) {
      printf("Still decelerating!");
    } else if (state == state::coasting) {
      stepper->SetTargetHz(0);
    }

    sleep_ms(500);
  }
}
