#include "PIOStepper.hxx"
#include "PIOStepper.pio.h"
#include <cassert> // Needed for assert()
#include <cmath>   // Needed for sqrt() and PI calculations
#include <cstdint> // Needed for uint32_t, int32_t types

namespace PIOStepper
{
	PIOStepper::PIOStepper(PIO pio, uint sm, uint stepPin, uint startSpeedHz, float div, uint stepsPerRotation, uint acceleration, uint deceleration, Callback aStoppedCallback = nullptr, Callback aCoastingCallback = nullptr, Callback aAcceleratingCallback = nullptr, Callback aDeceleratingCallback = nullptr)
		: myPio(pio), mySm(sm), myStepPin(stepPin), myStartSpeedHz(startSpeedHz), myIsRunning(false), myTargetHz(0), myCurrentPeriod(0), myConfiguredDiv(div), myStepsPerRotation(stepsPerRotation), myAcceleration(acceleration), myDeceleration(deceleration), myOffset(0), mySysClk(0), myStoppedCallback(aStoppedCallback), myCoastingCallback(aCoastingCallback), myAcceleratingCallback(aAcceleratingCallback), myDeceleratingCallback(aDeceleratingCallback)
	{

		assert(startSpeedHz == 0);
		assert(div > 0);
		assert(stepsPerRotation > 0);
		assert(acceleration > 0);
		assert(deceleration > 0);

		// Define program inline
		static const uint16_t instructions[] = {
			0x8020, //  0: pull   block          <-- This is the 'public start:' label (offset 0)
			0xa027, //  1: mov    x, osr
			0xa042, //  2: mov    y, x        side 0
			0x0043, //  3: jmp    y--, 3      side 0
			0xa042, //  4: nop                 side 1
			0xa042, //  5: nop                 side 0
		};
		struct pio_program program = {
			.instructions = instructions,
			.length = 6,
			.origin = -1 // Relocatable
		};

		// Load program
		myOffset = pio_add_program(myPio, &program);

		// Configure state machine
		pio_sm_config c = pio_get_default_sm_config();
		sm_config_set_clkdiv(&c, div);
		pio_sm_init(myPio, mySm, myOffset, &c);
		mySysClk = clock_get_hz(clk_sys);
		sm_config_set_wrap(&c, myOffset + WRAP_TARGET, myOffset + WRAP);
		sm_config_set_sideset(&c, SIDESET_BITS, true, false);

		// Configure step pin
		pio_gpio_init(pio, step_pin);
		pio_sm_set_consecutive_pindirs(pio, sm, step_pin, 1, true);
		sm_config_set_sideset_pins(&c, step_pin);

		myState = CallbackEvent::STOPPED;
	}

	void PIOStepper::Start(float targetHz)
	{
		assert(targetHz > 0);
		myTargetHz = targetHz;
		if (myState != CallbackEvent::STOPPED)
		{
			return;
		}

		myCurrentPeriod = PrivFrequencyToPeriod(myStartSpeedHz);
		pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
		// Jump to start of program (offset 0)
		pio_sm_exec(myPio, mySm, pio_encode_jmp(myOffset));
		pio_sm_set_enabled(myPio, mySm, true);

		myState = CallbackEvent::ACCELERATING;
		if (myAcceleratingCallback != nullptr)
		{
			myAcceleratingCallback(CallbackEvent::ACCELERATING);
		}
	}

	void PIOStepper::ForceStop()
	{
		pio_sm_set_enabled(myPio, mySm, false);
		gpio_put(myStepPin, 0);
		myState = CallbackEvent::STOPPED;
		if (myStoppedCallback != nullptr)
		{
			myStoppedCallback(CallbackEvent::STOPPED);
		}
	}

	bool PIOStepper::Step()
	{
		if (myState == CallbackEvent::STOPPED)
		{
			return false;
		}

		// Calculate next period using internal target_hz
		int32_t nextPeriod = calculateNextPeriod(myCurrentPeriod, myTargetHz);

		if (nextPeriod >= PrivFrequencyToPeriod(myStartSpeedHz))
		{
			ForceStop();
			return false;
		}

		// Check if we're coasting
		else if (nextPeriod == myCurrentPeriod)
		{
			if (myState != CallbackEvent::COASTING)
			{
				myState = CallbackEvent::COASTING;
				if (myCoastingCallback != nullptr)
				{
					myCoastingCallback(CallbackEvent::COASTING);
				}
			}
		}

		else if (nextPeriod < myCurrentPeriod)
		{
			if (myState != CallbackEvent::ACCELERATING)
			{
				myState = CallbackEvent::ACCELERATING;
				if (myDeceleratingCallback != nullptr)
				{
					myDeceleratingCallback(CallbackEvent::ACCELERATING);
				}
			}
		}

		else if (nextPeriod > myCurrentPeriod)
		{

			if (myState != CallbackEvent::DECELERATING)
			{
				myState = CallbackEvent::DECELERATING;
				if (myDeceleratingCallback != nullptr)
				{
					myDeceleratingCallback(CallbackEvent::DECELERATING);
				}
			}
			else if (nextPeriod >= PrivFrequencyToPeriod(myTargetHz))
			{
				ForceStop();
				return false;
			}
		}

		// Update period and step
		myCurrentPeriod = nextPeriod;
		pio_sm_put_blocking(myPio, mySm, myCurrentPeriod);
		return true;
	}

	void PIOStepper::PrivSetEnabled(bool enabled)
	{
		pio_sm_set_enabled(myPio, mySm, enabled);
		if (!enabled)
		{
			gpio_put(myStepPin, 0);
		}
	}

	int32_t PIOStepper::PrivCalculateNextPeriod(uint32_t currentPeriod, float targetHz)
	{
		if (currentPeriod <= 0)
		{
			// First interval calculation
			double alpha = 2 * PI / myStepsPerRotation;
			if (myAcceleration <= 0)
			{
				return 0; // Prevent divide by zero
			}
			return (mySysClk / myConfiguredDiv) * std::sqrt(2 * alpha / myAcceleration);
		}

		// Calculate current speed in Hz (protect against divide by zero)
		if (currentPeriod == 0)
		{
			return 0;
		}
		double currentHz = (mySysClk / myConfiguredDiv) / (2.0 * currentPeriod);

		if (targetHz == 0)
		{
			// Specifically handle stopping case
			double currentHz = PrivPeriodToFrequency(currentPeriod);
			if (currentHz <= 0.0f)
			{
				return 0; // This will trigger a stop in the Step() function
			}
		}

		// Determine if we need to accelerate or decelerate
		if (currentHz > targetHz)
		{
			// Need to decelerate - increase period
			double alpha = 2 * PI / myStepsPerRotation;
			double currentInterval = currentPeriod * myConfiguredDiv / mySysClk;
			double radsPerSecondSquared = myDeceleration * alpha; // Use deceleration value

			if (alpha == 0)
			{
				return currentPeriod; // Prevent divide by zero
			}

			double nextInterval = currentInterval +
								  (2 * currentInterval * currentInterval * radsPerSecondSquared) / alpha;

			// Check if we're stopping and would go below zero Hz
			if (targetHz == 0)
			{
				if (nextInterval <= 0)
				{
					return currentPeriod; // Prevent divide by zero
				}
				double nextHz = mySysClk / (myConfiguredDiv * nextInterval * 2.0);
				if (nextHz <= 0)
				{
					return 0;
				}
			}

			return (mySysClk / myConfiguredDiv) * nextInterval;
		}
		else if (currentHz < targetHz)
		{
			// Need to accelerate - decrease period
			double alpha = 2 * PI / myStepsPerRotation;
			if (alpha == 0)
			{
				return currentPeriod; // Prevent divide by zero
			}

			double currentInterval = currentPeriod * myConfiguredDiv / mySysClk;
			double radsPerSecondSquared = myAcceleration * alpha;

			double nextInterval = currentInterval -
								  (2 * currentInterval * currentInterval * radsPerSecondSquared) / alpha;

			return (mySysClk / myConfiguredDiv) * nextInterval;
		}

		// At target speed - maintain current period
		return currentPeriod;
	}

	uint32_t PIOStepper::PrivFrequencyToPeriod(float hz) const
	{
		return hz > 0 ? (mySysClk / (myConfiguredDiv * hz * 2)) : 0;
	}

	float PIOStepper::PrivPeriodToFrequency(uint32_t period) const
	{
		return period > 0 ? (mySysClk / (myConfiguredDiv * period * 2.0)) : 0;
	}
} // namespace PowerFeed::Drivers