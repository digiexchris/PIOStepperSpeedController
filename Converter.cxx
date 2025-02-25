#include <PIOStepperSpeedController/Converter.hxx>
#include <stdexcept>

namespace PIOStepperSpeedController {

Converter::Converter(uint32_t aSysClk, uint32_t aPrescaler) {
  if (aPrescaler == 0) {
    throw std::invalid_argument("Prescaler cannot be zero");
  }
  mySysClk = aSysClk;
  myPrescaler = aPrescaler;
}

uint32_t Converter::ToPeriod(float aFrequencyHz) const {
  if (aFrequencyHz <= 0) {
    throw std::invalid_argument("Frequency must be positive");
  }
  return static_cast<uint32_t>((static_cast<float>(mySysClk) / myPrescaler) /
                               aFrequencyHz);
}

float Converter::ToFrequency(uint32_t aPeriodTicks) const {
  if (aPeriodTicks == 0) {
    throw std::invalid_argument("Period cannot be zero");
  }
  return static_cast<float>(mySysClk) / (myPrescaler * aPeriodTicks);
}

/*
df = acceleration * ((sysclk/(prescaler * f)) * prescaler / sysclk)
         = acceleration * (1/f)
 */
float Converter::CalculateNextFrequency(float currentFrequency,
                                        int32_t anAcceleration) const {
  if (anAcceleration == 0) {
    return currentFrequency;
  }

  // Convert frequency to period in ticks
  uint32_t currentPeriodTicks = ToPeriod(currentFrequency);

  // Calculate time for one period in seconds
  float periodInSeconds =
      static_cast<float>(currentPeriodTicks * myPrescaler) / mySysClk;

  // Calculate frequency change for this period
  float deltaFreq = static_cast<float>(anAcceleration) * periodInSeconds;

  return currentFrequency + deltaFreq;
}
} // namespace PIOStepperSpeedController