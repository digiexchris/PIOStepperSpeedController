#include "Converter.hxx"
#include <stdexcept>

namespace PIOStepperSpeedController {

Converter::Converter(uint32_t aSysClk, uint32_t aPrescaler) {
  if (aPrescaler == 0) {
    throw std::invalid_argument("Prescaler cannot be zero");
  }
  mySysClk = aSysClk;
  myPrescaler = aPrescaler;
}

uint32_t Converter::ToPeriod(float aFrequencyHz) {
  if (aFrequencyHz <= 0) {
    throw std::invalid_argument("Frequency must be positive");
  }
  return static_cast<uint32_t>((static_cast<float>(mySysClk) / myPrescaler) /
                               aFrequencyHz);
}

float Converter::ToFrequency(uint32_t aPeriodTicks) {
  if (aPeriodTicks == 0) {
    throw std::invalid_argument("Period cannot be zero");
  }
  return static_cast<float>(mySysClk) / (myPrescaler * aPeriodTicks);
}

float Converter::CalculateNextFrequency(uint32_t aCurrentPeriod,
                                        int32_t anAcceleration) {
  if (anAcceleration == 0) {
    return ToFrequency(aCurrentPeriod);
  }

  return ToFrequency(aCurrentPeriod) -
         (static_cast<float>(anAcceleration) / aCurrentPeriod);
}

} // namespace PIOStepperSpeedController