#pragma once

#include <cstdint>
namespace PIOStepperSpeedController {
class Converter {
public:
  Converter(uint32_t aSysClk = 125000000, uint32_t aPrescaler = 1);

  uint32_t ToPeriod(float aFrequencyHz);
  float ToFrequency(uint32_t aPeriodTicks);
  float CalculateNextFrequency(uint32_t aCurrentPeriod, int32_t anAcceleration);

private:
  uint32_t mySysClk;
  uint32_t myPrescaler;
};

} // namespace PIOStepperSpeedController