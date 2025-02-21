#include "Converter.hxx"
#include <gtest/gtest.h>

using namespace PIOStepperSpeedController;

TEST(ConverterTest, ConstructorValidation) {
  EXPECT_NO_THROW(Converter(133000000, 1));
  EXPECT_THROW(Converter(100000000, 0), std::invalid_argument);
}

TEST(ConverterTest, ToPeriodCalculation) {
  Converter conv(100000000, 1);
  EXPECT_EQ(conv.ToPeriod(1.0f), 100000000);
  EXPECT_EQ(conv.ToPeriod(100.0f), 1000000);
  EXPECT_THROW(conv.ToPeriod(0.0f), std::invalid_argument);
  EXPECT_THROW(conv.ToPeriod(-1.0f), std::invalid_argument);
}

TEST(ConverterTest, ToFrequencyCalculation) {
  Converter conv(100000000, 1);
  EXPECT_FLOAT_EQ(conv.ToFrequency(100000000), 1.0f);
  EXPECT_FLOAT_EQ(conv.ToFrequency(1000000), 100.0f);
  EXPECT_THROW(conv.ToFrequency(0), std::invalid_argument);
}

TEST(ConverterTest, PrescalerEffects) {
  Converter conv(100000000, 10);
  EXPECT_EQ(conv.ToPeriod(1.0f), 10000000);
  EXPECT_FLOAT_EQ(conv.ToFrequency(10000000), 1.0f);

  conv = Converter(125000000, 10);
  EXPECT_EQ(conv.ToPeriod(125000), 100);
  EXPECT_FLOAT_EQ(conv.ToFrequency(100), 125000);
}

TEST(ConverterTest, CalculateNextFrequency) {
  Converter conv(100000000, 1);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 0), 1.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100, 0), 100.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 1000), 1001.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100, 1000), 110.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -100), 999.9f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -1000), 999.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -2000), 998.0f);

  conv = Converter(100000000, 10);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 0), 1.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100, 0), 100.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 1000), 1001.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100, 1000), 110.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -100), 999.9f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -1000), 999.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -2000), 998.0f);

  conv = Converter(100000000, 33);
  EXPECT_NEAR(conv.CalculateNextFrequency(1, 0), 1.0f, 0.1f);
  EXPECT_NEAR(conv.CalculateNextFrequency(100, 0), 100.0f, 0.1f);
  EXPECT_NEAR(conv.CalculateNextFrequency(1, 1000), 1001.0f, 0.1f);
  EXPECT_NEAR(conv.CalculateNextFrequency(100, 1000), 110.0f, 0.1f);
  EXPECT_NEAR(conv.CalculateNextFrequency(1000, -100), 999.9f, 0.1f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -1000), 999.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000, -2000), 998.0f);

  conv = Converter(125000000, 1);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 0), 1.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100, 0), 100.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1, 1000), 1001.0f);
}

TEST(ConverterTest, AccelerateFrom1HzTo2001Hz) {
  Converter conv = Converter(125000000, 1);
  float nextFreq = 1.0f;
  float targetFreq = 2001.0f;
  int iterations = 0;

  while (nextFreq < targetFreq) {
    nextFreq = conv.CalculateNextFrequency(nextFreq, 1000);
    iterations++;
  }

  int expectedIterations = 1502; // From 125000 * ln(2001)
  EXPECT_EQ(iterations, expectedIterations);
  EXPECT_NEAR(nextFreq, 2001, 0.2f);
}

TEST(ConverterTest, AccelerateFrom100HzTo3333Hz) {
  Converter conv = Converter(125000000, 1);
  float nextFreq = 100.0f;
  float targetFreq = 3333.0f;
  int iterations = 0;
  float lastFreq = nextFreq;

  while (nextFreq < targetFreq) {
    lastFreq = nextFreq;
    nextFreq = conv.CalculateNextFrequency(nextFreq, 100);
    iterations++;
  }

  /*
  df = acceleration * ((sysclk/(prescaler * f)) * prescaler / sysclk)
         = acceleration * (1/f)

  N = ∫(f_start→f_end) df/(acceleration/f)
  = ∫(f_start→f_end) f/acceleration df
  = (1/acceleration) * ∫(f_start→f_end) f df
  = (1/acceleration) * [f²/2]₍₁₀₀→₃₃₃₃₎
  = (1/100) * (3333² - 100²)/2
  ≈ 55,494
  */
  int expectedIterations = 55494;
  EXPECT_EQ(iterations, expectedIterations);
  EXPECT_NEAR(nextFreq, 3333, 0.1f);
}