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
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100000000, 0), 1.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000000, 0), 100.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100000000, 1000000), 0.99f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000000, 1000000), 99.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100000000, -1000000), 1.01f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(1000000, -1000000), 101.0f);

  conv = Converter(100000000, 10);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(10000000, 0), 1.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100000, 0), 100.0f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(10000000, 100000), 0.99f);
  EXPECT_FLOAT_EQ(conv.CalculateNextFrequency(100000, 100000), 99.0f);

  conv = Converter(100000000, 33);
  EXPECT_NEAR(conv.CalculateNextFrequency(3030303, 0), 1.0f, 0.01);
  EXPECT_NEAR(conv.CalculateNextFrequency(30303, 0), 100.0f, 0.01);
  EXPECT_NEAR(conv.CalculateNextFrequency(3030303, 30303), 0.99f, 0.01);
  EXPECT_NEAR(conv.CalculateNextFrequency(30303, 30303), 99.0f, 0.01);
}