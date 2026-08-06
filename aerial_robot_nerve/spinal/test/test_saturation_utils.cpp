#include <gtest/gtest.h>

#include <flight_control/attitude/saturation_utils.h>

TEST(SaturationUtils, PreservesUnsaturatedVector)
{
  const float base[] = {2.0f, 3.0f};
  const float roll_pitch[] = {1.0f, 1.0f};
  EXPECT_FLOAT_EQ(
      1.0f,
      spinal_saturation::maxBaseThrustScale(base, roll_pitch, 2, 10.0f));
}

TEST(SaturationUtils, SolvesScalarBaseShedding)
{
  const float base[] = {8.0f};
  const float roll_pitch[] = {4.0f};
  EXPECT_NEAR(
      0.75f,
      spinal_saturation::maxBaseThrustScale(base, roll_pitch, 1, 10.0f),
      1.0e-6f);
}

TEST(SaturationUtils, UsesSelectedRotorsWholeVector)
{
  // This is the rotor_coef=2 case that the old rotor-index/component-index
  // mix-up could not handle: the selected rotor's base force is in component 1.
  const float base[] = {0.0f, 8.0f};
  const float roll_pitch[] = {8.0f, 0.0f};
  EXPECT_NEAR(
      0.75f,
      spinal_saturation::maxBaseThrustScale(base, roll_pitch, 2, 10.0f),
      1.0e-6f);
}

TEST(SaturationUtils, FullyShedsWhenRollPitchAloneIsInfeasible)
{
  const float base[] = {0.0f, 4.0f, 0.0f};
  const float roll_pitch[] = {11.0f, 0.0f, 0.0f};
  EXPECT_FLOAT_EQ(
      0.0f,
      spinal_saturation::maxBaseThrustScale(base, roll_pitch, 3, 10.0f));
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
