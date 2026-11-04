#include <gtest/gtest.h>
#include <hih/control/hih_admittance.h>
#include <hih/control/hih_contact_wrench.h>
#include <limits>

using namespace hih_control;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Admittance, FourAxesRespondWithCorrectSign)
{
  AdmittanceParameters p; p.deadband.fill(0); p.filter_cutoff_hz = 0;
  Admittance4D model(p);
  ASSERT_TRUE(model.step({{1, -1, 1, -0.1}}, 0.025));
  EXPECT_GT(model.offset()[0], 0); EXPECT_LT(model.offset()[1], 0);
  EXPECT_GT(model.offset()[2], 0); EXPECT_LT(model.offset()[3], 0);
}

TEST(Admittance, ZeroErrorDoesNotMove)
{
  Admittance4D model;
  for (int i = 0; i < 1000; ++i) ASSERT_TRUE(model.step({{0, 0, 0, 0}}, 0.025));
  for (double x : model.offset()) EXPECT_DOUBLE_EQ(x, 0);
}

TEST(Admittance, BoundsAndAntiWindup)
{
  AdmittanceParameters p; p.deadband.fill(0); p.filter_cutoff_hz = 0;
  Admittance4D model(p);
  for (int i = 0; i < 5000; ++i) ASSERT_TRUE(model.step({{1e6, -1e6, 1e6, 1e6}}, 0.025));
  for (int i = 0; i < 4; ++i)
  {
    EXPECT_LE(std::abs(model.offset()[i]), p.max_offset[i]);
    EXPECT_LE(std::abs(model.velocity()[i]), p.max_velocity[i]);
    EXPECT_LE(std::abs(model.filteredWrench()[i]), p.max_wrench[i]);
    EXPECT_DOUBLE_EQ(model.velocity()[i], 0);
  }
  // Reverse contact error must release the bound immediately, not unwind a hidden integrator.
  const double previous = model.offset()[0];
  ASSERT_TRUE(model.step({{-1e6, 1e6, -1e6, -1e6}}, 0.025));
  EXPECT_LT(model.offset()[0], previous);
}

TEST(Admittance, HoldFreezesOffsetAndRemovesVelocity)
{
  Admittance4D model;
  for (int i = 0; i < 20; ++i) model.step({{1, 0, 0, 0.1}}, 0.025);
  const auto offset = model.offset();
  model.hold();
  EXPECT_EQ(offset, model.offset());
  for (double v : model.velocity()) EXPECT_DOUBLE_EQ(v, 0);
}

TEST(Admittance, InvalidTimeAndInputDoNotCorruptState)
{
  Admittance4D model;
  model.step({{1, 0, 0, 0}}, 0.025);
  const auto before = model.offset();
  EXPECT_FALSE(model.step({{1, 0, 0, 0}}, 0));
  EXPECT_FALSE(model.step({{1, 0, 0, 0}}, -0.1));
  EXPECT_FALSE(model.step({{1, 0, 0, 0}}, 0.5));
  EXPECT_FALSE(model.step({{std::numeric_limits<double>::quiet_NaN(), 0, 0, 0}}, 0.025));
  EXPECT_EQ(before, model.offset());
}

TEST(Admittance, ZeroOffsetLimitDisablesAnAxis)
{
  AdmittanceParameters p; p.max_offset[2] = p.max_offset[3] = 0;
  Admittance4D model(p);
  for (int i = 0; i < 100; ++i) model.step({{1, 1, 1, 1}}, 0.025);
  EXPECT_DOUBLE_EQ(model.offset()[2], 0); EXPECT_DOUBLE_EQ(model.offset()[3], 0);
  EXPECT_DOUBLE_EQ(model.velocity()[2], 0); EXPECT_DOUBLE_EQ(model.velocity()[3], 0);
}

TEST(Admittance, RejectsInvalidPhysicalParameters)
{
  AdmittanceParameters p; p.mass[3] = 0;
  EXPECT_THROW(Admittance4D model(p), std::invalid_argument);
  p = AdmittanceParameters(); p.damping[0] = -1;
  EXPECT_THROW(Admittance4D model(p), std::invalid_argument);
  p = AdmittanceParameters(); p.bias[0] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Admittance4D model(p), std::invalid_argument);
}

TEST(Admittance, DeadbandAndBias)
{
  AdmittanceParameters p; p.bias[0] = 2;
  Admittance4D model(p);
  for (int i = 0; i < 100; ++i) model.step({{2.01, 0, 0, 0}}, 0.025);
  EXPECT_DOUBLE_EQ(model.offset()[0], 0);
}

TEST(Admittance, PositiveStiffnessConvergesToForceOverStiffness)
{
  AdmittanceParameters p;
  p.stiffness.fill(20); p.max_offset.fill(1); p.deadband.fill(0); p.filter_cutoff_hz = 0;
  Admittance4D model(p);
  for (int i = 0; i < 2000; ++i) model.step({{1, 1, 1, 0.1}}, 0.025);
  EXPECT_NEAR(model.offset()[0], 0.05, 1e-6);
  EXPECT_NEAR(model.offset()[3], 0.005, 1e-6);
}

TEST(ContactWrench, DesiredCompressionCancelsMeasuredReaction)
{
  tf::Matrix3x3 identity; identity.setIdentity();
  const auto error = contactWrenchErrorWorld({tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, 0)},
                                           identity, tf::Vector3(1, 0, 0), 10);
  for (double x : error) EXPECT_DOUBLE_EQ(x, 0);
}

TEST(ContactWrench, TooLittleForceMovesTowardPayloadAndTooMuchRetreats)
{
  tf::Matrix3x3 identity; identity.setIdentity();
  auto low = contactWrenchErrorWorld({tf::Vector3(-5, 0, 0), tf::Vector3(0, 0, 0)},
                                   identity, tf::Vector3(1, 0, 0), 10);
  auto high = contactWrenchErrorWorld({tf::Vector3(-15, 0, 0), tf::Vector3(0, 0, 0)},
                                    identity, tf::Vector3(1, 0, 0), 10);
  EXPECT_GT(low[0], 0); EXPECT_LT(high[0], 0);
}

TEST(ContactWrench, RotatedNormalAndYawMomentUseWorldAxes)
{
  tf::Matrix3x3 rotation; rotation.setRPY(0, 0, 1.5707963267948966);
  auto error = contactWrenchErrorWorld({tf::Vector3(-5, 0, 0), tf::Vector3(0, 0, 0.2)},
                                     rotation, tf::Vector3(1, 0, 0), 10);
  EXPECT_NEAR(error[0], 0, 1e-12); EXPECT_NEAR(error[1], 5, 1e-12);
  EXPECT_NEAR(error[3], 0.2, 1e-12);
}

TEST(ContactWrench, MomentShiftRemovesLeverArmContribution)
{
  tf::Matrix3x3 identity; identity.setIdentity();
  tf::Transform cog_from_contact(identity, tf::Vector3(0.2, 0, 0));
  const auto contact = wrenchAtContact(tf::Vector3(-10, 2, 0), tf::Vector3(0, 0, 0.4),
                                      identity, cog_from_contact);
  EXPECT_NEAR(contact.torque.z(), 0, 1e-12);
  EXPECT_NEAR(contact.force.x(), -10, 1e-12);
}

TEST(ContactWrench, YawCompensationPreservesContactCenterPosition)
{
  tf::Matrix3x3 nominal, command;
  nominal.setRPY(0, 0, 0.3); command.setRPY(0, 0, 0.5);
  const tf::Vector3 lever(0.2, 0.01, 0);
  const auto compensation = contactCenterCompensation(nominal, command, lever);
  const auto after = compensation + command * lever;
  EXPECT_NEAR((after - nominal * lever).length(), 0, 1e-12);
}
