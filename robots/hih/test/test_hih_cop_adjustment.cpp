#include <gtest/gtest.h>
#include <hih/control/hih_cop_adjustment.h>
#include <hih/control/hih_contact_wrench.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <limits>

using namespace hih_control;
using Snapshot = aerial_robot_navigation::BaseNavigator::TargetSnapshot;

namespace
{
constexpr double pi = 3.14159265358979323846;

Snapshot reference()
{
  Snapshot target;
  target.pos = tf::Vector3(1, 2, 3); target.vel = tf::Vector3(.1, .2, .3);
  target.acc = tf::Vector3(.4, .5, .6); target.rpy = tf::Vector3(.2, -.3, .4);
  target.omega = tf::Vector3(.03, -.02, .01); target.ang_acc = tf::Vector3(.7, .8, .9);
  return target;
}

void expectVector(const tf::Vector3& a, const tf::Vector3& b, double tolerance = 1e-12)
{ for (int i = 0; i < 3; ++i) EXPECT_NEAR(a[i], b[i], tolerance); }

class Cop : public ::testing::Test
{
protected:
  CopAdjustmentParameters p;
  HihCopAdjustment model;
  double time = 10;
  void SetUp() override { p.enabled = true; p.filter_cutoff_hz = 0; model.configure(p); }
  CopAdjustmentState tick(double y, double z, double fn = 10, bool flight = true, bool adm = true)
  {
    time += .025;
    const tf::Vector3 force(-fn, 1.3, -.7);
    const tf::Vector3 torque = tf::Vector3(0, y, z).cross(force) + tf::Vector3(.31, 0, 0);
    return model.step(force, torque, time, true, time, .025, flight, adm);
  }
};
}  // namespace

TEST_F(Cop, ReconstructsPlanarCoPWithShearAndNormalTorsion)
{
  for (double y : {-.08, 0.0, .08})
    for (double z : {-.07, 0.0, .07})
    {
      const auto state = tick(y, z);
      ASSERT_TRUE(state.valid);
      expectVector(state.raw, tf::Vector3(0, y, z));
      EXPECT_DOUBLE_EQ(state.normal_force, 10);
    }
}

TEST_F(Cop, RotatedCogWrenchMustFirstBeShiftedToContactCenter)
{
  tf::Matrix3x3 world_from_cog, cog_from_contact_rotation;
  world_from_cog.setRPY(.15, -.2, .7); cog_from_contact_rotation.setRPY(-.1, .2, -.3);
  const tf::Vector3 lever(.26, .02, -.03), force_contact(-12, 2, -3), cop(0, .06, -.04);
  const tf::Vector3 torque_contact = cop.cross(force_contact) + tf::Vector3(.4, 0, 0);
  const tf::Vector3 force_cog = cog_from_contact_rotation * force_contact;
  const tf::Vector3 torque_cog = cog_from_contact_rotation * torque_contact + lever.cross(force_cog);
  const auto contact = wrenchAtContact(world_from_cog * force_cog, torque_cog,
    world_from_cog, tf::Transform(cog_from_contact_rotation, lever));
  const auto result = model.step(contact.force, contact.torque, 10, true, 10, .025, true, true);
  ASSERT_TRUE(result.valid);
  expectVector(result.raw, cop);
}

TEST_F(Cop, ZeroWeakAndTensileForceAreInvalidBeforeDivision)
{
  for (double fn : {0.0, .5, -2.0})
  {
    const auto result = tick(.1, -.1, fn);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.status, "low_normal_force_held");
    EXPECT_TRUE(std::isnan(result.raw.y()));
    EXPECT_DOUBLE_EQ(result.yaw_offset, 0); EXPECT_DOUBLE_EQ(result.z_offset, 0);
  }
}

TEST_F(Cop, ExactForceThresholdIsAccepted)
{ EXPECT_TRUE(tick(.02, -.03, p.min_normal_force).valid); }

TEST_F(Cop, InsideAndOnSafetyBoundaryNoOffsetAccumulates)
{
  for (int i = 0; i < 1000; ++i)
  {
    const auto result = tick(i % 2 ? .05 : -.05, i % 2 ? -.05 : .05);
    EXPECT_DOUBLE_EQ(result.yaw_offset, 0); EXPECT_DOUBLE_EQ(result.z_offset, 0);
    EXPECT_FALSE(result.applying);
  }
}

TEST_F(Cop, UsesBoundaryExcessAndRateGainRatherThanDistanceFromCenter)
{
  const auto result = tick(.07, -.06);
  EXPECT_NEAR(result.error.y(), .02, 1e-12); EXPECT_NEAR(result.error.z(), -.01, 1e-12);
  EXPECT_NEAR(result.yaw_rate, p.yaw.gain * .02, 1e-12);
  EXPECT_NEAR(result.z_velocity, -p.z.gain * .01, 1e-12);
  EXPECT_NEAR(result.yaw_offset, p.yaw.gain * .02 * .025, 1e-12);
  EXPECT_NEAR(result.z_offset, -p.z.gain * .01 * .025, 1e-12);
}

TEST_F(Cop, PositiveAndNegativeSignsFollowConfiguredDirections)
{
  const auto positive = tick(.08, .08);
  EXPECT_GT(positive.yaw_offset, 0); EXPECT_GT(positive.z_offset, 0);
  p.yaw.direction = p.z.direction = -1;
  model.configure(p);
  const auto negative = tick(.08, .08);
  EXPECT_LT(negative.yaw_offset, 0); EXPECT_LT(negative.z_offset, 0);
}

TEST_F(Cop, ZeroGainMeansZeroAdjustment)
{
  p.yaw.gain = p.z.gain = 0; model.configure(p);
  for (int i = 0; i < 100; ++i) tick(.2, -.2);
  EXPECT_DOUBLE_EQ(model.state().yaw_offset, 0); EXPECT_DOUBLE_EQ(model.state().z_offset, 0);
}

TEST_F(Cop, AxisEnablesAreIndependent)
{
  p.yaw.enabled = false; model.configure(p);
  const auto z_only = tick(.1, .1);
  EXPECT_DOUBLE_EQ(z_only.yaw_offset, 0); EXPECT_GT(z_only.z_offset, 0);
  p.yaw.enabled = true; p.z.enabled = false; model.configure(p);
  const auto yaw_only = tick(.1, .1);
  EXPECT_GT(yaw_only.yaw_offset, 0); EXPECT_DOUBLE_EQ(yaw_only.z_offset, 0);
}

TEST_F(Cop, SaturationStoresNoWindupAndReleasesImmediately)
{
  for (int i = 0; i < 5000; ++i)
  {
    const auto state = tick(.2, -.2);
    EXPECT_LE(std::abs(state.yaw_offset), p.yaw.max_offset);
    EXPECT_LE(std::abs(state.z_offset), p.z.max_offset);
    EXPECT_LE(std::abs(state.yaw_rate), p.yaw.max_rate + 1e-12);
    EXPECT_LE(std::abs(state.z_velocity), p.z.max_rate + 1e-12);
  }
  const auto bounded = model.state();
  EXPECT_TRUE(bounded.yaw_saturated); EXPECT_TRUE(bounded.z_saturated);
  EXPECT_DOUBLE_EQ(bounded.yaw_rate, 0); EXPECT_DOUBLE_EQ(bounded.z_velocity, 0);
  const auto reverse = tick(-.2, .2);
  EXPECT_LT(reverse.yaw_offset, bounded.yaw_offset); EXPECT_GT(reverse.z_offset, bounded.z_offset);
}

TEST_F(Cop, CoPInsideBoundaryHoldsTheLearnedOffset)
{
  for (int i = 0; i < 10; ++i) tick(.1, -.1);
  const auto before = model.state();
  for (int i = 0; i < 100; ++i) tick(0, 0);
  const auto after = model.state();
  EXPECT_DOUBLE_EQ(before.yaw_offset, after.yaw_offset); EXPECT_DOUBLE_EQ(before.z_offset, after.z_offset);
  EXPECT_DOUBLE_EQ(after.yaw_rate, 0); EXPECT_DOUBLE_EQ(after.z_velocity, 0);
  EXPECT_TRUE(after.applying);
}

TEST_F(Cop, HysteresisLatchDoesNotIntegrateInsideTheBoundary)
{
  EXPECT_TRUE(tick(.06, 0).yaw_boundary_active);
  const auto inside = tick(.047, 0);
  EXPECT_TRUE(inside.yaw_boundary_active); EXPECT_DOUBLE_EQ(inside.yaw_rate, 0);
  EXPECT_FALSE(tick(.04, 0).yaw_boundary_active);
}

TEST_F(Cop, MasterOffReturnsSmoothlyAndPublishesActualOffsetDerivative)
{
  for (int i = 0; i < 30; ++i) tick(.1, -.1);
  auto previous = model.state();
  model.setEnabled(false, true, true);
  for (int i = 0; i < 300; ++i)
  {
    const auto current = tick(.1, -.1);
    EXPECT_NEAR(current.yaw_rate, (current.yaw_offset - previous.yaw_offset) / .025, 1e-12);
    EXPECT_NEAR(current.z_velocity, (current.z_offset - previous.z_offset) / .025, 1e-12);
    EXPECT_LE(std::abs(current.yaw_offset), std::abs(previous.yaw_offset));
    EXPECT_LE(std::abs(current.z_offset), std::abs(previous.z_offset));
    previous = current;
  }
  EXPECT_DOUBLE_EQ(previous.yaw_offset, 0); EXPECT_DOUBLE_EQ(previous.z_offset, 0);
  EXPECT_FALSE(previous.applying);
}

TEST_F(Cop, OneAxisOffReturnsWithoutStoppingTheOtherAxis)
{
  for (int i = 0; i < 20; ++i) tick(.1, .1);
  const auto previous = model.state();
  model.setEnabled(true, false, true);
  const auto current = tick(.1, .1);
  EXPECT_LT(current.yaw_offset, previous.yaw_offset); EXPECT_GT(current.z_offset, previous.z_offset);
}

TEST_F(Cop, StaleFutureAndInvalidSampleHoldOffsetsWithoutFakeZeroForce)
{
  tick(.1, .1);
  const auto previous = model.state();
  for (double stamp : {time - 1, time + 1, 0.0})
  {
    const auto current = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, 1),
      stamp, true, time, .025, true, true);
    EXPECT_FALSE(current.valid);
    EXPECT_DOUBLE_EQ(current.yaw_offset, previous.yaw_offset);
    EXPECT_DOUBLE_EQ(current.z_offset, previous.z_offset);
    EXPECT_DOUBLE_EQ(current.yaw_rate, 0); EXPECT_DOUBLE_EQ(current.z_velocity, 0);
  }
  const auto invalid = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, 1),
    time, false, time, .025, true, true);
  EXPECT_FALSE(invalid.valid); EXPECT_DOUBLE_EQ(invalid.yaw_offset, previous.yaw_offset);
}

TEST_F(Cop, LostContactHoldsOffsetsAndReacquisitionReseedsFilter)
{
  p.filter_cutoff_hz = 1; model.configure(p);
  tick(.1, .1); const auto before = model.state();
  const auto lost = tick(0, 0, 0);
  EXPECT_FALSE(lost.valid); EXPECT_DOUBLE_EQ(lost.yaw_offset, before.yaw_offset);
  const auto regained = tick(0, 0);
  EXPECT_TRUE(regained.valid); expectVector(regained.filtered, tf::Vector3(0, 0, 0));
  EXPECT_DOUBLE_EQ(regained.yaw_rate, 0);
}

TEST_F(Cop, AdmittanceInactiveHoldsButDoesNotIntegrate)
{
  tick(.1, .1); const auto before = model.state();
  const auto after = tick(.2, .2, 10, true, false);
  EXPECT_DOUBLE_EQ(after.yaw_offset, before.yaw_offset); EXPECT_DOUBLE_EQ(after.z_offset, before.z_offset);
  EXPECT_DOUBLE_EQ(after.yaw_rate, 0); EXPECT_EQ(after.status, "admittance_inactive_held");
}

TEST_F(Cop, LeavingFlightModeClearsTrimAndResetPreventsCarryover)
{
  tick(.1, .1);
  const auto inactive = tick(.1, .1, 10, false, true);
  EXPECT_DOUBLE_EQ(inactive.yaw_offset, 0); EXPECT_DOUBLE_EQ(inactive.z_offset, 0);
  EXPECT_FALSE(inactive.applying);
  tick(.1, .1); model.reset();
  EXPECT_FALSE(model.state().applying); EXPECT_FALSE(model.state().valid);
  EXPECT_DOUBLE_EQ(tick(0, 0).yaw_offset, 0);
}

TEST_F(Cop, InvalidControlIntervalsNeverInventMotion)
{
  tick(.1, -.1); const auto before = model.state();
  for (double dt : {0.0, -.01, .5, std::numeric_limits<double>::quiet_NaN()})
  {
    const auto after = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 1, 1),
      time, true, time, dt, true, true);
    EXPECT_DOUBLE_EQ(after.yaw_offset, before.yaw_offset); EXPECT_DOUBLE_EQ(after.z_offset, before.z_offset);
    EXPECT_DOUBLE_EQ(after.yaw_rate, 0); EXPECT_DOUBLE_EQ(after.z_velocity, 0);
  }
}

TEST_F(Cop, NonfiniteWrenchNeverEntersReference)
{
  tick(.1, -.1); const auto before = model.state();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const tf::Vector3& bad : {tf::Vector3(nan, 0, 0), tf::Vector3(-10, inf, 0)})
  {
    const auto after = model.step(bad, tf::Vector3(0, 1, 1), time, true, time, .025, true, true);
    EXPECT_FALSE(after.valid); EXPECT_DOUBLE_EQ(after.yaw_offset, before.yaw_offset);
  }
  EXPECT_FALSE(model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, nan, 1),
    time, true, time, .025, true, true).valid);
}

TEST_F(Cop, OutliersAreRejectedRatherThanClippedIntoTheSafeArea)
{
  tick(.1, 0); const auto before = model.state();
  const auto after = tick(1.0, 0);
  EXPECT_FALSE(after.valid); EXPECT_NEAR(after.raw.y(), 1.0, 1e-12);
  EXPECT_EQ(after.status, "cop_outlier_held"); EXPECT_DOUBLE_EQ(after.yaw_offset, before.yaw_offset);
}

TEST_F(Cop, FilterUsesNewMeasurementTimeAndDoesNotRefilterDuplicateStamps)
{
  p.filter_cutoff_hz = 2; model.configure(p);
  tick(.1, 0);
  auto duplicate = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, -1),
    time, true, time, .025, true, true);
  EXPECT_NEAR(duplicate.filtered.y(), .1, 1e-12);
  const double alpha = 1 - std::exp(-2 * pi * 2 * .01);
  const auto next = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, -1),
    time + .01, true, time + .01, .025, true, true);
  EXPECT_NEAR(next.filtered.y(), .1 + alpha * (-.1 - .1), 1e-12);
}

TEST_F(Cop, OutOfOrderSamplesCannotResumeIntegration)
{
  tick(.1, 0); const auto before = model.state();
  for (int i = 0; i < 2; ++i)
  {
    const auto older = model.step(tf::Vector3(-10, 0, 0), tf::Vector3(0, 0, 1),
      time - .01, true, time, .025, true, true);
    EXPECT_FALSE(older.valid); EXPECT_EQ(older.status, "out_of_order_input_held");
    EXPECT_DOUBLE_EQ(older.yaw_offset, before.yaw_offset);
  }
  EXPECT_TRUE(tick(0, 0).valid);
}

TEST_F(Cop, BiasIsAppliedInContactFrameOnly)
{
  p.force_bias_contact = tf::Vector3(.5, -.2, .1);
  p.torque_bias_contact = tf::Vector3(.1, -.02, .03); model.configure(p);
  const tf::Vector3 force(-10, 2, -1), expected(0, .07, -.03);
  const tf::Vector3 torque = expected.cross(force) + tf::Vector3(.1, 0, 0);
  const auto after = model.step(force + p.force_bias_contact, torque + p.torque_bias_contact,
    10, true, 10, .025, true, true);
  ASSERT_TRUE(after.valid); expectVector(after.raw, expected);
}

TEST_F(Cop, DisabledOrZeroCorrectionIsAnExactReferenceNoOp)
{
  model.setEnabled(false, true, true); tick(.1, .1);
  const auto before = reference(); auto after = before; model.applyReference(after);
  EXPECT_EQ(before.pos, after.pos); EXPECT_EQ(before.vel, after.vel); EXPECT_EQ(before.acc, after.acc);
  EXPECT_EQ(before.rpy, after.rpy); EXPECT_EQ(before.omega, after.omega); EXPECT_EQ(before.ang_acc, after.ang_acc);
}

TEST_F(Cop, AddsOnlyWorldZAndEulerYawAfterAdmittance)
{
  const auto state = tick(.1, .1);
  const auto before = reference(); auto after = before; model.applyReference(after);
  EXPECT_DOUBLE_EQ(after.pos.x(), before.pos.x()); EXPECT_DOUBLE_EQ(after.pos.y(), before.pos.y());
  EXPECT_DOUBLE_EQ(after.rpy.x(), before.rpy.x()); EXPECT_DOUBLE_EQ(after.rpy.y(), before.rpy.y());
  EXPECT_NEAR(after.pos.z(), before.pos.z() + state.z_offset, 1e-12);
  EXPECT_NEAR(after.vel.z(), before.vel.z() + state.z_velocity, 1e-12);
  EXPECT_NEAR(after.rpy.z(), before.rpy.z() + state.yaw_offset, 1e-12);
  EXPECT_DOUBLE_EQ(after.vel.x(), before.vel.x()); EXPECT_DOUBLE_EQ(after.vel.y(), before.vel.y());
  EXPECT_EQ(after.acc, before.acc); EXPECT_EQ(after.ang_acc, before.ang_acc);
}

TEST_F(Cop, BodyOmegaMatchesFiniteDifferenceOfYawOffsetComposition)
{
  const auto state = tick(.1, 0);
  const auto before = reference(); auto after = before; model.applyReference(after);
  tf::Matrix3x3 r_before, yaw_offset, r_after;
  r_before.setRPY(before.rpy.x(), before.rpy.y(), before.rpy.z());
  r_after.setRPY(after.rpy.x(), after.rpy.y(), after.rpy.z());
  yaw_offset.setRPY(0, 0, state.yaw_offset);
  const double h = 1e-6;
  tf::Quaternion body_increment(before.omega.normalized(), before.omega.length() * h);
  tf::Matrix3x3 next_offset; next_offset.setRPY(0, 0, state.yaw_offset + state.yaw_rate * h);
  const tf::Matrix3x3 next = next_offset * r_before * tf::Matrix3x3(body_increment);
  const tf::Matrix3x3 delta = next * r_after.inverse();
  const tf::Vector3 observed_world((delta[2][1] - delta[1][2]) / (2 * h),
    (delta[0][2] - delta[2][0]) / (2 * h), (delta[1][0] - delta[0][1]) / (2 * h));
  expectVector(observed_world, r_after * after.omega, 1e-6);
}

TEST_F(Cop, ZOnlyTrimDoesNotRewriteAnUnwrappedYawReference)
{
  p.yaw.enabled = false; model.configure(p); tick(.1, .1);
  auto target = reference(); target.rpy.setZ(7.0);
  const auto omega = target.omega;
  model.applyReference(target);
  EXPECT_DOUBLE_EQ(target.rpy.z(), 7.0); EXPECT_EQ(target.omega, omega);
}

TEST_F(Cop, FreshReferenceDoesNotAccumulateTrimTwice)
{
  const auto state = tick(.1, .1);
  const auto base = reference();
  for (int i = 0; i < 1000; ++i)
  {
    auto target = base; model.applyReference(target);
    EXPECT_NEAR(target.pos.z() - base.pos.z(), state.z_offset, 1e-12);
    EXPECT_NEAR(target.rpy.z() - base.rpy.z(), state.yaw_offset, 1e-12);
  }
}

TEST_F(Cop, YawWrapIsNormalizedWithoutChangingOtherAxes)
{
  tick(.1, 0);
  auto target = reference(); target.rpy.setZ(pi - 1e-6); model.applyReference(target);
  EXPECT_GE(target.rpy.z(), -pi); EXPECT_LT(target.rpy.z(), pi);
  EXPECT_DOUBLE_EQ(target.rpy.x(), .2); EXPECT_DOUBLE_EQ(target.rpy.y(), -.3);
}

TEST_F(Cop, UnsupportedContactNormalIsNeverUsedSilently)
{
  EXPECT_THROW(model.configure(p, tf::Vector3(0, 0, 1)), std::invalid_argument);
  p.enabled = false; model.configure(p, tf::Vector3(0, 0, 1));
  model.setEnabled(true, true, true);
  EXPECT_FALSE(tick(.1, .1).valid); EXPECT_FALSE(model.state().applying);
}

TEST_F(Cop, InvalidParametersAreRejectedBeforeChangingState)
{
  for (int item = 0; item < 9; ++item)
  {
    auto bad = p;
    if (item == 0) bad.min_normal_force = 0;
    if (item == 1) bad.safe_y = -1;
    if (item == 2) bad.hysteresis_z = bad.safe_z;
    if (item == 3) bad.yaw.direction = 0;
    if (item == 4) bad.z.gain = std::numeric_limits<double>::quiet_NaN();
    if (item == 5) bad.yaw.max_offset = pi;
    if (item == 6) bad.sample_timeout = -1;
    if (item == 7) bad.max_abs_cop_y = .01;
    if (item == 8) bad.force_bias_contact.setX(std::numeric_limits<double>::infinity());
    EXPECT_THROW(model.configure(bad), std::invalid_argument);
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST_F(Cop, RearPlateRecoversCopAndReversesOnlyYawGeometry)
{
  model.configure(p,tf::Vector3(-1,0,0));
  const tf::Vector3 force(10,2,-1), cop(0,.08,.09);
  const auto s=model.step(force,cop.cross(force),10,true,10,.01,true,true);
  EXPECT_TRUE(s.valid); EXPECT_DOUBLE_EQ(s.normal_force,10);
  EXPECT_NEAR(s.raw.y(),.08,1e-12); EXPECT_NEAR(s.raw.z(),.09,1e-12);
  EXPECT_LT(s.yaw_offset,0); EXPECT_GT(s.z_offset,0);
  const auto wrong=model.step(tf::Vector3(-10,0,0),tf::Vector3(0,0,0),10.01,true,10.01,.01,true,true);
  EXPECT_FALSE(wrong.valid);
}
