#include <gtest/gtest.h>
#include <hih/control/hih_admittance.h>
#include <hih/control/hih_contact_wrench.h>
#include <hih/control/hih_contact_guard.h>
#include <hih/control/hih_force_release.h>
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

TEST(ContactWrench, RearPlateForceAndMomentSigns)
{
  tf::Matrix3x3 identity; identity.setIdentity();
  const tf::Vector3 n(-1,0,0), lever(-0.26,0,0), force(10,2,-1);
  const auto measured = wrenchAtContact(force, lever.cross(force), identity, tf::Transform(identity,lever));
  EXPECT_NEAR(measured.torque.length(),0,1e-12);
  for (double fn : {5.,10.,15.}) {
    auto e=contactWrenchErrorWorld({tf::Vector3(fn,0,0),tf::Vector3(0,0,0)},identity,n,10);
    EXPECT_DOUBLE_EQ(e[0],fn-10); // Low compression advances -X; high retreats +X.
  }
  tf::Matrix3x3 turned; turned.setRPY(0,0,1.5707963267948966);
  auto e=contactWrenchErrorWorld({tf::Vector3(5,0,0),tf::Vector3(0,0,0)},turned,n,10);
  EXPECT_NEAR(e[0],0,1e-12); EXPECT_NEAR(e[1],-5,1e-12);
  auto compensation=contactCenterCompensation(identity,turned,lever);
  EXPECT_NEAR((compensation+turned*lever-lever).length(),0,1e-12);
}

TEST(Admittance, MovingReferenceCarriesTranslationAndNinetyDegreeYawWithoutLocalSaturation)
{
  for (int axis=0; axis<4; ++axis) {
    AdmittanceParameters p; p.moving_reference=true; p.filter_cutoff_hz=0; p.deadband.fill(0);
    p.max_travel.fill(3); Admittance4D model(p);
    Vector4 force{{0,0,0,0}}; force[axis]=axis==3 ? .3 : 2;
    const double goal=axis==3 ? 1.5707963267948966 : .25;
    double previous=0;
    for (int i=0; i<40000 && model.totalOffset()[axis]<goal; ++i) {
      ASSERT_TRUE(model.step(force,.005));
      const double total=model.totalOffset()[axis];
      EXPECT_GE(total,previous); EXPECT_LE(total-previous,p.max_velocity[axis]*.005+1e-12);
      EXPECT_LT(std::abs(model.offset()[axis]),p.max_offset[axis]); previous=total;
    }
    EXPECT_GE(model.totalOffset()[axis],goal);
    EXPECT_GT(model.referenceOffset()[axis],0);
    const auto before=model.totalOffset(); model.hold(); EXPECT_EQ(model.totalOffset(),before);
  }
}
TEST(Admittance, TravelLimitRejectsStepWithoutReferenceJump)
{
  AdmittanceParameters p; p.moving_reference=true; p.max_travel.fill(.2);
  p.max_offset.fill(.1); p.filter_cutoff_hz=0; p.deadband.fill(0); Admittance4D model(p);
  bool rejected=false;
  for(int i=0;i<10000;++i) {
    const auto before=model.totalOffset();
    if(!model.step({{2,0,0,0}},.01)) { EXPECT_EQ(model.totalOffset(),before); rejected=true; break; }
  }
  EXPECT_TRUE(rejected); EXPECT_LT(model.totalOffset()[0],.2);
}
TEST(Admittance, MovingReferenceRequiresZeroSpringAndFiniteTravel)
{
  AdmittanceParameters p; p.moving_reference=true; p.stiffness[0]=1;
  EXPECT_THROW(Admittance4D model(p),std::invalid_argument);
  p.stiffness[0]=0; p.max_travel[0]=.01;
  EXPECT_THROW(Admittance4D model(p),std::invalid_argument);
}
TEST(ContactGuard, RequiresCompressiveNormalBeforeAnyForceRamp)
{
  ContactGuard g; EXPECT_EQ(g.step(1,true,true,0),ContactGuard::WAITING);
  EXPECT_EQ(g.step(2,true,true,-5),ContactGuard::WAITING);
  EXPECT_EQ(g.step(3,true,true,1),ContactGuard::ACTIVE);
}
TEST(ContactGuard, LostContactPausesImmediatelyThenLatchesUntilReset)
{
  ContactGuard g; g.step(1,true,true,1);
  EXPECT_EQ(g.step(2,true,true,0),ContactGuard::HOLD);
  EXPECT_EQ(g.step(2.25,true,true,0),ContactGuard::FAULT);
  EXPECT_EQ(g.step(3,true,true,10),ContactGuard::FAULT);
  EXPECT_EQ(g.status(),"contact_lost_hold"); g.reset();
  EXPECT_EQ(g.step(4,true,true,10),ContactGuard::ACTIVE);
}
TEST(ContactGuard, BriefLossCanResumeWithoutLosingAcquisition)
{
  ContactGuard g; g.step(1,true,true,1); g.step(2,true,true,0);
  EXPECT_EQ(g.step(2.1,true,true,1),ContactGuard::ACTIVE); EXPECT_TRUE(g.acquired());
  EXPECT_EQ(g.step(3,true,true,0),ContactGuard::HOLD);
}
TEST(ContactGuard, ControlGapPreservesContactAndOverloadStillWins)
{
  ContactGuard g; g.step(1,true,true,1);
  EXPECT_EQ(g.step(2,false,true,10),ContactGuard::HOLD); EXPECT_TRUE(g.acquired());
  EXPECT_EQ(g.step(2.01,true,true,10),ContactGuard::ACTIVE);
  EXPECT_EQ(g.step(3,false,true,20),ContactGuard::FAULT);
  EXPECT_EQ(g.status(),"normal_force_limit_hold");
}
TEST(ContactGuard, OverloadIsCheckedEvenBeforeFirstContact)
{
  for(double f : {-21.,20.,21.}) {
    ContactGuard g; EXPECT_EQ(g.step(1,true,true,f),ContactGuard::FAULT);
  }
}
TEST(ContactGuard, StaleMobHoldsThenLatchesAndRejectsNonfiniteForce)
{
  ContactGuard g; g.step(1,true,true,1);
  EXPECT_EQ(g.step(2,true,false,10),ContactGuard::HOLD);
  EXPECT_EQ(g.step(2.6,true,false,10),ContactGuard::FAULT);
  EXPECT_EQ(g.status(),"mob_timeout_hold");
  g.reset(); EXPECT_EQ(g.step(3,true,true,std::numeric_limits<double>::quiet_NaN()),ContactGuard::WAITING);
}
TEST(ContactGuard, ThresholdHysteresisRetainsEstablishedContact)
{
  ContactGuard g; g.step(1,true,true,1);
  EXPECT_EQ(g.step(2,true,true,.6),ContactGuard::ACTIVE);
  EXPECT_EQ(g.step(3,true,true,.3),ContactGuard::HOLD);
}
TEST(ContactGuard, RejectsInvalidProtectionSettings)
{
  ContactGuardParameters p; p.release_force=2; ContactGuard g;
  EXPECT_THROW(g.configure(p),std::invalid_argument);
}

TEST(ForceRelease, ReferenceRampsDownAndNeverRestartsOnRepeatedRequest)
{
  ForceRelease r; ASSERT_TRUE(r.request(1,10,10));
  ASSERT_TRUE(r.advance(1.1,.1,true,10,20,.1)); EXPECT_NEAR(r.reference(),9.9,1e-12);
  EXPECT_FALSE(r.request(1.2,10,10));
  ASSERT_TRUE(r.advance(1.2,.1,true,2,20,.1)); EXPECT_NEAR(r.reference(),1.9,1e-12);
  ASSERT_TRUE(r.advance(1.3,.1,true,0,20,.1)); EXPECT_DOUBLE_EQ(r.reference(),0);
  EXPECT_EQ(r.phase(),ForceRelease::RELEASING);
}
TEST(ForceRelease, RequestCannotIncreaseExistingForceReference)
{
  ForceRelease a,b; a.request(1,10,2); EXPECT_DOUBLE_EQ(a.reference(),2);
  b.request(1,0,10); EXPECT_DOUBLE_EQ(b.reference(),0);
}
TEST(ForceRelease, ZeroReferenceRequiresMeasuredUnloadingAndMotionDwell)
{
  ForceReleaseParameters p; p.settle_time=.1; ForceRelease r; r.configure(p); r.request(1,0,0);
  ASSERT_TRUE(r.advance(1.1,.025,true,0,20,.1));
  r.observe(1.1,0,2,0,true,0,0,0,0,0,0); EXPECT_EQ(r.phase(),ForceRelease::RELEASING);
  r.observe(1.2,0,0,0,false,0,0,0,0,0,0); EXPECT_EQ(r.phase(),ForceRelease::RELEASING);
  r.observe(1.3,0,0,0,true,.2,0,0,0,0,0); EXPECT_EQ(r.phase(),ForceRelease::RELEASING);
  r.observe(1.4,0,0,0,true,0,0,0,0,0,0);
  r.observe(1.55,0,0,0,true,0,0,0,0,0,0); EXPECT_EQ(r.phase(),ForceRelease::READY);
  r.complete(); EXPECT_EQ(r.phase(),ForceRelease::HOVER); EXPECT_EQ(r.status(),"released_hover");
  EXPECT_FALSE(r.request(2,10,10));
}
TEST(ForceRelease, StaleAndControlGapPauseThenFailWithoutForceReduction)
{
  ForceRelease r; r.request(1,10,10);
  EXPECT_FALSE(r.advance(2,.2,true,10,20,.1)); EXPECT_DOUBLE_EQ(r.reference(),10);
  EXPECT_FALSE(r.advance(2.6,.025,false,0,20,.1)); EXPECT_EQ(r.phase(),ForceRelease::FAILED);
  EXPECT_EQ(r.status(),"release_data_timeout_hold");
}
TEST(ForceRelease, OverloadAndTimeoutNeverProduceSuccess)
{
  ForceRelease r; r.request(1,10,10); r.advance(2,.025,true,21,20,.1);
  EXPECT_EQ(r.status(),"release_force_limit_hold"); r.complete(); EXPECT_EQ(r.phase(),ForceRelease::FAILED);
  r.reset(); r.request(1,0,0); r.advance(32,.025,true,0,20,.1);
  EXPECT_EQ(r.status(),"release_timeout_hold");
}
TEST(ForceRelease, MotionBoundFailsInsteadOfUnlimitedWithdrawal)
{
  ForceRelease r; r.request(1,10,10);r.advance(2,.025,true,10,20,.1);
  r.observe(2,10,10,0,true,0,0,0,0,.16,0); EXPECT_EQ(r.status(),"release_motion_limit_hold");
}
TEST(ForceRelease, InvalidParametersAreRejected)
{
  ForceReleaseParameters p; p.rate=0; ForceRelease r;
  EXPECT_THROW(r.configure(p),std::invalid_argument);
}

TEST(ForceRelease, ClearingOldInputPreservesAccumulatedReference)
{
  Admittance4D model; for(int i=0;i<20;++i) ASSERT_TRUE(model.step({{-2,0,0,0}},.025));
  const auto before=model.totalOffset(); model.hold(true);
  EXPECT_EQ(model.totalOffset(),before);
  for(double value:model.filteredWrench()) EXPECT_DOUBLE_EQ(value,0);
  for(double value:model.velocity()) EXPECT_DOUBLE_EQ(value,0);
}
