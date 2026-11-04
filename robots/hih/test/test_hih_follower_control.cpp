#include <gtest/gtest.h>
#include <hih/control/hih_follower_controller.h>

namespace aerial_robot_control
{
class TestNavigator : public aerial_robot_navigation::HihNavigator
{
public:
  TestNavigator()
  {
    force_landing_flag_=false;
    xy_control_mode_=aerial_robot_navigation::POS_CONTROL_MODE;
    setNaviState(aerial_robot_navigation::HOVER_STATE);
  }
};

// Exercise the actual reference hook without initializing the motor controller.
class HihFollowerControlTest : public ::testing::Test
{
protected:
  using Snapshot=aerial_robot_navigation::BaseNavigator::TargetSnapshot;
  HihFollowerController controller;
  boost::shared_ptr<TestNavigator> nav;
  void SetUp() override
  {
    nav=boost::make_shared<TestNavigator>();
    controller.navigator_=nav; controller.hih_nav_=nav;
    controller.ready_.store(true); controller.enabled_=true;
    controller.contact_to_world_.setIdentity();
    controller.contact_offset_cog_.setValue(-.26,0,0);
    controller.normal_contact_.setValue(-1,0,0);
    hih_control::AdmittanceParameters p; p.moving_reference=true;
    p.filter_cutoff_hz=0; p.deadband.fill(0);
    controller.admittance_.configure(p);
    // Debug publishing is tested separately; these tests never create flight publishers.
    controller.debug_period_=1e6; controller.last_debug_=ros::Time::now();
  }
  Snapshot tick(double dt, double fn, bool fresh=true, double nominal_x=0)
  {
    controller.last_update_=ros::Time::now()-ros::Duration(dt);
    controller.sample_valid_=fresh;
    controller.sample_stamp_=ros::Time::now();
    controller.sample_force_contact_.setValue(fn,0,0);
    controller.sample_torque_contact_.setZero();
    nav->setTargetPos(nominal_x,0,1);
    auto target=nav->getTargetSnapshot();
    controller.modifyTargetSnapshot(target);
    return target;
  }
  Snapshot load()
  {
    Snapshot target;
    for (int i=0;i<160;++i) target=tick(.025,1);
    EXPECT_LT(target.pos.x(),-.15); // Past the old fixed-reference bound.
    return target;
  }
  void expectHold(const Snapshot& before, const Snapshot& after)
  {
    EXPECT_NEAR((before.pos-after.pos).length(),0,1e-10);
    EXPECT_NEAR((before.rpy-after.rpy).length(),0,1e-10);
    EXPECT_DOUBLE_EQ(after.vel.length(),0); EXPECT_DOUBLE_EQ(after.omega.length(),0);
    EXPECT_DOUBLE_EQ(after.acc.length(),0); EXPECT_DOUBLE_EQ(after.ang_acc.length(),0);
  }
  void requestRelease() { controller.release_requested_.store(true); }
  void configureRelease()
  {
    hih_control::ForceReleaseParameters p; p.settle_time=.03; controller.force_release_.configure(p);
    controller.release_motion_valid_=true; controller.release_speed_=controller.release_omega_=0;
    controller.pid_controllers_.resize(6);
    for(int i=0;i<6;++i) controller.pid_controllers_[i].setErrI(i+.5);
  }
  hih_control::ForceRelease::Phase releasePhase() { return controller.force_release_.phase(); }
  double integral(int i) { return controller.pid_controllers_[i].getErrI(); }
  double cop_yaw_=0,cop_z_=0;
  Snapshot finalPreHandoffReference()
  {
    auto t=controller.held_target_; t.pos.setZ(t.pos.z()+cop_z_);
    t.rpy.setZ(angles::normalize_angle(t.rpy.z()+cop_yaw_)); return t;
  }
  void seedCop()
  {
    hih_control::CopAdjustmentParameters p; p.enabled=true;
    controller.cop_adjustment_.configure(p,tf::Vector3(-1,0,0));
    controller.cop_adjustment_.step(tf::Vector3(10,0,0),tf::Vector3(0,1,-1),10,true,10,.025,true,true);
    cop_yaw_=controller.cop_adjustment_.state().yaw_offset; cop_z_=controller.cop_adjustment_.state().z_offset;
  }
  void observerBeforeFirstTimestamp()
  {
    nav->setNaviState(aerial_robot_navigation::ARM_OFF_STATE);
    controller.prev_est_wrench_timestamp_=0;
    controller.externalWrenchEstimate(); // No estimator/IMU queue exists in this fixture.
    EXPECT_FALSE(controller.sample_valid_);
  }
  bool fault() { return controller.contact_guard_.state()==hih_control::ContactGuard::FAULT; }
  double reference() { return controller.normal_force_reference_; }
  double baseline() { return controller.admittance_.referenceOffset()[0]; }
  double correction() { return controller.admittance_.offset()[0]; }
  double total() { return controller.admittance_.totalOffset()[0]; }
};

TEST_F(HihFollowerControlTest, ObserverStartupDoesNotReadUnavailableImuTimestamp)
{
  EXPECT_NO_THROW(observerBeforeFirstTimestamp());
}
TEST_F(HihFollowerControlTest, InternalBaselineMovesWithoutWritingNavigator)
{
  const auto target=load(); EXPECT_FALSE(fault());
  EXPECT_DOUBLE_EQ(nav->getTargetPos().x(),0);
  EXPECT_LT(baseline(),0);
  EXPECT_LT(std::abs(correction()),.15);
}
TEST_F(HihFollowerControlTest, ControlGapHoldsCommandAndForceReference)
{
  const auto before=load(); const auto force=reference();
  expectHold(before,tick(.2,1,true,99));
  EXPECT_DOUBLE_EQ(reference(),force); EXPECT_FALSE(fault());
}
TEST_F(HihFollowerControlTest, ContactLossNeverCommandsAutomaticReapproach)
{
  const auto before=load(); const auto force=reference();
  expectHold(before,tick(.025,0,true,99));
  EXPECT_DOUBLE_EQ(reference(),force);
  ros::WallDuration(.23).sleep();
  expectHold(before,tick(.025,0,true,100)); EXPECT_TRUE(fault());
  expectHold(before,tick(.025,10,true,101)); // No automatic resume after latched loss.
}
TEST_F(HihFollowerControlTest, OverloadHoldsDespiteNewKeyboardTarget)
{
  const auto before=load(); const auto force=reference();
  expectHold(before,tick(.025,21,true,99)); EXPECT_TRUE(fault());
  EXPECT_DOUBLE_EQ(reference(),force);
}
TEST_F(HihFollowerControlTest, MissingForceHoldsRatherThanTreatingItAsZeroForce)
{
  const auto before=load(); const auto force=reference();
  expectHold(before,tick(.025,0,false,99));
  EXPECT_DOUBLE_EQ(reference(),force);
}
TEST_F(HihFollowerControlTest, LandingClearsFaultAndStoredDisplacement)
{
  load(); tick(.025,21); ASSERT_TRUE(fault());
  nav->setNaviState(aerial_robot_navigation::LAND_STATE);
  const auto target=tick(.025,0,true,.4);
  EXPECT_DOUBLE_EQ(target.pos.x(),.4); EXPECT_FALSE(fault());
  EXPECT_DOUBLE_EQ(total(),0);
}
TEST_F(HihFollowerControlTest, ReleaseTransfersReferenceWithoutPoseJumpOrPidReset)
{
  load(); configureRelease(); seedCop(); requestRelease();
  tick(.025,1);
  const auto before=tick(.025,0); // Intentional contact loss must not latch a normal contact-loss fault.
  ros::WallDuration(.05).sleep(); const auto after=tick(.025,0);
  ASSERT_EQ(releasePhase(),hih_control::ForceRelease::HOVER); EXPECT_FALSE(fault());
  EXPECT_LE((after.pos-before.pos).length(),.005*.025+1e-9); // Ordinary bounded motion between ticks.
  const auto handoff=finalPreHandoffReference();
  EXPECT_NEAR((after.pos-handoff.pos).length(),0,1e-10);
  EXPECT_NEAR((after.rpy-handoff.rpy).length(),0,1e-10);
  EXPECT_NEAR((nav->getTargetPos()-after.pos).length(),0,1e-8);
  EXPECT_NEAR((nav->getTargetRPY()-after.rpy).length(),0,1e-8);
  for(int i=0;i<6;++i) EXPECT_DOUBLE_EQ(integral(i),i+.5);
  EXPECT_DOUBLE_EQ(reference(),0); EXPECT_DOUBLE_EQ(total(),0);
  const auto manual=tick(.025,10,true,.7); // No automatic force reacquisition; manual hover remains usable.
  EXPECT_DOUBLE_EQ(manual.pos.x(),.7); EXPECT_EQ(releasePhase(),hih_control::ForceRelease::HOVER);
}
TEST_F(HihFollowerControlTest, ReleaseCannotReportSuccessWithPressureOrMissingState)
{
  load(); configureRelease(); requestRelease();
  for(int i=0;i<10;++i) tick(.025,1);
  EXPECT_EQ(releasePhase(),hih_control::ForceRelease::RELEASING);
  EXPECT_GT(reference(),0);
}
TEST_F(HihFollowerControlTest, ReleaseStaleDataPreservesLastReferenceAndNeverAnnouncesHover)
{
  load(); configureRelease(); requestRelease(); const auto before=tick(.025,1); const double force=reference();
  expectHold(before,tick(.025,0,false,99)); EXPECT_DOUBLE_EQ(reference(),force);
  EXPECT_NE(releasePhase(),hih_control::ForceRelease::HOVER);
}
TEST_F(HihFollowerControlTest, ReleaseKeepsOverloadProtectionAndCannotClearExistingFault)
{
  load(); configureRelease(); requestRelease(); tick(.025,21);
  EXPECT_TRUE(fault()); EXPECT_EQ(releasePhase(),hih_control::ForceRelease::FAILED);
  requestRelease(); tick(.025,0); EXPECT_TRUE(fault());
}
TEST_F(HihFollowerControlTest, LandingCancelsReleaseAndResetsTheNextFlight)
{
  load(); configureRelease(); requestRelease(); tick(.025,1);
  nav->setNaviState(aerial_robot_navigation::LAND_STATE); tick(.025,0);
  EXPECT_EQ(releasePhase(),hih_control::ForceRelease::OFF);
}

}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"hih_test_follower_control");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
