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
  bool fault() { return controller.contact_guard_.state()==hih_control::ContactGuard::FAULT; }
  double reference() { return controller.normal_force_reference_; }
  double baseline() { return controller.admittance_.referenceOffset()[0]; }
  double correction() { return controller.admittance_.offset()[0]; }
  double total() { return controller.admittance_.totalOffset()[0]; }
};

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
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"hih_test_follower_control");
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
