#include <gtest/gtest.h>
#include <hih/control/hih_cop_adjustment.h>
#include <hih/control/hih_role_parameters.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <ros/master.h>
#include <atomic>
#include <mutex>

using hih_control::HihCopAdjustment;
using Snapshot = aerial_robot_navigation::BaseNavigator::TargetSnapshot;

namespace
{
Snapshot reference()
{
  Snapshot t;
  t.pos = tf::Vector3(1, 2, 3); t.vel.setZero(); t.acc.setZero();
  t.rpy = tf::Vector3(.1, -.1, .2); t.omega.setZero(); t.ang_acc.setZero();
  return t;
}

class CopRos : public ::testing::Test
{
protected:
  std::unique_ptr<ros::NodeHandle> nh;
  HihCopAdjustment model;
  void SetUp() override
  {
    nh = std::make_unique<ros::NodeHandle>(std::string("/hih_cop_test/") +
      ::testing::UnitTest::GetInstance()->current_test_info()->name());
    // Match the real launch -> role override -> active controller path.
    XmlRpc::XmlRpcValue role;
    role["wrench_estimate_flag"] = true;
    role["cop_adjustment"]["enabled"] = true;
    role["cop_adjustment"]["safe_y"] = .05;
    role["cop_adjustment"]["safe_z"] = .05;
    role["cop_adjustment"]["filter_cutoff_hz"] = 0.0;
    role["cop_adjustment"]["debug_period"] = .001;
    role["cop_adjustment"]["yaw"]["enabled"] = true;
    role["cop_adjustment"]["z"]["enabled"] = true;
    nh->setParam("hih/follower/controller", role);
    hih_control::applyRoleParameters(*nh, "follower");
  }
  Snapshot update()
  {
    auto t = reference();
    const auto now = ros::Time::now();
    model.update(t, tf::Vector3(-10, 0, 0), tf::Vector3(.2, -.8, .8),
      now, true, true, true, now);
    return t;
  }
};
}  // namespace

TEST_F(CopRos, RoleYamlEnablesBothAxesAndPublishesValidDiagnostics)
{
  model.initialize(*nh, tf::Vector3(1, 0, 0), "world");
  std::mutex mutex;
  geometry_msgs::PointStamped received_raw, received_command;
  bool received_valid = false;
  std::atomic<bool> have_raw{false}, have_command{false}, have_valid{false};
  auto raw = nh->subscribe<geometry_msgs::PointStamped>("hih/cop/raw", 1,
    [&](const geometry_msgs::PointStampedConstPtr& msg) {
      std::lock_guard<std::mutex> lock(mutex); received_raw = *msg; have_raw.store(true);
    });
  auto command = nh->subscribe<geometry_msgs::PointStamped>("hih/cop/command_position", 1,
    [&](const geometry_msgs::PointStampedConstPtr& msg) {
      std::lock_guard<std::mutex> lock(mutex); received_command = *msg; have_command.store(true);
    });
  auto valid = nh->subscribe<std_msgs::Bool>("hih/cop/valid", 1,
    [&](const std_msgs::BoolConstPtr& msg) {
      std::lock_guard<std::mutex> lock(mutex); received_valid = msg->data; have_valid.store(true);
    });
  const auto deadline = ros::WallTime::now() + ros::WallDuration(3);
  while (ros::WallTime::now() < deadline &&
         (!have_raw.load() || !have_command.load() || !have_valid.load() || model.state().yaw_offset == 0))
  { update(); ros::WallDuration(.01).sleep(); }
  ASSERT_TRUE(have_raw.load()); ASSERT_TRUE(have_command.load()); ASSERT_TRUE(have_valid.load());
  raw.shutdown(); command.shutdown(); valid.shutdown();
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_TRUE(received_valid);
    EXPECT_NEAR(received_raw.point.y, .08, 1e-12); EXPECT_NEAR(received_raw.point.z, .08, 1e-12);
    EXPECT_EQ(received_raw.header.frame_id, nh->getNamespace().substr(1) + "/contact_point");
    EXPECT_EQ(received_command.header.frame_id, "world");
  }
  EXPECT_GT(model.state().yaw_offset, 0); EXPECT_GT(model.state().z_offset, 0);
  ros::master::V_TopicInfo topics;
  ASSERT_TRUE(ros::master::getTopics(topics));
  for (const auto& topic : topics)
  {
    EXPECT_EQ(topic.name.find("/uav/nav"), std::string::npos);
    EXPECT_EQ(topic.name.find("/four_axes/command"), std::string::npos);
  }
}

TEST_F(CopRos, CachedAxisAndMasterFlagsReturnOffsetsWithoutRestart)
{
  model.initialize(*nh, tf::Vector3(1, 0, 0), "world");
  for (int i = 0; i < 15; ++i) { update(); ros::WallDuration(.01).sleep(); }
  ASSERT_GT(model.state().yaw_offset, 0); const double z_before = model.state().z_offset;
  nh->setParam("controller/cop_adjustment/yaw/enabled", false);
  for (int i = 0; i < 150 && model.state().yaw_offset != 0; ++i)
  { update(); ros::WallDuration(.01).sleep(); }
  EXPECT_DOUBLE_EQ(model.state().yaw_offset, 0);
  EXPECT_GT(model.state().z_offset, z_before);
  nh->setParam("controller/cop_adjustment/enabled", false);
  for (int i = 0; i < 200 && model.state().applying; ++i)
  { update(); ros::WallDuration(.01).sleep(); }
  EXPECT_DOUBLE_EQ(model.state().z_offset, 0); EXPECT_FALSE(model.state().applying);
}

TEST_F(CopRos, DisabledMasterPreservesTheAdmittanceReferenceExactly)
{
  nh->setParam("controller/cop_adjustment/enabled", false);
  model.initialize(*nh, tf::Vector3(1, 0, 0), "world");
  for (int i = 0; i < 5; ++i)
  {
    const auto after = update(); const auto before = reference();
    EXPECT_EQ(after.pos, before.pos); EXPECT_EQ(after.vel, before.vel); EXPECT_EQ(after.rpy, before.rpy);
    EXPECT_EQ(after.omega, before.omega); ros::WallDuration(.01).sleep();
  }
}

TEST_F(CopRos, InvalidYamlTypeIsRejectedAtStartup)
{
  nh->setParam("controller/cop_adjustment/yaw/gain", "invalid");
  EXPECT_THROW(model.initialize(*nh, tf::Vector3(1, 0, 0), "world"), std::invalid_argument);
}

TEST_F(CopRos, MissingMobIsRejectedWhenAdjustmentIsEnabled)
{
  nh->setParam("controller/wrench_estimate_flag", false);
  EXPECT_THROW(model.initialize(*nh, tf::Vector3(1, 0, 0), "world"), std::invalid_argument);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hih_cop_integration_test");
  ::testing::InitGoogleTest(&argc, argv);
  ros::AsyncSpinner spinner(1); spinner.start();
  const int result = RUN_ALL_TESTS();
  spinner.stop(); ros::shutdown();
  return result;
}
