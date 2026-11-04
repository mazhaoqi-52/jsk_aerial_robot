#pragma once
#include <hih/control/hih_controller.h>
#include <hih/control/hih_admittance.h>
#include <hih/control/hih_cop_adjustment.h>
#include <aerial_robot_control/control/base/target_reference_modifier.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <atomic>

namespace aerial_robot_control
{
class HihFollowerController : public HihController, public TargetReferenceModifier
{
public:
  HihFollowerController();
  ~HihFollowerController() override;
  bool update() override;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
    double ctrl_loop_rate) override;
  void modifyTargetSnapshot(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target) override;

protected:
  void externalWrenchEstimate() override;
  void reset() override;

private:
  bool copFlightAllowed() const;
  void modifyAdmittanceTarget(aerial_robot_navigation::BaseNavigator::TargetSnapshot& target);
  void publishDebug(const aerial_robot_navigation::BaseNavigator::TargetSnapshot& nominal,
                    const aerial_robot_navigation::BaseNavigator::TargetSnapshot& command,
                    const std::string& status, bool applying, const ros::Time& now);
  hih_control::Admittance4D admittance_;
  hih_control::HihCopAdjustment cop_adjustment_;
  bool admittance_applying_ = false;
  bool cop_flight_allowed_last_ = false;
  std::atomic<bool> ready_{false};
  bool enabled_ = false;
  bool apply_velocity_ = true;
  double sample_timeout_ = 0.15;
  double max_dt_ = 0.1;
  double debug_period_ = 0.05;
  double normal_force_desired_ = 10.0;
  double normal_force_max_ = 20.0;
  double normal_force_rate_ = 1.0;
  double contact_threshold_ = 0.8;
  double normal_force_reference_ = 0.0;
  bool contact_acquired_ = false;
  tf::Vector3 normal_contact_{1, 0, 0};
  tf::Vector3 contact_offset_cog_{0, 0, 0};
  tf::Matrix3x3 contact_to_world_;
  std::string world_frame_ = "world";
  ros::Time last_update_, last_debug_;
  std::mutex sample_mutex_;
  // Force/moment about the contact center, expressed in contact axes.
  tf::Vector3 sample_force_contact_{0, 0, 0};
  tf::Vector3 sample_torque_contact_{0, 0, 0};
  ros::Time sample_stamp_;
  bool sample_valid_ = false;
  boost::shared_ptr<aerial_robot_navigation::HihNavigator> hih_nav_;
  boost::shared_ptr<HihRobotModel> hih_model_;
  ros::Publisher nominal_pub_, command_pub_, offset_pub_, velocity_pub_;
  ros::Publisher wrench_pub_, yaw_pub_, active_pub_, status_pub_;
  ros::Publisher force_reference_pub_, measured_normal_pub_;
};
}
